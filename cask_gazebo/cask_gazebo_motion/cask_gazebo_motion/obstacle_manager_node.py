"""Shared obstacle snapshot and RViz visualization for mobile robots."""

import math
import threading
import json
import queue
import re
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class SharedObstacleState:
    """Give any Robot Skill Server the same versioned obstacle snapshot."""

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.positions = ()
        self.revision = 0
        self.subscription = node.create_subscription(
            PoseArray, '/cask/obstacles', self.on_snapshot, 10
        )

    def on_snapshot(self, message):
        if message.header.frame_id != 'world':
            self.node.get_logger().warning(
                'Ignoring shared obstacles outside world frame'
            )
            return
        positions = tuple(
            (pose.position.x, pose.position.y, pose.position.z)
            for pose in message.poses
        )
        if not all(all(math.isfinite(value) for value in point)
                   for point in positions):
            self.node.get_logger().warning('Ignoring invalid obstacle snapshot')
            return
        with self.lock:
            if positions == self.positions:
                return
            self.positions = positions
            self.revision += 1
        self.node.get_logger().info(
            f'Shared obstacle snapshot received: {len(positions)} obstacle(s)'
        )

    def snapshot(self):
        with self.lock:
            return self.revision, self.positions

    def ground_snapshot(self):
        """Project shared 3D positions onto the ground navigation plane."""
        revision, positions = self.snapshot()
        return revision, tuple((x, y) for x, y, _ in positions)


class ObstacleManager(Node):
    def __init__(self):
        super().__init__('cask_obstacle_manager')
        self.declare_parameter('obstacle_radius', 0.45)
        self.radius = float(self.get_parameter('obstacle_radius').value)
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('obstacle_radius must be positive')

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(
            PoseArray, '/cask/obstacles', latched
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/cask/obstacle_markers', latched
        )
        self.update_sub = self.create_subscription(
            PoseArray, '/cask/obstacle_updates', self.on_update, 10
        )
        self.declare_parameter('gazebo_physics', False)
        self.declare_parameter('gazebo_world', 'empty')
        self.declare_parameter('obstacle_height', 0.8)
        self.gazebo_physics = bool(self.get_parameter('gazebo_physics').value)
        self.gazebo_world = str(self.get_parameter('gazebo_world').value)
        self.height = float(self.get_parameter('obstacle_height').value)
        if not re.fullmatch(r'[A-Za-z0-9_-]+', self.gazebo_world):
            raise ValueError('gazebo_world must be a simple world name')
        if not math.isfinite(self.height) or self.height <= 0:
            raise ValueError('obstacle_height must be positive')
        self.physics_lock = threading.Lock()
        self.physics_pending = None
        self.physics_running = False
        self.physics_models = {}
        self.physics_model_serial = 0
        self.physics_results = queue.Queue()
        self.physics_stop = threading.Event()
        self.physics_timer = self.create_timer(0.1, self.apply_physics_results)
        self.positions = ()
        self.publish_timer = self.create_timer(0.5, self.publish_state)
        self.publish_markers()
        self.get_logger().info(
            'Obstacle Manager: updates=/cask/obstacle_updates, '
            'snapshot=/cask/obstacles, markers=/cask/obstacle_markers'
        )

    def on_update(self, message):
        if message.header.frame_id != 'world':
            self.get_logger().warning('Obstacle update must use world frame')
            return
        positions = tuple(
            (pose.position.x, pose.position.y, pose.position.z)
            for pose in message.poses
        )
        if not all(all(math.isfinite(value) for value in point)
                   for point in positions):
            self.get_logger().warning('Ignoring non-finite obstacle position')
            return
        if self.gazebo_physics:
            self.request_physics_update(tuple(dict.fromkeys(positions)))
            return
        if positions == self.positions:
            return
        self.positions = positions
        self.publish_state()
        self.publish_markers()
        self.get_logger().info(
            f'Shared obstacle map updated: {len(positions)} obstacle(s)'
        )

    @staticmethod
    def physical_obstacle_sdf(name, radius, height):
        root = ET.Element('sdf', version='1.7')
        model = ET.SubElement(root, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='obstacle_link')
        # Model pose is the ground reference supplied in PoseArray.
        ET.SubElement(link, 'pose').text = f'0 0 {height/2} 0 0 0'
        for kind in ('visual', 'collision'):
            part = ET.SubElement(link, kind, name=kind)
            cylinder = ET.SubElement(ET.SubElement(part, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(radius)
            ET.SubElement(cylinder, 'length').text = str(height)
            if kind == 'visual':
                material = ET.SubElement(part, 'material')
                ET.SubElement(material, 'ambient').text = '0.9 0.05 0.02 1'
                ET.SubElement(material, 'diffuse').text = '0.9 0.05 0.02 1'
        return ET.tostring(root, encoding='unicode')

    def gazebo_service(self, operation, request_type, request):
        result = subprocess.run([
            'ign', 'service', '-s', f'/world/{self.gazebo_world}/{operation}',
            '--reqtype', request_type, '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '3000', '--req', request],
            capture_output=True, text=True, timeout=6)
        output = result.stdout + result.stderr
        if result.returncode or not re.search(r'\bdata:\s*true\b', output):
            raise RuntimeError(f'Gazebo {operation} failed: {output.strip()}')

    def request_physics_update(self, positions):
        with self.physics_lock:
            self.physics_pending = positions
            if self.physics_running:
                return
            self.physics_running = True
        threading.Thread(target=self.physics_worker, daemon=True).start()

    def physics_worker(self):
        while not self.physics_stop.is_set():
            with self.physics_lock:
                requested = self.physics_pending
                self.physics_pending = None
                if requested is None:
                    self.physics_running = False
                    return
            error = None
            try:
                # Add first, then remove obsolete models. No shell expansion.
                for point in requested:
                    if point in self.physics_models:
                        continue
                    name = f'cask_obstacle_{self.physics_model_serial}'
                    self.physics_model_serial += 1
                    sdf = self.physical_obstacle_sdf(name, self.radius, self.height)
                    x, y, z = point
                    request = (f'sdf: {json.dumps(sdf)} name: {json.dumps(name)} '
                               f'allow_renaming: false pose: {{position: {{x: {x} y: {y} z: {z}}}}}')
                    self.gazebo_service('create', 'ignition.msgs.EntityFactory', request)
                    self.physics_models[point] = name
                for point, name in tuple(self.physics_models.items()):
                    if point in requested:
                        continue
                    self.gazebo_service('remove', 'ignition.msgs.Entity',
                                        f'name: {json.dumps(name)} type: MODEL')
                    del self.physics_models[point]
            except Exception as exception:
                error = str(exception)
            # Publish only the accepted Gazebo model set, including any partial
            # success. A failed creation must not introduce an invisible obstacle.
            self.physics_results.put((tuple(self.physics_models), error))
        with self.physics_lock:
            self.physics_running = False

    def apply_physics_results(self):
        while True:
            try:
                positions, error = self.physics_results.get_nowait()
            except queue.Empty:
                return
            self.positions = positions
            self.publish_state()
            self.publish_markers()
            if error:
                self.get_logger().error('GAZEBO_OBSTACLE_SYNC_FAILED: ' + error)
            else:
                self.get_logger().info(
                    f'GAZEBO_OBSTACLES_READY: {len(positions)} obstacle(s); '
                    'visible models with collision; planning snapshot synchronized')

    def publish_state(self):
        message = PoseArray()
        message.header.frame_id = 'world'
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in self.positions:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self.state_pub.publish(message)

    def publish_markers(self):
        message = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        for index, (x, y, z) in enumerate(self.positions):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = clear.header.stamp
            marker.ns = 'cask_obstacles'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z + self.height/2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * self.radius
            marker.scale.y = 2 * self.radius
            marker.scale.z = self.height
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.0
            marker.color.a = 0.85
            message.markers.append(marker)
        self.marker_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.physics_stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
