"""Physical scene owned by Component Manager; no robot control or task execution."""
from collections import deque
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

ABOX = 'http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#'
RDF_TYPE = 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type'


def load_scene(path):
    scene = json.loads(Path(path).read_text(encoding='utf-8'))
    if tuple(scene['robots']) != ('crane', 'jekko', 'innok223'):
        raise ValueError('Final scene requires Crane, Jekko and Innok223')
    for pose in [*scene['robots'].values(), *scene['locations'].values(),
                 scene['cargo']['initial_pose']]:
        if len(pose) != 4 or not all(math.isfinite(v) for v in pose):
            raise ValueError('Scene poses must be finite [x, y, z, yaw]')
    for size in (scene['cargo']['size'], scene['workbench']['size'], scene['site']['size']):
        if not all(math.isfinite(v) and v > 0 for v in size):
            raise ValueError('Scene dimensions must be positive')
    if not math.isfinite(scene['cargo']['mass']) or scene['cargo']['mass'] <= 0:
        raise ValueError('Cargo mass must be positive')
    if scene.get('reference_site', {}).get('enabled'):
        scene['_asset_root'] = str(Path(path).resolve().parent.parent)
    return scene


def box(link, name, center, size, color, collision=True):
    for tag in ('visual', 'collision') if collision else ('visual',):
        element = ET.SubElement(link, tag, name=name)
        ET.SubElement(element, 'pose').text = ' '.join(map(str, (*center, 0, 0, 0)))
        geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(geometry, 'size').text = ' '.join(map(str, size))
        if tag == 'visual':
            material = ET.SubElement(element, 'material')
            for key in ('ambient', 'diffuse'):
                ET.SubElement(material, key).text = color
        else:
            ode = ET.SubElement(ET.SubElement(ET.SubElement(element, 'surface'), 'friction'), 'ode')
            for key in ('mu', 'mu2'):
                ET.SubElement(ode, key).text = '0.9'


def scene_models(scene):
    """Return independently named SDF models; cargo_link matches native attachment API."""
    if scene.get('reference_site', {}).get('enabled'):
        from .roof_site import scene_models as mesh_scene_models
        return mesh_scene_models(scene)
    result = []
    for spec, dynamic in ((scene['workbench'], False), (scene['cargo'], True)):
        root = ET.Element('sdf', version='1.7')
        model = ET.SubElement(root, 'model', name=spec['model'])
        ET.SubElement(model, 'static').text = 'false' if dynamic else 'true'
        pose = spec['initial_pose'] if dynamic else (*spec['center'], 0)
        ET.SubElement(model, 'pose').text = ' '.join(map(str, (*pose[:3], 0, 0, pose[3])))
        link = ET.SubElement(model, 'link', name='cargo_link' if dynamic else 'platform_link')
        box(link, 'steel' if dynamic else 'solid_platform', (0, 0, 0), spec['size'],
            '0.65 0.71 0.76 1' if dynamic else '0.58 0.61 0.64 1')
        if dynamic:
            mass = spec['mass']
            x, y, z = spec['size']
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass').text = str(mass)
            inertia = ET.SubElement(inertial, 'inertia')
            for key, value in dict(ixx=mass*(y*y+z*z)/12, iyy=mass*(x*x+z*z)/12,
                                   izz=mass*(x*x+y*y)/12, ixy=0, ixz=0, iyz=0).items():
                ET.SubElement(inertia, key).text = str(value)
            plugin = ET.SubElement(model, 'plugin',
                filename='ignition-gazebo-pose-publisher-system',
                name='ignition::gazebo::systems::PosePublisher')
            for key, value in dict(publish_model_pose='true', publish_nested_model_pose='true',
                                   publish_link_pose='false', use_pose_vector_msg='true',
                                   update_frequency='30').items():
                ET.SubElement(plugin, key).text = value
        result.append((spec['model'], ET.tostring(root, encoding='unicode')))
    root = ET.Element('sdf', version='1.7')
    model = ET.SubElement(root, 'model', name='final_site_markings')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='markings')
    cx, cy = scene['site']['center']
    sx, sy = scene['site']['size']
    for index, (x, y, dx, dy) in enumerate(((cx, cy-sy/2, sx, .08),
            (cx, cy+sy/2, sx, .08), (cx-sx/2, cy, .08, sy), (cx+sx/2, cy, .08, sy))):
        box(link, 'boundary_'+str(index), (x, y, .006), (dx, dy, .01), '0.95 0.75 0.12 1', False)
    for name, color in (('startpoint', '0.1 0.75 0.3 1'), ('endpoint', '0.15 0.45 0.95 1'),
                        ('loading_bay', '0.95 0.65 0.12 1'), ('unloading_bay', '0.95 0.65 0.12 1')):
        x, y, z, yaw = scene['locations'][name]
        top = 1.007 if name == 'endpoint' else .012
        # Outlines are visual only: no invisible parking or floor obstacles.
        dx, dy = (1.6, .85) if name in ('startpoint', 'endpoint') else (2., 1.35)
        for index, (ox, oy, a, b) in enumerate(((0, -dy/2, dx, .04), (0, dy/2, dx, .04),
                                               (-dx/2, 0, .04, dy), (dx/2, 0, .04, dy))):
            box(link, name+'_'+str(index), (x+ox, y+oy, top), (a, b, .01), color, False)
    result.append(('final_site_markings', ET.tostring(root, encoding='unicode')))
    return result


def creation_command(scene, name, sdf):
    """Use explicit create pose flags; never depend on the SDF model pose default."""
    root = ET.fromstring(sdf)
    model = root.find('model')
    element = model.find('pose')
    pose = tuple(map(float, element.text.split())) if element is not None else (0.,)*6
    if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
        raise ValueError('Model creation expects a finite xyz/rpy pose')
    if element is not None:
        # The CLI owns the world pose; keep the model origin neutral so that
        # no version of the create utility can add the displacement twice.
        element.text = '0 0 0 0 0 0'
    command = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', scene['world'],
               '-name', name, '-allow_renaming', 'false']
    for flag, value in zip(('-x', '-y', '-z', '-R', '-P', '-Y'), pose):
        command.extend((flag, str(value)))
    return command + ['-string', ET.tostring(root, encoding='unicode')]


class FinalScene:
    """Spawn once in a worker; ROS pose and world-model updates remain asynchronous."""
    def __init__(self, node, config_path):
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from rclpy.clock import Clock, ClockType
        self.node, self.scene = node, load_scene(config_path)
        self.pose_type, self.string_type = PoseStamped, String
        self.results = queue.Queue()
        self.spawned = self.failed = self.ready = False
        self.latest = None
        self.last_written = None
        self.last_seen = 0.
        self.pose_messages = 0
        self.last_frames = []
        self.initial_write_total = self.initial_write_done = 0
        self.world_model_initialized = False
        self.last_pose_wait_log = -math.inf
        self.pending = None
        self.operations = deque()
        self.stopping = threading.Event()
        self.worker = None
        self.started = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = node.create_publisher(String, '/cask/final/scene_status', qos)
        self.pose = node.create_publisher(PoseStamped, '/cask/final/cargo_pose', 10)
        self.clearance = node.create_publisher(String, '/cask/final/cargo_clearance', 10)
        self.subscription = node.create_subscription(TFMessage,
            '/model/'+self.scene['cargo']['model']+'/pose', self.on_pose, 10)
        # Initialization and service timeouts must not slow down with Gazebo's
        # simulation clock. The timer is a watchdog; writes advance on replies.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(.25, self.tick, clock=self.watchdog_clock)
        self.set_status('INITIALIZING')

    def set_status(self, value):
        message = self.string_type()
        message.data = value
        self.status.publish(message)
        if value != getattr(self, '_last_status_text', None):
            self.node.get_logger().info('FINAL_SCENE_STATUS: '+value)
            self._last_status_text = value

    def error(self, message):
        self.failed = True
        self.set_status('ERROR: '+message)
        self.node.get_logger().error('FINAL_SCENE: '+message)

    def spawn(self):
        try:
            for name, sdf in scene_models(self.scene):
                if self.stopping.is_set():
                    return
                result = subprocess.run(creation_command(self.scene, name, sdf),
                    capture_output=True, text=True, timeout=35)
                output = result.stdout+result.stderr
                if result.returncode or 'OK creation of entity' not in output:
                    raise RuntimeError(name+' creation failed: '+output[-1500:])
            self.results.put(None)
        except Exception as error:
            self.results.put(str(error))

    def initial_operations(self):
        cargo = self.scene['cargo']
        for name, pose in self.scene['locations'].items():
            yield name, RDF_TYPE, 'http://www.w3.org/2002/07/owl#NamedIndividual', False
            for key, value in zip(('hasPositionX', 'hasPositionY', 'hasPositionZ', 'hasOrientationYaw'), pose):
                yield name, ABOX+key, str(value), True
            yield name, ABOX+'hasReferenceFrame', 'world', True
        bench = self.scene['workbench']
        yield 'FinalWorkbench_01', RDF_TYPE, 'http://www.w3.org/2002/07/owl#NamedIndividual', False
        for key, value in dict(hasGazeboModel=bench['model'], hasReferenceFrame='world',
                hasPositionX=bench['center'][0], hasPositionY=bench['center'][1], hasPositionZ=bench['center'][2],
                hasSizeX=bench['size'][0], hasSizeY=bench['size'][1], hasSizeZ=bench['size'][2]).items():
            yield 'FinalWorkbench_01', ABOX+key, str(value), True
        name = cargo['individual']
        yield name, RDF_TYPE, 'http://www.w3id.org/hsu-aut/VDI3682#Product', False
        for key, value in dict(hasProductKind=cargo['kind'], hasMass=cargo['mass'],
                               hasGazeboModel=cargo['model'], hasReferenceFrame='world',
                               hasSizeX=cargo['size'][0], hasSizeY=cargo['size'][1],
                               hasSizeZ=cargo['size'][2]).items():
            yield name, ABOX+key, str(value), True

    def tick(self):
        if self.failed or self.stopping.is_set():
            return
        if self.worker is None:
            self.worker = threading.Thread(target=self.spawn, daemon=True)
            self.worker.start()
        if not self.spawned:
            try:
                error = self.results.get_nowait()
            except queue.Empty:
                return
            if error:
                self.error(error)
                return
            self.spawned = True
            self.operations.extend(self.initial_operations())
            self.initial_write_total = len(self.operations)
            self.set_status('WAITING_FOR_MEASURED_CARGO_AND_WORLD_MODEL')
        if self.pending:
            future, sent = self.pending
            if not future.done():
                if time.monotonic()-sent > 15:
                    self.error('World-model update timed out; restart the launch after checking its service')
                return
            self.pending = None
            try:
                response = future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                self.operations.popleft()
                if not self.world_model_initialized:
                    self.initial_write_done += 1
                    if (self.initial_write_done % 10 == 0
                            or self.initial_write_done == self.initial_write_total):
                        self.node.get_logger().info(
                            f'FINAL_SCENE_INITIALIZING: {self.initial_write_done}/{self.initial_write_total} world-model writes')
            except Exception as error:
                self.error('World-model update failed: '+str(error))
                return
        if self.operations:
            self.write_next()
            return
        if not self.world_model_initialized:
            self.world_model_initialized = True
            self.node.get_logger().info('FINAL_SCENE_WORLD_MODEL_READY: initial scene metadata written')
        if self.latest is None or time.monotonic()-self.last_seen > 2:
            self.set_status('WAITING_FOR_MEASURED_CARGO_POSE')
            if time.monotonic()-self.last_pose_wait_log >= 10.:
                self.node.get_logger().warning(
                    'FINAL_SCENE_WAITING_FOR_CARGO_POSE: topic=/model/'+self.scene['cargo']['model']+
                    '/pose; messages='+str(self.pose_messages)+'; last_child_frames='+str(self.last_frames))
                self.last_pose_wait_log = time.monotonic()
            if time.monotonic()-self.started > 120 and not self.ready:
                self.error('No fresh cargo pose: check /model/final_steel/pose and the ROS bridge')
            return
        # READY requires completed initial metadata plus one completed measured
        # pose write. Continuously arriving measurements need not be identical.
        if not self.ready and self.last_written is not None:
            self.ready = True
            self.set_status('READY')
            self.node.get_logger().info(
                'FINAL_SCENE_READY: measured steel cargo, solid 4 x 2.4 x 1 m platform; '
                f'initialization_elapsed={time.monotonic()-self.started:.2f} s wall time')
        if self.ready and getattr(self, '_last_status_text', None) != 'READY':
            self.set_status('READY')
        if self.last_written != self.latest:
            self.last_written = self.latest
            for key, value in zip(('hasPositionX', 'hasPositionY', 'hasPositionZ', 'hasOrientationYaw'), self.latest):
                self.operations.append((self.scene['cargo']['individual'], ABOX+key, str(value), True))
            self.write_next()

    def write_next(self):
        """Keep one ordered request in flight without a timer delay per triple."""
        if self.pending or not self.operations or self.failed or self.stopping.is_set():
            return
        if not self.node.modify_client.service_is_ready():
            if time.monotonic()-self.started > 120:
                self.error('World-model service did not become ready')
            return
        from cask_interfaces.srv import ModifyWorldModel
        name, predicate, value, literal = self.operations[0]
        request = self.node.modify_request(ModifyWorldModel.Request.SET,
            ABOX+name, predicate, value, literal)
        future = self.node.modify_client.call_async(request)
        self.pending = future, time.monotonic()
        future.add_done_callback(self.on_write_done)

    def on_write_done(self, future):
        # Only startup needs an immediate chain. Once ready, keep normal runtime
        # pose persistence paced by the watchdog instead of writing at pose rate.
        if (not self.ready and not self.failed and not self.stopping.is_set()
                and self.pending is not None and self.pending[0] is future):
            self.tick()

    def on_pose(self, message):
        self.pose_messages += 1
        self.last_frames = [t.child_frame_id for t in message.transforms][:6]
        for transform in message.transforms:
            name = transform.child_frame_id.replace('::', '/').rstrip('/').split('/')[-1]
            if name != self.scene['cargo']['model']:
                continue
            p, q = transform.transform.translation, transform.transform.rotation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1) > .05:
                continue
            if self.latest is None:
                self.node.get_logger().info(
                    'FINAL_CARGO_POSE_RECEIVED: model='+self.scene['cargo']['model']+
                    '; child_frame='+transform.child_frame_id+'; source_frame='+transform.header.frame_id)
            pose = self.pose_type()
            pose.header.stamp, pose.header.frame_id = transform.header.stamp, 'world'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
            pose.pose.orientation = q
            self.pose.publish(pose)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.latest = tuple(round(v, 3) for v in (p.x, p.y, p.z, yaw))
            self.last_seen = time.monotonic()
            support = 0.
            for key in ('loading_bay','unloading_bay'):
                if math.dist((p.x,p.y),self.scene['locations'][key][:2]) < 1.:
                    support = self.scene['trailer']['deck_height']
            bench = self.scene['workbench']
            if (abs(p.x-bench['center'][0]) <= bench['size'][0]/2 and
                    abs(p.y-bench['center'][1]) <= bench['size'][1]/2):
                support = bench['center'][2]+bench['size'][2]/2
            message = self.string_type()
            message.data = json.dumps(dict(frame='world',pose=[p.x,p.y,p.z],
                bottom_offset=-self.scene['cargo']['size'][2]/2,support_top_z=support,
                roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))))
            self.clearance.publish(message)

    def close(self):
        self.stopping.set()
        self.timer.cancel()
