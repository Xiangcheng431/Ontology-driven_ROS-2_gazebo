'Gazebo motion implementation extracted unchanged from crane_lifting_server.py.'

from cask_gazebo_motion.delivery_hold import hold_delivery
import json
import math
from cask_gazebo_motion.motion_speed import crane_speed, crane_period
from cask_gazebo_motion.crane_servo import AngularServo, TrolleyServo
import time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import PoseStamped
from cask_gazebo_motion.gazebo_attachment import NativeAttachment
from rclpy.qos import qos_profile_sensor_data
from cask_gazebo_motion.gazebo_motion import MotionSupport, MotionError, angle
from ros_interfaces.action import ExecuteSkill
from cask_handoff.channel import Channel
from cask_handoff.contact import SupportContacts
from cask_gazebo_motion.crane_handoff import finish_handoff
from cask_gazebo_motion.obstacle_manager_node import SharedObstacleState

class CraneGazeboMotion(MotionSupport):
    def _gazebo_craneliftingserver_setup_0(self):
        self.handoff_channel = Channel(self)
        self.handoff_contacts = SupportContacts(self, 'task002_soil', '/cask/soil/contacts')
        self.obstacle_state = SharedObstacleState(self)
        self.declare_parameter('physical_cargo_mode', False)
        self.declare_parameter('angular_velocity_control', False)
        self.angular_velocity_control = bool(self.get_parameter('angular_velocity_control').value)
        self.angular_targets = {}
        self.angular_velocity_publishers = {}
        self.angular_servos = {}
        self.declare_parameter('trolley_velocity_control', False)
        self.trolley_velocity_control = bool(self.get_parameter('trolley_velocity_control').value)
        self.trolley_target = None
        self.trolley_servo = TrolleyServo()
        self.trolley_velocity_command = 0.0
        self.declare_parameter('gazebo_world', 'empty')
        self.declare_parameter('crane_world_x', 2.8)
        self.declare_parameter('crane_world_y', 0.0)
        self.declare_parameter('crane_world_yaw', 0.0)

    def _gazebo_craneliftingserver_setup_1(self):
        self.physical_cargo_mode = bool(self.get_parameter('physical_cargo_mode').value)
        self.declare_parameter('pickup_clearance_margin', 0.15)
        self.declare_parameter('cargo_clearance_topic', '/cask/task001/cargo_clearance')
        self.clearance_description = None
        self.clearance_received = None
        self.cargo_pose = None
        self.cargo_feedback_time = None
        self.native_attachment_state = None
        self.native_state_revision = 0
        self.native_attachment = NativeAttachment(self.get_parameter('gazebo_world').value)
        self.active_cargo_task = 'Task_001'
        self.cargo_contexts = {}
        self.native_attachments = {'Task_001': self.native_attachment, 'Task_002': NativeAttachment(self.get_parameter('gazebo_world').value, child_model='task002_soil', topic='/cask/crane/soil')}
        self.native_attachments['Task_005'] = NativeAttachment(self.get_parameter('gazebo_world').value, child_model='task005_steel', topic='/cask/crane/task005')
        self.hook_yaw_position = 0.0
        if self.physical_cargo_mode:
            self.clearance_subscription = self.create_subscription(String, self.get_parameter('cargo_clearance_topic').value, lambda msg: self.cargo_message('Task_001', 'clearance', msg), 10)
            self.cargo_subscription = self.create_subscription(PoseStamped, '/cask/task001/cargo_pose', lambda msg: self.cargo_message('Task_001', 'pose', msg), 10)
            self.native_state_subscription = self.create_subscription(String, '/cask/crane/cargo/state', lambda msg: self.cargo_message('Task_001', 'native', msg), 10)
            self.soil_cargo_subscription = self.create_subscription(PoseStamped, '/cask/task002/cargo_pose', lambda msg: self.cargo_message('Task_002', 'pose', msg), 10)
            self.soil_clearance_subscription = self.create_subscription(String, '/cask/task002/cargo_clearance', lambda msg: self.cargo_message('Task_002', 'clearance', msg), 10)
            self.soil_native_subscription = self.create_subscription(String, '/cask/crane/soil/state', lambda msg: self.cargo_message('Task_002', 'native', msg), 10)
        if self.physical_cargo_mode:
            self.task005_pose_sub = self.create_subscription(PoseStamped, '/cask/task005/cargo_pose', lambda msg: self.cargo_message('Task_005', 'pose', msg), 10)
            self.task005_clearance_sub = self.create_subscription(String, '/cask/task005/cargo_clearance', lambda msg: self.cargo_message('Task_005', 'clearance', msg), 10)
            self.task005_native_sub = self.create_subscription(String, '/cask/crane/task005/state', lambda msg: self.cargo_message('Task_005', 'native', msg), 10)
        self.declare_parameter('hook_zero_world_z', 10.0)
        self.declare_parameter('hook_joint_lower', 0.0)
        self.declare_parameter('hook_joint_upper', 9.4)
        self.declare_parameter('tower_joint_lower', -math.pi)
        self.declare_parameter('tower_joint_upper', math.pi)
        self.declare_parameter('trolley_joint_lower', 0.0)
        self.declare_parameter('trolley_joint_upper', 5.2)
        self.declare_parameter('initial_tower_yaw', 1.5708)
        self.declare_parameter('initial_trolley_position', 1.8)
        self.declare_parameter('initial_hook_position', 9.27)
        self.hook_zero_world_z = float(self.get_parameter('hook_zero_world_z').value)
        self.hook_joint_lower = float(self.get_parameter('hook_joint_lower').value)
        self.hook_joint_upper = float(self.get_parameter('hook_joint_upper').value)
        self.tower_joint_lower = float(self.get_parameter('tower_joint_lower').value)
        self.tower_joint_upper = float(self.get_parameter('tower_joint_upper').value)
        self.trolley_joint_lower = float(self.get_parameter('trolley_joint_lower').value)
        self.trolley_joint_upper = float(self.get_parameter('trolley_joint_upper').value)
        self.joint_names = ['crane_tower_joint', 'crane_trolley_joint', 'crane_hook_joint']
        if self.physical_cargo_mode:
            self.joint_names.append('crane_hook_yaw_joint')
        self.tower_position = float(self.get_parameter('initial_tower_yaw').value)
        self.trolley_position = float(self.get_parameter('initial_trolley_position').value)
        self.hook_position = float(self.get_parameter('initial_hook_position').value)
        self.setup_motion()
        self.command_publishers = {'tower_position': self.create_publisher(Float64, '/crane/tower_cmd', 10), 'trolley_position': self.create_publisher(Float64, '/crane/trolley_cmd', 10), 'hook_position': self.create_publisher(Float64, '/crane/hook_cmd', 10)}
        if self.physical_cargo_mode:
            self.command_publishers['hook_yaw_position'] = self.create_publisher(Float64, '/crane/hook_yaw_cmd', 10)
        if self.angular_velocity_control:
            self.angular_position_subscriptions = []
            for (key, position_topic, velocity_topic) in (('tower_position', '/crane/tower_cmd', '/crane/tower_velocity_cmd'), ('hook_yaw_position', '/crane/hook_yaw_cmd', '/crane/hook_yaw_velocity_cmd')):
                self.angular_velocity_publishers[key] = self.create_publisher(Float64, velocity_topic, 10)
                self.angular_position_subscriptions.append(self.create_subscription(Float64, position_topic, lambda msg, k=key: self.set_angular_target(k, msg.data), 10))
        if self.trolley_velocity_control:
            self.trolley_velocity_publisher = self.create_publisher(Float64, '/crane/trolley_velocity_cmd', 10)
            self.trolley_position_subscription = self.create_subscription(Float64, '/crane/trolley_cmd', lambda msg: self.set_trolley_target(msg.data), 10)
        if self.angular_velocity_control or self.trolley_velocity_control:
            self.angular_timer = self.create_timer(crane_period(0.02), self.control_angular_joints)
        self.joint_subscription = self.create_subscription(JointState, '/crane/joint_states', self.gazebo_joints, qos_profile_sensor_data)
        attachment_qos = QoSProfile(depth=1)
        attachment_qos.reliability = ReliabilityPolicy.RELIABLE
        attachment_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.attachment_publisher = self.create_publisher(Bool, '/crane_01/object_attached', attachment_qos)
        self.object_attached = False
        self.publish_attachment_state()
        self.installation_subscription = self.create_subscription(Bool, '/cask/components/roof_component_03/installed', self.object_installed_callback, attachment_qos)

    def _gazebo_craneliftingserver_setup_2(self):
        minimum_world_z = self.hook_zero_world_z - self.hook_joint_upper
        maximum_world_z = self.hook_zero_world_z - self.hook_joint_lower
        self.get_logger().info('Crane lifting Action Server started on /crane_01/lifting')
        self.get_logger().info('Gazebo mode: consuming /crane/joint_states; publishing joint commands')
        self.get_logger().info(f'Initial Crane joints from scenario: tower={self.tower_position:.4f} rad, trolley={self.trolley_position:.3f} m, hook={self.hook_position:.3f} m')
        self.get_logger().info(f'Reachable hook height: {minimum_world_z:.2f} to {maximum_world_z:.2f} m in world')

    def _gazebo_craneliftingserver_visual_lifting(self, goal_handle, parameters):
        pickup_yaw = self.required_parameter(parameters, 'pickup_tower_yaw')
        pickup_trolley = self.required_parameter(parameters, 'pickup_trolley_position')
        delivery_yaw = self.required_parameter(parameters, 'delivery_tower_yaw')
        delivery_trolley = self.required_parameter(parameters, 'delivery_trolley_position')
        target_height = self.required_parameter(parameters, 'target_height')
        pickup_height = parameters.get('recoverypickupheight', parameters.get('pickupheight', 0.8))
        delivery_height = parameters.get('deliveryhookheight', target_height)
        self.finite(pickup_yaw, pickup_trolley, delivery_yaw, delivery_trolley, target_height, pickup_height, delivery_height)
        for (label, value) in [('pickup_tower_yaw', pickup_yaw), ('delivery_tower_yaw', delivery_yaw)]:
            self.validate_range(label, value, self.tower_joint_lower, self.tower_joint_upper)
        for (label, value) in [('pickup_trolley_position', pickup_trolley), ('delivery_trolley_position', delivery_trolley)]:
            self.validate_range(label, value, self.trolley_joint_lower, self.trolley_joint_upper)
        pickup_hook = self.world_height_to_joint_position(pickup_height)
        lift_hook = self.world_height_to_joint_position(target_height)
        delivery_hook = self.world_height_to_joint_position(delivery_height)
        safe_height = min(self.hook_zero_world_z - self.hook_joint_lower, max(target_height, pickup_height, delivery_height) + 0.8)
        safe_hook = self.world_height_to_joint_position(safe_height)
        self.object_attached = False
        self.publish_attachment_state()
        self.move_phase(goal_handle, {'hook_position': safe_hook}, 0, 5, 'RAISING_TO_CLEARANCE')
        self.move_phase(goal_handle, {'tower_position': pickup_yaw, 'trolley_position': pickup_trolley}, 5, 15, 'MOVING_TO_PICKUP')
        self.move_phase(goal_handle, {'hook_position': pickup_hook}, 15, 20, 'LOWERING_TO_PICKUP')
        self.object_attached = True
        self.publish_attachment_state()
        self.phase_feedback(goal_handle, 20, 'ATTACHING | object_attached=true')
        self.move_phase(goal_handle, {'hook_position': lift_hook}, 20, 60, 'LIFTING')
        self.move_phase(goal_handle, {'tower_position': delivery_yaw, 'trolley_position': delivery_trolley}, 60, 90, 'ROTATING_TO_DELIVERY')
        self.move_phase(goal_handle, {'hook_position': delivery_hook}, 90, 99, 'LOWERING')
        self.phase_feedback(goal_handle, 100, 'HOLDING_AT_DELIVERY | object_attached=true')

    def _gazebo_craneliftingserver_close_physics(self):
        self.stop_angular_joints()
        for attachment in self.native_attachments.values():
            attachment.close()

    def gazebo_joints(self, message):
        state = dict(zip(message.name, message.position))
        if not all(name in state for name in self.joint_names):
            return
        if not all(math.isfinite(state[name]) for name in self.joint_names):
            return
        self.tower_position, self.trolley_position, self.hook_position = [
            state[name] for name in self.joint_names[:3]]
        if self.physical_cargo_mode:
            self.hook_yaw_position = state[self.joint_names[3]]
        self.last_joint_feedback = time.monotonic()

    def command_positions(self, targets):
        for name, value in targets.items():
            if self.angular_velocity_control and name in self.angular_velocity_publishers:
                self.set_angular_target(name,value)
            elif self.trolley_velocity_control and name == 'trolley_position':
                self.set_trolley_target(value)
            else:
                self.command_publishers[name].publish(Float64(data=float(value)))

    def set_trolley_target(self, value):
        if math.isfinite(value):
            self.trolley_target = float(value)

    def set_angular_target(self, name, value):
        if math.isfinite(value):
            self.angular_targets[name] = float(value)

    @staticmethod
    def angular_speed(current, target):
        # Proportional velocity feedback, not a position error divided by the
        # Gazebo physics timestep. Slow continuously as the angle converges.
        if not math.isfinite(current) or not math.isfinite(target):
            return 0.0
        return max(-crane_speed(.132),min(crane_speed(.132),crane_speed(.88)*(target-current)))

    def control_angular_joints(self):
        fresh = (self.last_joint_feedback is not None
                 and time.monotonic()-self.last_joint_feedback <= self.feedback_timeout)
        sim_seconds = self.get_clock().now().nanoseconds/1e9
        for name,publisher in self.angular_velocity_publishers.items():
            target = self.angular_targets.get(name)
            servo = self.angular_servos.setdefault(name, AngularServo())
            if fresh and target is not None:
                current = getattr(self,name)
                speed = servo.step(current, target, self.angular_speed(current,target), sim_seconds)
            else:
                servo.stop()
                speed = 0.0
            publisher.publish(Float64(data=speed))
        if self.trolley_velocity_control:
            if fresh:
                if self.trolley_target is None:
                    self.trolley_target = self.trolley_position
                speed = self.trolley_servo.command(self.trolley_position,self.trolley_target,sim_seconds)
            else:
                self.trolley_servo.stop()
                speed = 0.0
            self.trolley_velocity_command = speed
            self.trolley_velocity_publisher.publish(Float64(data=speed))

    def stop_angular_joints(self):
        if self.trolley_velocity_control:
            self.trolley_servo.stop()
            self.trolley_velocity_command = 0.0
            self.trolley_velocity_publisher.publish(Float64(data=0.0))
        for servo in self.angular_servos.values():
            servo.stop()
        for publisher in self.angular_velocity_publishers.values():
            publisher.publish(Float64(data=0.0))

    def hold_position(self):
        self.stop_angular_joints()
        if self.last_joint_feedback is not None:
            self.command_positions({name: getattr(self, name)
                for name in self.command_publishers})

    def move_phase(self, goal, targets, start, end, status):
        self.wait_positions(goal, targets,
            lambda: self.command_positions(targets),
            lambda fraction: self.phase_feedback(goal, start+(end-start)*fraction, status))

    def phase_feedback(self, goal, progress, status):
        message = ExecuteSkill.Feedback()
        message.progress = float(progress)
        message.status = (f'{status} | tower={self.tower_position:.4f} | '
            f'trolley={self.trolley_position:.3f} | '
            f'hook_world_z={self.hook_zero_world_z-self.hook_position:.3f} m')
        if self.angular_velocity_control:
            errors = ', '.join(f'{name}_error={target-getattr(self,name):.5f}'
                               for name,target in self.angular_targets.items())
            message.status += ' | '+errors
        if self.trolley_velocity_control and self.trolley_target is not None:
            message.status += (f' | trolley_error={self.trolley_target-self.trolley_position:.5f}'
                               f' | trolley_v_cmd={self.trolley_velocity_command:.4f}')
        goal.publish_feedback(message)

    def publish_attachment_state(self):
        message = Bool()
        message.data = self.object_attached
        self.attachment_publisher.publish(message)

    def object_installed_callback(self, message):
        if not bool(message.data):
            return

        if self.object_attached:
            self.object_attached = False
            self.publish_attachment_state()
            self.get_logger().info(
                'Installation confirmed: object_attached=False'
            )

    @staticmethod
    def validate_range(name, value, lower, upper):
        if value < lower - 1e-6 or value > upper + 1e-6:
            raise ValueError(
                f'{name}={value:.12g} is outside range '
                f'[{lower:.12g}, {upper:.12g}]'
            )

    def world_height_to_joint_position(self, target_height):
        # For this prismatic joint: hook_world_z = zero_z - joint_position.
        target_joint_position = self.hook_zero_world_z - target_height

        tolerance = 1e-6
        if target_joint_position < self.hook_joint_lower - tolerance:
            raise ValueError(
                f'target_height={target_height:.3f} m is above the crane '
                f'maximum {self.hook_zero_world_z - self.hook_joint_lower:.3f} m'
            )

        if target_joint_position > self.hook_joint_upper + tolerance:
            raise ValueError(
                f'target_height={target_height:.3f} m is below the crane '
                f'minimum {self.hook_zero_world_z - self.hook_joint_upper:.3f} m'
            )

        return min(
            max(target_joint_position, self.hook_joint_lower),
            self.hook_joint_upper,
        )

    def cargo_message(self, task, kind, message):
        self.cargo_contexts.setdefault(task,{})[kind]=(message,time.monotonic())
        if task==self.active_cargo_task:
            self.apply_cargo_message(kind,message)

    def apply_cargo_message(self,kind,message):
        {'pose':self.measured_cargo_pose,'clearance':self.measured_clearance_description,
         'native':self.measured_attachment_state}[kind](message)

    def select_cargo(self,task):
        if task not in self.native_attachments:
            raise MotionError('PHYSICAL_CARGO_UNSUPPORTED: '+task)
        if self.object_attached:
            raise MotionError('LOAD_ALREADY_ATTACHED')
        self.active_cargo_task=task
        self.native_attachment=self.native_attachments[task]
        self.cargo_pose=self.cargo_feedback_time=None
        self.clearance_description=self.clearance_received=None
        self.native_attachment_state=None
        self.object_attached=False
        for kind,(msg,stamp) in self.cargo_contexts.get(task,{}).items():
            if time.monotonic()-stamp<=self.feedback_timeout:
                self.apply_cargo_message(kind,msg)

    def measured_cargo_pose(self, message):
        p, q = message.pose.position, message.pose.orientation
        if message.header.frame_id not in ('world', 'empty'):
            return
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values):
            return
        if abs(sum(v*v for v in values[3:])-1) > .05:
            return
        self.cargo_pose = (p.x,p.y,p.z,
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.cargo_feedback_time = time.monotonic()

    def measured_attachment_state(self, message):
        if message.data in ('attached', 'detached'):
            self.native_attachment_state = message.data
            self.native_state_revision += 1
            self.object_attached = message.data == 'attached'
            self.publish_attachment_state()

    def measured_cargo(self, goal):
        self.check_feedback(goal)
        if (self.cargo_pose is None or self.cargo_feedback_time is None
                or time.monotonic()-self.cargo_feedback_time > self.feedback_timeout):
            raise MotionError('CARGO_FEEDBACK_LOST: no fresh Component Manager cargo pose')
        return self.cargo_pose

    def hook_pose(self):
        yaw = float(self.get_parameter('crane_world_yaw').value)+self.tower_position
        radius = .40+self.trolley_position
        return (float(self.get_parameter('crane_world_x').value)+radius*math.cos(yaw),
                float(self.get_parameter('crane_world_y').value)+radius*math.sin(yaw),
                self.hook_zero_world_z-self.hook_position,
                yaw+self.hook_yaw_position)

    def crane_targets(self, x,y,z,cargo_yaw):
        dx = x-float(self.get_parameter('crane_world_x').value)
        dy = y-float(self.get_parameter('crane_world_y').value)
        base_yaw = float(self.get_parameter('crane_world_yaw').value)
        tower = angle(math.atan2(dy,dx)-base_yaw)
        trolley = math.hypot(dx,dy)-.40
        self.validate_range('trolley',trolley,self.trolley_joint_lower,self.trolley_joint_upper)
        self.validate_range('tower',tower,self.tower_joint_lower,self.tower_joint_upper)
        return {'tower_position':tower,'trolley_position':trolley,
                'hook_position':self.world_height_to_joint_position(z),
                'hook_yaw_position':angle(cargo_yaw-base_yaw-tower)}

    def physical_move(self, goal, targets, start, end, status):
        self.finite(*targets.values())
        began = time.monotonic()
        stable = None
        initial = {k:abs(getattr(self,k)-v) for k,v in targets.items()}
        while True:
            self.measured_cargo(goal)
            if time.monotonic()-began > self.motion_timeout:
                raise MotionError('MOTION_TIMEOUT: physical crane joint target not reached')
            self.command_positions(targets)
            errors = {k:abs(getattr(self,k)-v) for k,v in targets.items()}
            fraction = max(0.,1-max(errors[k]/max(initial[k],.01) for k in targets))
            self.phase_feedback(goal,start+(end-start)*fraction,status)
            # At a 5 m radius the old .025 rad tolerance permits .125 m XY error.
            if all(e <= (.004 if 'yaw' in k or k == 'tower_position' else .008)
                   for k,e in errors.items()):
                if stable is None:
                    stable = time.monotonic()
                elif time.monotonic()-stable > .5:
                    return
            else:
                stable = None
            if self.object_attached:
                cargo, hook = self.measured_cargo(goal), self.hook_pose()
                if (math.dist(cargo[:2],hook[:2]) > .20
                        or abs((hook[2]-cargo[2])-self.attached_gap) > .15):
                    raise MotionError('CARGO_CONNECTION_LOST: measured load no longer follows physical joint')
            time.sleep(crane_period(.1))

    def wait_native_state(self, goal, expected, previous_revision, tick=None):
        until = time.monotonic()+8
        while time.monotonic() < until:
            self.measured_cargo(goal)
            if tick is not None:
                tick()
            if (self.native_state_revision > previous_revision
                    and self.native_attachment_state == expected):
                return
            time.sleep(.1)
        raise MotionError('ATTACHMENT_NOT_CONFIRMED: expected native '+expected+
                          '; check plugin output and '+self.native_attachment.TOPIC+'/state bridge')

    def measured_clearance_description(self, message):
        try:
            data = json.loads(message.data)
            values = [*data['pose'], data['bottom_offset'], data['support_top_z'],
                      data['roll'], data['pitch']]
            if (data['frame'] != 'world' or len(data['pose']) != 3
                    or not all(math.isfinite(v) for v in values)):
                raise ValueError('Invalid clearance description')
            self.clearance_description = data
            self.clearance_received = time.monotonic()
        except (ValueError, TypeError, KeyError):
            self.clearance_description = None
            self.clearance_received = None

    def pickup_clearance_plan(self, goal, pickup, target_height, delivery_z):
        """Snapshot stationary support geometry before attaching or lifting.

        This calculation does not depend on a task ID or a carrier name.
        A scene adapter must supply a valid support envelope for its cargo.
        """
        self.measured_cargo(goal)
        data = self.clearance_description
        if (data is None or self.clearance_received is None
                or time.monotonic()-self.clearance_received > self.feedback_timeout):
            raise MotionError('CLEARANCE_GEOMETRY_MISSING: no fresh Component Manager description')
        if math.dist(data['pose'], pickup[:3]) > .05:
            raise MotionError('CLEARANCE_POSE_MISMATCH: geometry is not at the pickup pose')
        if max(abs(data['roll']), abs(data['pitch'])) > .01:
            raise MotionError('CLEARANCE_GEOMETRY_UNSUPPORTED: tilted cargo needs a 3D envelope')
        margin = float(self.get_parameter('pickup_clearance_margin').value)
        self.finite(target_height, delivery_z, margin)
        if margin < .05:
            raise MotionError('INVALID_PARAMETER: clearance margin must be at least 0.05 m')
        support_top = data['support_top_z']
        bottom = data['bottom_offset']
        # .03 m command allowance keeps measured clearance above the margin
        # despite joint tolerance; target_height remains the ontology goal.
        transport_z = max(target_height, pickup[2]+.15,
                          support_top-bottom+margin+.03, delivery_z+.30)
        return dict(support_top=support_top, bottom=bottom, margin=margin,
                    transport_z=transport_z, pickup=tuple(pickup))

    def ensure_pickup_clearance(self, goal, plan):
        """Raise vertically, then verify measured separation before any transport."""
        target = self.world_height_to_joint_position(plan['transport_z']+self.attached_gap)
        self.phase_feedback(goal,20,
            f"CLEARANCE_CHECK | support_top_z={plan['support_top']:.3f} | "
            f"transport_cargo_z={plan['transport_z']:.3f}")
        self.physical_move(goal, {'hook_position':target}, 20,55, 'LIFTING_FOR_CLEARANCE')
        stable = None
        until = time.monotonic()+5.
        while time.monotonic() < until:
            cargo = self.measured_cargo(goal)
            data = self.clearance_description
            if (data is None or self.clearance_received is None
                    or time.monotonic()-self.clearance_received > self.feedback_timeout):
                raise MotionError('CLEARANCE_GEOMETRY_MISSING: feedback lost during verification')
            if max(abs(data['roll']),abs(data['pitch'])) > .01:
                raise MotionError('CLEARANCE_GEOMETRY_UNSUPPORTED: cargo tilted during lift')
            if (math.dist(cargo[:2],plan['pickup'][:2]) > .06
                    or abs(angle(cargo[3]-plan['pickup'][3])) > .03):
                raise MotionError('CLEARANCE_DRIFT: cargo moved sideways before separation')
            clearance = cargo[2]+plan['bottom']-plan['support_top']
            if clearance >= plan['margin'] and self.object_attached:
                if stable is None:
                    stable = time.monotonic()
                elif time.monotonic()-stable >= .5:
                    self.phase_feedback(goal,55,
                        f'CLEARANCE_VERIFIED | measured_clearance={clearance:.3f} | '
                        f'measured_cargo_z={cargo[2]:.3f}')
                    return
            else:
                stable = None
            time.sleep(.1)
        raise MotionError('CLEARANCE_NOT_VERIFIED: hold load; rotation remains blocked')

    def execute_physical_task001(self, goal):
        p = self.parameter_map(goal.request)

        def required(name):
            return self.required_parameter(p, name)
        expected = tuple((required('cargo_pickup_' + axis) for axis in ('x', 'y', 'z')))
        delivery = tuple((required('cargo_delivery_' + axis) for axis in ('x', 'y', 'z', 'yaw')))
        gap = required('rigging_gap')
        self.finite(*expected, *delivery, gap)
        if not 0.45 <= gap <= 1.0:
            raise MotionError('INVALID_PARAMETER: rigging gap must clear the bundle')
        cargo = self.measured_cargo(goal)
        if math.dist(cargo[:3], expected) > 0.35:
            raise MotionError('PICKUP_POSE_MISMATCH: cargo differs from ontology recovery/input pose')
        if self.object_attached:
            raise MotionError('LOAD_ALREADY_ATTACHED: resolve previous execution before another task')
        pickup = cargo
        clearance_plan = self.pickup_clearance_plan(goal, pickup, required('target_height'), delivery[2])
        safe_z = clearance_plan['transport_z'] + gap
        pickup_targets = self.crane_targets(*pickup[:2], pickup[2] + gap, pickup[3])
        deliver_targets = self.crane_targets(*delivery[:2], delivery[2] + gap + 0.015, delivery[3])
        safe_joint = self.world_height_to_joint_position(safe_z)
        self.physical_move(goal, {'hook_position': safe_joint}, 0, 5, 'RAISING_TO_CLEARANCE')
        self.physical_move(goal, {k: v for (k, v) in pickup_targets.items() if k != 'hook_position'}, 5, 15, 'MOVING_TO_PICKUP')
        self.physical_move(goal, {'hook_position': pickup_targets['hook_position']}, 15, 20, 'LOWERING_TO_PICKUP')
        (actual, hook) = (self.measured_cargo(goal), self.hook_pose())
        if math.dist(actual[:2], hook[:2]) > 0.06 or abs(hook[2] - actual[2] - gap) > 0.05 or abs(angle(hook[3] - actual[3])) > 0.03:
            raise MotionError('HOOK_ALIGNMENT_FAILED: physical hook is not above measured cargo')
        self.hold_position()
        self.physical_move(goal, {'tower_position': self.tower_position, 'trolley_position': self.trolley_position, 'hook_yaw_position': self.hook_yaw_position}, 19, 20, 'SETTLING_BEFORE_ATTACHMENT')
        (actual, hook) = (self.measured_cargo(goal), self.hook_pose())
        self.attached_gap = hook[2] - actual[2]
        previous = self.native_state_revision
        try:
            self.native_attachment.attach()
        except (RuntimeError, OSError) as error:
            raise MotionError('ATTACHMENT_FAILED: ' + str(error)) from error
        try:
            self.wait_native_state(goal, 'attached', previous)
        except MotionError as error:
            if not str(error).startswith('ATTACHMENT_NOT_CONFIRMED'):
                raise
            self.phase_feedback(goal, 20, 'ATTACHMENT_STATE_HANDSHAKE')
            previous = self.native_state_revision
            self.native_attachment.detach()
            self.wait_native_state(goal, 'detached', previous)
            previous = self.native_state_revision
            self.native_attachment.attach()
            self.wait_native_state(goal, 'attached', previous)
        self.object_attached = True
        self.publish_attachment_state()
        self.phase_feedback(goal, 20, 'ATTACHING | native_joint=attached')
        self.ensure_pickup_clearance(goal, clearance_plan)
        self.physical_move(goal, {k: v for (k, v) in deliver_targets.items() if k != 'hook_position'}, 55, 85, 'ROTATING_TO_DELIVERY')
        if p.get('handoffmode') == 1.0:
            return finish_handoff(self, goal, p, delivery)
        duration = float(p.get('deliveryholdseconds', 0.0))
        if duration > 0:
            hold_targets = dict(deliver_targets)
            hold_targets['hook_position'] = self.world_height_to_joint_position(required('target_height') + self.attached_gap)
            self.physical_move(goal, hold_targets, 85, 87, 'MOVING_TO_HOLD_HEIGHT')
            try:
                hold_delivery(lambda : self.measured_cargo(goal), lambda : self.command_positions(hold_targets), lambda : self.get_clock().now().nanoseconds / 1000000000.0, lambda status: self.phase_feedback(goal, 88, status), (delivery[0], delivery[1], required('target_height'), delivery[3]), duration)
            except RuntimeError as error:
                raise MotionError(str(error)) from error
        self.physical_move(goal, {'hook_position': deliver_targets['hook_position']}, 85, 95, 'LOWERING')
        measured = self.measured_cargo(goal)
        if math.dist(measured[:2], delivery[:2]) > 0.15 or abs(measured[2] - delivery[2]) > 0.06 or abs(angle(measured[3] - delivery[3])) > 0.06:
            raise MotionError('DELIVERY_ALIGNMENT_FAILED: hold load; release is unsafe')
        previous = self.native_state_revision
        # CASK_TASK005_RELEASE_INLINE_V2
        if goal.request.task_id == 'Task_005':
            self._release_task005(goal, delivery, previous)
        else:
            self.native_attachment.detach()
            self.wait_native_state(goal, 'detached', previous)
        self.object_attached = False
        self.publish_attachment_state()
        self.phase_feedback(goal, 95, 'RELEASING | native_joint=detached')
        self.physical_move(goal, {'hook_position': self.world_height_to_joint_position(delivery[2] + gap + 0.5)}, 95, 99, 'WITHDRAWING_HOOK')
        if (goal.request.task_id == 'Task_005') or getattr(self, 'execution_context', {}).get('clear_processing_area', False):
            self.physical_move(goal, {'hook_position': self.world_height_to_joint_position(max(safe_z, delivery[2] + gap + 0.5))}, 98, 99, 'CLEARING_WORK_AREA')
            self.physical_move(goal, {k: v for (k, v) in pickup_targets.items() if k != 'hook_position'}, 98, 99, 'WITHDRAWING_FROM_PROCESSING_SITE')
        (until, stable) = (time.monotonic() + 10, None)
        while time.monotonic() < until:
            measured = self.measured_cargo(goal)
            if math.dist(measured[:2], delivery[:2]) <= 0.18 and abs(measured[2] - delivery[2]) <= 0.06 and (abs(angle(measured[3] - delivery[3])) <= 0.08):
                if stable is None:
                    stable = time.monotonic()
                elif time.monotonic() - stable > 0.7:
                    self.phase_feedback(goal, 100, 'SUCCESS | physical_cargo_verified=true')
                    return self._complete_crane_delivery(goal)
            else:
                stable = None
            time.sleep(0.1)
        raise MotionError('DELIVERY_NOT_VERIFIED: released cargo is outside ontology target')

    def _release_task005(self, goal, delivery, previous_ros_revision):
        attachment = self.native_attachment
        previous_native_revision, _ = attachment.state_snapshot()
        last_feedback = [-math.inf]

        def confirmed():
            if (self.native_state_revision > previous_ros_revision
                    and self.native_attachment_state == 'detached'):
                return True
            revision, state = attachment.state_snapshot()
            return revision > previous_native_revision and state == 'detached'

        def tick():
            cargo = self.measured_cargo(goal)
            if (math.dist(cargo[:2], delivery[:2]) > .15
                    or abs(cargo[2] - delivery[2]) > .06
                    or abs(angle(cargo[3] - delivery[3])) > .06):
                raise MotionError('DELIVERY_ALIGNMENT_FAILED: hold load; release is unsafe')
            if time.monotonic() - last_feedback[0] >= .5:
                last_feedback[0] = time.monotonic()
                self.phase_feedback(goal, 95,
                    'RELEASING_TASK005 | awaiting_native_detach=true')

        try:
            attachment.detach(tick=tick, confirmed=confirmed)
        except (RuntimeError, OSError) as error:
            raise MotionError('TASK005_RELEASE_FAILED: ' + str(error)) from error
        if not confirmed():
            raise MotionError('ATTACHMENT_NOT_CONFIRMED: Task005 native detach not observed')

# CASK_ALL_ROBOT_CONTEXT_V1
