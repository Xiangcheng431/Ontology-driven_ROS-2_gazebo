'Gazebo motion implementation extracted unchanged from jekko_skill_server.py.'
from cask_gazebo_motion.gazebo_joint_commands import create_joint_command_publisher

from cask_gazebo_motion.task005_geometry import rotate
import math
import time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from cask_gazebo_motion.task005_geometry import angle
from cask_gazebo_motion.obstacle_manager_node import SharedObstacleState
from cask_gazebo_motion.ground_navigation import PoseController
from cask_gazebo_motion.task005_route_planner import Task005RoutePlanner
from std_msgs.msg import String
from cask_gazebo_motion.jekko_geometry import jekko_targets
from cask_gazebo_motion.gazebo_attachment import NativeAttachment
from cask_gazebo_motion.jekko_visuals import JekkoCableVisual
from cask_gazebo_motion.jekko_navigation import JekkoPoseController

class JekkoGazeboMotionBase:
    def _gazebo_jekkoskillserverbase_setup_0(self, robot):
        self.robot = robot
        self.motion_scale = 2.0

    def _gazebo_jekkoskillserverbase_setup_1(self):
        self.declare_parameter('feedback_timeout', 2.0)
        self.declare_parameter('motion_timeout', 180.0)
        self.declare_parameter('crane_world_x', 2.8)
        self.declare_parameter('crane_world_y', 0.0)
        self.declare_parameter('crane_world_yaw', 0.0)
        # Fixed crane footprint follows scene placement; navigation is unchanged.
        self.crane_base_obstacle = (
            float(self.get_parameter('crane_world_x').value),
            float(self.get_parameter('crane_world_y').value),
            float(self.get_parameter('crane_world_yaw').value),
            -1.3, 1.3, -1.1, 1.1)


    def _gazebo_jekkoskillserverbase_setup_2(self, commands, robot):
        self.pose = None
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.pose_time = 0.0
        self.joints = {}
        self.joints_time = 0.0
        self.entities = {}
        self.cargo = None
        self.cargo_time = 0.0
        self.loaded = False
        self.tool_mode = 0
        self.native_state = None
        self.native_revision = 0
        self.velocity_joints = set()
        self.targets = {}
        self.command_time = 0.0
        self.drive_pub = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        self.joint_command_publishers = {name: create_joint_command_publisher(self, Float64, topic, 10) for (name, topic) in commands.items()}
        self.create_subscription(JointState, f'/{robot}/joint_states', self.on_joints, qos_profile_sensor_data)
        pose_robots = ('jekko', 'innok223')
        self.pose_subscriptions = [self.create_subscription(TFMessage, f'/model/{name}/pose', lambda msg, n=name: self.on_pose(msg, n), 10) for name in pose_robots]
        self.create_subscription(PoseStamped, '/cask/task005/cargo_pose', lambda msg: self.on_cargo(msg) if getattr(self, 'generated_selected_task', 'Task_005') == 'Task_005' else None, 10)  # CASK_GENERATED_IOC_V1
        self.obstacles = SharedObstacleState(self)

    def _gazebo_jekkoskillserverbase_setup_3(self):
        self.watchdog = self.create_timer(0.05 / self.motion_scale, self.hold, callback_group=self.group)

    def _gazebo_jekkoskillserverbase_check_feedback(self, goal, cargo):
        now = time.monotonic()
        limit = float(self.get_parameter('feedback_timeout').value)
        if self.pose is None or now - self.pose_time > limit or now - self.joints_time > limit:
            raise RuntimeError('FEEDBACK_LOST: Gazebo pose/joints are not fresh')
        if cargo and (self.cargo is None or now - self.cargo_time > limit):
            raise RuntimeError('CARGO_FEEDBACK_LOST')

    def _gazebo_jekkoskillserverbase_wait_scene(self, goal):
        until = time.monotonic() + 10.0
        while self.pose is None or not self.joints or self.cargo is None:
            if goal.is_cancel_requested or time.monotonic() > until:
                raise RuntimeError('FEEDBACK_NOT_READY')
            time.sleep(0.05)

    def _gazebo_jekkoskillserverbase_freeze_targets(self):
        for n in self.targets:
            if n in self.joints:
                self.targets[n] = 0.0 if n in self.velocity_joints else self.joints[n]

    def _gazebo_jekkoskillserverbase_stop(self):
        self.drive()
        for n in self.velocity_joints:
            self.targets[n] = 0.0
        self.hold()

    @staticmethod
    def valid(p, q):
        return all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < .05

    def on_pose(self, msg, robot):
        # Model pose is absolute; link-relative transforms must never be treated as world poses.
        for t in msg.transforms:
            if t.child_frame_id.replace('::', '/').split('/')[-1] != robot:
                continue
            p, q = t.transform.translation, t.transform.rotation
            if not self.valid(p, q):
                return
            pose = (p.x, p.y, p.z, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            self.entities[robot] = (pose, time.monotonic())
            if robot == self.robot:
                self.pose, self.pose_time = pose, time.monotonic()
                self.orientation = (q.x,q.y,q.z,q.w)
            return

    def on_cargo(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        if msg.header.frame_id == 'world' and self.valid(p, q):
            self.cargo = (p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            self.cargo_time = time.monotonic()

    def on_joints(self, msg):
        for n, v in zip(msg.name, msg.position):
            if math.isfinite(v):
                self.joints[n] = v
        self.joints_time = time.monotonic()

    def drive(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.drive_pub.publish(msg)
        self.command_time = time.monotonic()

    def hold(self):
        now = time.monotonic()
        if self.busy and (now-self.command_time > .25/self.motion_scale or now-self.pose_time > 2.):
            self.drive()
        if now-self.joints_time > 2.:
            # Freeze at last measurements on feedback loss instead of completing an old trajectory.
            self.targets = {n: (0. if n in self.velocity_joints else self.joints[n])
                            for n in self.targets if n in self.joints}
        for n, value in list(self.targets.items()):
            # CASK_JEKKO_PITCH_FORCE_V1
            if self.robot == 'jekko' and n == 'jekko_boom_pitch_joint':
                # Use simulation time so slow Gazebo cannot accelerate this trajectory.
                value = self.pitch_position_command(value)
            self.joint_command_publishers[n].publish(Float64(data=float(value)))

    def joint_move(self, goal, targets, stage, tolerance=.006, verify=None, settle=True):
        if any(n not in self.joint_command_publishers or not math.isfinite(v) for n,v in targets.items()):
            raise RuntimeError('INVALID_JOINT_TARGET')
        began, stable, last = time.monotonic(), None, 0.
        while True:
            self.check(goal,self.loaded)
            now = time.monotonic()
            if now-began > self.get_parameter('motion_timeout').value:
                raise RuntimeError('JOINT_MOTION_TIMEOUT: '+stage)
            if any(n not in self.joints for n in targets):
                raise RuntimeError('JOINT_FEEDBACK_MISSING: '+stage)
            self.targets.update(targets)
            self.hold()
            # CASK_JEKKO_APPLIED_LEVEL_TARGET_V1: compare the reference actually sent by hold().
            effective_targets = dict(targets)
            if self.robot == 'jekko' and 'jekko_hook_level_joint' in effective_targets:
                effective_targets['jekko_hook_level_joint'] = getattr(
                    self, 'applied_hook_level_target', effective_targets['jekko_hook_level_joint'])
            error = max(abs(self.joints[n]-v) for n,v in effective_targets.items())
            if error <= tolerance and (verify is None or verify()):
                if stable is None:
                    stable = now
                elif now-stable > (.35/self.motion_scale if settle else 0.):
                    return
            else:
                stable = None
            if now-last > .3:
                # CASK_JEKKO_JOINT_FEEDBACK_V1
                if self.robot == 'jekko':
                    worst = max(effective_targets, key=lambda n: abs(self.joints[n]-effective_targets[n]))
                    pending = ','.join(f'{n}:{self.joints[n]:.5f}->{v:.5f}'
                        for n,v in effective_targets.items() if abs(self.joints[n]-v) > tolerance)
                    self.feedback(goal, 50., f'{stage} | measured_joint_error={error:.5f}'
                        f' | worst_joint={worst} | measured={self.joints[worst]:.5f}'
                        f' | target={effective_targets[worst]:.5f} | requested_target={targets[worst]:.5f} | pending={pending}')
                else:
                    self.feedback(goal, 50., f'{stage} | measured_joint_error={error:.5f}')
                last = now
            time.sleep(.04/self.motion_scale)

    @staticmethod
    def pose_param(p, prefix, yaw=True):
        fields = ('x','y','z','yaw') if yaw else ('x','y','z')
        return tuple(p[prefix+'_'+s] for s in fields)

    def scene(self, p):
        revision, shared = self.obstacles.ground_snapshot()
        obstacles = [(x,y,.45) for x,y in shared]
        if self.robot == 'jekko' and self.cargo is not None and not self.loaded:
            obstacles.append((self.cargo[0], self.cargo[1], .48))
        # Work support and crane base are tested as rectangles by Task005RoutePlanner.
        for name,(pose,received) in self.entities.items():
            if name == self.robot:
                continue
            if time.monotonic()-received > 2.:
                raise RuntimeError('SCENE_FEEDBACK_LOST: '+name)
            x,y,_,yaw = pose
            if name == 'jekko':
                obstacles += [(x+d*math.cos(yaw),y+d*math.sin(yaw),.43) for d in (-.5,0.,.5)]
                # Stationary deployed support feet; include their actual measured extensions.
                for mx,my,a in ((.64,.19,.72),(.64,-.19,-.72),(-.65,.19,2.42),(-.65,-.19,-2.42)):
                    lx,ly = mx+1.02*math.cos(a),my+1.02*math.sin(a)
                    obstacles.append((x+lx*math.cos(yaw)-ly*math.sin(yaw),y+lx*math.sin(yaw)+ly*math.cos(yaw),.14))
            elif name == 'innok223':
                # Conservative tractor/trailer envelope from its measured tractor pose.
                obstacles += [(x+d*math.cos(yaw),y+d*math.sin(yaw),.85)
                              for d in (-2.0,-1.0,0.0)]
        return (revision, tuple(round(v,2) for o in obstacles for v in o)), tuple(obstacles)

    def navigate(self, goal, target, p, stage):
        began, last, stable = time.monotonic(), 0., None
        index, path, signature = 0, [], None
        controller_type = JekkoPoseController if self.robot == 'jekko' else PoseController
        controller = controller_type(.3, .25,
                                    arrival_tolerance=(.035,.025), fixed_pivot=0.)
        deadline = [float('inf')]
        def health():
            self.check(goal, True)
            if time.monotonic() > deadline[0]:
                raise RuntimeError('PLANNING_TIMEOUT')
        planner = Task005RoutePlanner(self.robot, health)
        x,y,_,yaw = self.pose_param(p,'delivery')
        planner.static_boxes = ((x,y,yaw,-.4,.4,-.25,.25),self.crane_base_obstacle)
        try:
            while True:
                self.check(goal, True)
                now = time.monotonic()
                if now-began > self.get_parameter('motion_timeout').value:
                    raise RuntimeError('NAVIGATION_TIMEOUT')
                pose = (self.pose[0],self.pose[1],self.pose[3])
                sig, obstacles = self.scene(p)
                if not planner.pose_clear(planner.virtual_pose(pose), obstacles):
                    raise RuntimeError('PATH_BLOCKED: current body overlaps an obstacle')
                if math.dist(pose[:2],target[:2]) <= .035 and (self.robot == 'jekko' or abs(angle(pose[2]-target[2])) <= .025):
                    self.drive()
                    if stable is None:
                        stable = now
                    elif now-stable > .5:
                        return
                    time.sleep(.04/self.motion_scale)
                    continue
                stable = None
                if not path or signature != sig:
                    self.drive()
                    deadline[0] = now+6.
                    path = planner.plan_route(pose, target, obstacles)
                    deadline[0] = float('inf')
                    if not path:
                        raise RuntimeError('PATH_BLOCKED: no route to ontology work pose')
                    index, signature = 0, sig
                    controller.align_final = False
                    self.feedback(goal, 5., stage+' | PATH_READY')
                while index < len(path)-1 and math.dist(pose[:2],path[index]) < .12:
                    index += 1
                v,w,status = controller.command(planner.virtual_pose(pose),path[index],planner.virtual_pose(target),index == len(path)-1)
                if planner.reverse:
                    v = -v
                v, w = v*self.motion_scale, w*self.motion_scale
                horizon = .35/self.motion_scale
                predicted = (pose[0]+horizon*v*math.cos(pose[2]),pose[1]+horizon*v*math.sin(pose[2]),pose[2]+horizon*w)
                if not planner.pose_clear(planner.virtual_pose(predicted), obstacles):
                    self.drive()
                    path = []
                    time.sleep(.04/self.motion_scale)
                    continue
                self.drive(v,w)
                if now-last > .3:
                    self.feedback(goal,50.,f'{stage} | {status} | position_error={math.dist(pose[:2],target[:2]):.3f} | v={v:.3f} | w={w:.3f} | yaw={pose[2]:.3f}')
                    last = now
                time.sleep(.04/self.motion_scale)
        finally:
            self.drive()

    def verify_delivery(self, p):
        delivery = self.pose_param(p,'delivery')
        if math.dist(self.cargo[:3], delivery[:3]) > .08 or abs(angle(self.cargo[3]-delivery[3])) > .06:
            raise RuntimeError('COMPONENT_NOT_AT_WORK_POSE')

class JekkoGazeboMotion:
    def _gazebo_jekkoserver_setup_0(self):
        commands = {f'jekko_{j}_joint': f'/jekko/{j}_cmd' for j in ('slew', 'boom_pitch', 'telescope_1', 'telescope_2', 'hook_level', 'hoist', 'hook_yaw')}
        for side in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            for j in ('extension', 'foot'):
                commands[f'jekko_{side}_{j}_joint'] = f'/jekko/{side}_{j}_cmd'
        return (commands,)

    def _gazebo_jekkoserver_setup_1(self):
        self.attachment = NativeAttachment(child_model='task005_steel', topic='/cask/jekko/task005', parent_model='jekko', parent_link='jekko_cargo_attachment_link')
        self.create_subscription(String, '/cask/jekko/task005/state', lambda msg: self.on_native(msg) if getattr(self, 'generated_selected_task', 'Task_005') == 'Task_005' else None, 10)
        self.cable_visual = JekkoCableVisual(self.get_logger())
        self.declare_parameter('planning_timeout', 15.0)

    def _gazebo_jekkoserver_close_physics(self):
        self.cable_visual.close()
        self.attachment.close()

    def joint_move(self, goal, targets, stage, **kwargs):
        ranges = {'RETRACTING_SUPPORT_FEET': (0.0, 1.0), 'RETRACTING_SUPPORTS': (1.0, 2.0), 'STOWING_FOR_DRIVE': (2.0, 5.0), 'DRIVING_TO_WORK': (5.0, 20.0), 'DEPLOYING_SUPPORTS': (20.0, 25.0), 'LOWERING_SUPPORT_FEET': (25.0, 30.0), 'RAISING_HOOK': (30.0, 35.0), 'MOVING_TO_PICKUP': (35.0, 40.0), 'LOWERING_TO_PICKUP': (40.0, 45.0), 'LIFTING': (45.0, 55.0), 'TRANSPORTING': (55.0, 80.0), 'LOWERING': (80.0, 90.0), 'WITHDRAWING_HOOK': (92.0, 96.0), 'CLEARING_WORK_AREA': (96.0, 99.0)}
        key = stage.split(' | ', 1)[0]
        context = getattr(self, '_jekko_progress_context', None)
        if key in ranges and (context is None or context[0] != key or context[1] != 'hook'):
            first, last = ranges[key]
            self._jekko_progress_context = (key, 'joint', {n: self.joints.get(n,v) for n,v in targets.items()}, dict(targets), first, last)
        targets = dict(targets)
        for name in ('jekko_slew_joint', 'jekko_hook_yaw_joint'):
            if name in targets and name in self.joints:
                targets[name] = self.joints[name]+angle(targets[name]-self.joints[name])
        return super().joint_move(goal, targets, stage, **kwargs)

    def on_native(self, msg):
        if msg.data in ('attached','detached'):
            self.native_state = msg.data
            self.native_revision += 1
            self.loaded = msg.data == 'attached'

    def native_wait(self, goal, wanted, before):
        until = time.monotonic()+6.
        while time.monotonic() < until:
            self.check(goal, True)
            if self.native_revision > before and self.native_state == wanted:
                self.loaded = wanted == 'attached'
                return
            time.sleep(.05)
        raise RuntimeError('ATTACHMENT_NOT_CONFIRMED: '+wanted)

    def prepare_drive_to_work(self, goal, p, pickup, delivery, safe_z, gap):
        work = self.pose_param(p, 'jekko_work')
        expected = (work[0], work[1], self.pose[2], work[3])
        # Validate the destination before driving; validate actual measured pose again afterward.
        for point in ((*pickup[:2], pickup[2]+gap), (*pickup[:2], safe_z+gap),
                      (*delivery[:2], safe_z+gap), (*delivery[:2], delivery[2]+gap)):
            jekko_targets(expected, point, delivery[3])
        sides = ('front_left', 'front_right', 'rear_left', 'rear_right')
        self.joint_move(goal, {f'jekko_{side}_foot_joint': 0. for side in sides},
                        'RETRACTING_SUPPORT_FEET')
        self.joint_move(goal, {f'jekko_{side}_extension_joint': 0. for side in sides},
                        'RETRACTING_SUPPORTS')
        self.joint_move(goal, {'jekko_slew_joint': 0., 'jekko_boom_pitch_joint': .3,
            'jekko_telescope_1_joint': 0., 'jekko_telescope_2_joint': 0.,
            'jekko_hoist_joint': 0., 'jekko_hook_level_joint': .3,
            'jekko_hook_yaw_joint': 0.}, 'STOWING_FOR_DRIVE')
        until = time.monotonic()+10.
        while 'innok223' not in self.entities:
            self.check(goal, True)
            if time.monotonic() > until:
                raise RuntimeError('SCENE_FEEDBACK_NOT_READY: Innok223 pose missing')
            time.sleep(.05)
        self.navigate(goal, (work[0], work[1], work[3]), p, 'DRIVING_TO_WORK')
        self.drive()
        self.feedback(goal, 10., 'WORK_AREA_REACHED | stopped_before_support_deployment=true')

    def pitch_position_command(self, target):
        now = self.get_clock().now().nanoseconds*1e-9
        measured = self.joints.get('jekko_boom_pitch_joint')
        if measured is None or not math.isfinite(measured):
            raise RuntimeError('JOINT_FEEDBACK_MISSING: jekko_boom_pitch_joint')
        target = max(0., min(1.19, float(target)))
        stamp = getattr(self, '_pitch_command_time', None)
        if stamp is None or now < stamp:
            self._pitch_command = max(0., min(1.19, measured))
            self._pitch_command_time = now
            return self._pitch_command
        dt = min(.1, max(0., now-stamp))
        delta = max(-.18*self.motion_scale*dt, min(.18*self.motion_scale*dt, target-self._pitch_command))
        self._pitch_command = max(0., min(1.19, self._pitch_command+delta))
        self._pitch_command_time = now
        return self._pitch_command

    def work_pose_reachable(self, p):
        pickup, delivery = self.pose_param(p, 'pickup'), self.pose_param(p, 'delivery')
        gap = .6
        safe_z = max(p['target_height'], pickup[2]+.4, delivery[2]+.4)
        try:
            for point in ((*pickup[:2],pickup[2]+gap), (*pickup[:2],safe_z+gap),
                          (*delivery[:2],safe_z+gap), (*delivery[:2],delivery[2]+gap)):
                jekko_targets(self.pose, point, delivery[3])
        except ValueError:
            return False
        return True

    def navigate(self, goal, target, p, stage, planner_setup=None):
        # Jekko-only override: upper slew performs hook alignment after parking.
        self._jekko_progress_context = (stage.split(' | ', 1)[0], 'drive', tuple(self.pose), tuple(target), 5., 20.)
        began, last, stable = time.monotonic(), 0., None
        index, path, signature = 0, [], None
        last_shortcut = -math.inf
        stopping = False
        previous = None
        linear_speed, angular_speed = float('inf'), float('inf')
        controller = JekkoPoseController(.3, .25, arrival_tolerance=(.035,.025), fixed_pivot=0.)
        deadline = [float('inf')]
        def health():
            self.check(goal, True)
            if time.monotonic() > deadline[0]:
                raise RuntimeError('PLANNING_TIMEOUT')
        planner = Task005RoutePlanner(self.robot, health)
        x,y,_,yaw = self.pose_param(p,'delivery')
        planner.static_boxes = ((x,y,yaw,-.4,.4,-.25,.25),self.crane_base_obstacle)
        if planner_setup is not None:
            planner_setup(planner)
        try:
            while True:
                self.check(goal, True)
                now = time.monotonic()
                if now-began > self.get_parameter('motion_timeout').value:
                    raise RuntimeError('NAVIGATION_TIMEOUT')
                pose = (self.pose[0],self.pose[1],self.pose[3])
                sim_now = self.get_clock().now().nanoseconds*1e-9
                if previous is None:
                    previous = (self.pose_time, sim_now, pose)
                elif self.pose_time != previous[0] and sim_now > previous[1]:
                    dt = sim_now-previous[1]
                    linear_speed = math.dist(pose[:2],previous[2][:2])/dt
                    angular_speed = abs(angle(pose[2]-previous[2][2]))/dt
                    previous = (self.pose_time, sim_now, pose)
                sig, obstacles = self.scene(p)
                virtual = planner.virtual_pose(pose)
                if not planner.pose_clear(virtual, obstacles):
                    raise RuntimeError('PATH_BLOCKED: current body overlaps an obstacle')
                distance = math.dist(pose[:2],target[:2])
                reachable = self.work_pose_reachable(p) if distance <= .24 else False
                if distance <= .18 and reachable and not stopping:
                    stopping = True
                    self.drive()
                    self.feedback(goal,50.,f'{stage} | WORK_AREA_STOPPING | position_error={distance:.3f} | reachable=true | v=0.000 | w=0.000')
                    last = now
                if stopping:
                    # Latch arrival while wheels brake: never resume heading
                    # corrections just because a few centimetres of slip occur.
                    self.drive()
                    if distance > .24 or not reachable:
                        stopping, stable = False, None
                        path = []
                    else:
                        if linear_speed <= .035 and angular_speed <= .06:
                            if stable is None:
                                stable = sim_now
                            elif sim_now-stable >= .25:
                                return
                        else:
                            stable = None
                        if now-last > .3:
                            self.feedback(goal,50.,f'{stage} | WORK_AREA_STOPPING | position_error={distance:.3f} | reachable=true | v=0.000 | w=0.000')
                            last = now
                        time.sleep(.04/self.motion_scale)
                        continue
                if distance <= .035 and not reachable:
                    self.drive()
                    raise RuntimeError('JEKKO_WORK_POSE_NOT_REACHABLE: measured parking cannot cover all hook targets')
                if (path and signature!=sig and getattr(planner,'reuse_clear_path',False)
                        and planner.route_still_clear(virtual,path[index:],planner.virtual_pose(target),obstacles)):
                    signature = sig
                if not path or signature != sig:
                    self.drive()
                    planning_started = time.monotonic()
                    budget = float(self.get_parameter('planning_timeout').value)
                    if not math.isfinite(budget) or budget <= 0.:
                        raise RuntimeError('INVALID_PLANNING_TIMEOUT')
                    deadline[0] = planning_started+budget
                    self.feedback(goal,5.,f'{stage} | PLANNING | budget={budget:.1f}s | start=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) | target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})')
                    try:
                        path = planner.plan_route(pose, target, obstacles)
                    finally:
                        deadline[0] = float('inf')
                    planning_seconds = time.monotonic()-planning_started
                    if not path:
                        raise RuntimeError('PATH_BLOCKED: no route to ontology work pose')
                    index, signature = 0, sig
                    self.feedback(goal,5.,f'{stage} | PATH_READY | planning_seconds={planning_seconds:.3f} | start=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) | target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})')
                shortcut = (getattr(planner,'shortcut_clear_path',False)
                            and now-last_shortcut>=.3)
                if shortcut:
                    last_shortcut = now
                while index < len(path)-1:
                    close = (math.dist(pose[:2],path[index]) <= .25
                             or shortcut)
                    next_point = path[index+1]
                    bearing = math.atan2(next_point[1]-pose[1],next_point[0]-pose[0])
                    if not (close and planner.turn_clear(pose[:2],virtual[2],bearing,obstacles)
                            and planner.segment_clear(pose[:2],next_point,obstacles)):
                        break
                    index += 1
                v,w,status = controller.command(planner.virtual_pose(pose),path[index],planner.virtual_pose(target),index == len(path)-1)
                if planner.reverse:
                    v = -v
                v,w = v*self.motion_scale, w*self.motion_scale
                horizon = .35/self.motion_scale
                predicted = (pose[0]+horizon*v*math.cos(pose[2]),pose[1]+horizon*v*math.sin(pose[2]),pose[2]+horizon*w)
                if not planner.pose_clear(planner.virtual_pose(predicted),obstacles):
                    self.drive()
                    path = []
                    time.sleep(.04/self.motion_scale)
                    continue
                self.drive(v,w)
                if now-last > .3:
                    self.feedback(goal,50.,f'{stage} | {status} | position_error={distance:.3f} | v={v:.3f} | w={w:.3f} | yaw={pose[2]:.3f}')
                    last = now
                time.sleep(.04/self.motion_scale)
        finally:
            self.drive()

    def on_cargo(self, msg):
        super().on_cargo(msg)
        q = msg.pose.orientation
        if self.cargo is not None and msg.header.frame_id == 'world' and self.valid(msg.pose.position,q):
            self.cargo_orientation = (q.x,q.y,q.z,q.w)

    def _lifting_eye(self, offset):
        shift = rotate((0.,0.,offset), getattr(self,'cargo_orientation',(0.,0.,0.,1.)))
        return tuple(a+b for a,b in zip(self.cargo[:3],shift))

    def _hook_pose(self):
        q = self.joints
        length = 2.12+q['jekko_telescope_1_joint']+q['jekko_telescope_2_joint']
        elevation = .20+q['jekko_boom_pitch_joint']
        slew = q['jekko_slew_joint']
        tilt = q['jekko_hook_level_joint']-q['jekko_boom_pitch_joint']
        paid_out = q['jekko_hoist_joint']+.58
        point = (-.08+length*math.cos(elevation)*math.cos(slew)-paid_out*math.sin(tilt)*math.cos(slew),
                 length*math.cos(elevation)*math.sin(slew)-paid_out*math.sin(tilt)*math.sin(slew),
                 1.11+length*math.sin(elevation)-paid_out*math.cos(tilt))
        world = rotate(point,self.orientation)
        return (*tuple(a+b for a,b in zip(self.pose[:3],world)),
                angle(self.pose[3]+slew+q['jekko_hook_yaw_joint']))

    def _fixed_hoist_targets(self, point, yaw, hoist):
        local = rotate(tuple(a-b for a,b in zip(point,self.pose[:3])),self.orientation,inverse=True)
        down = rotate((0.,0.,-1.),self.orientation,inverse=True)
        slew = self.joints['jekko_slew_joint']
        # The levelling axis compensates the gravity component in the boom plane.
        for _ in range(4):
            tilt = math.atan2(-(down[0]*math.cos(slew)+down[1]*math.sin(slew)),-down[2])
            d = (-math.sin(tilt)*math.cos(slew),-math.sin(tilt)*math.sin(slew),-math.cos(tilt))
            head = tuple(v-(hoist+.58)*dv for v,dv in zip(local,d))
            slew = math.atan2(head[1],head[0]+.08)
        radius, rise = math.hypot(head[0]+.08,head[1]), head[2]-1.11
        length = math.hypot(radius,rise)
        pitch = math.atan2(rise,radius)-.20
        if not (2.12-.001 <= length <= 4.42+.001 and 0 <= pitch <= 1.19 and 0 <= hoist <= 3):
            raise RuntimeError('JEKKO_FIXED_WIRE_TARGET_OUT_OF_REACH')
        extension = max(0.,min(1.15,(length-2.12)/2))
        return {'jekko_slew_joint':angle(slew),'jekko_boom_pitch_joint':pitch,
                'jekko_telescope_1_joint':extension,'jekko_telescope_2_joint':extension,
                'jekko_hook_level_joint':max(-1.4,min(1.4,pitch+tilt)),
                'jekko_hoist_joint':hoist,'jekko_hook_yaw_joint':angle(yaw-self.pose[3]-slew)}

    def level_position_command(self, target):
        pitch = self.joints.get('jekko_boom_pitch_joint')
        if pitch is None:
            return target
        down = rotate((0.,0.,-1.),self.orientation,inverse=True)
        slew = self.joints.get('jekko_slew_joint',0.)
        tilt = math.atan2(-(down[0]*math.cos(slew)+down[1]*math.sin(slew)),-down[2])
        velocity = getattr(self,'joint_velocities',{}).get('jekko_boom_pitch_joint',0.)
        # Feed-forward cancels the velocity-mode controller's following lag.
        lead = max(-.045,min(.045,velocity/8.))
        return max(-1.4,min(1.4,pitch+tilt+lead))

    def joint_position_command(self, name, target):
        return target

    def attach_cargo(self, goal):
        return self.attachment.attach()

    def move_hook(self, goal, destination, yaw, stage):
        # CASK_JEKKO_PROGRESS_CONTEXT_V2
        ranges = {'RETRACTING_SUPPORT_FEET': (0.0, 1.0), 'RETRACTING_SUPPORTS': (1.0, 2.0), 'STOWING_FOR_DRIVE': (2.0, 5.0), 'DRIVING_TO_WORK': (5.0, 20.0), 'DEPLOYING_SUPPORTS': (20.0, 25.0), 'LOWERING_SUPPORT_FEET': (25.0, 30.0), 'RAISING_HOOK': (30.0, 35.0), 'MOVING_TO_PICKUP': (35.0, 40.0), 'LOWERING_TO_PICKUP': (40.0, 45.0), 'LIFTING': (45.0, 55.0), 'TRANSPORTING': (55.0, 80.0), 'LOWERING': (80.0, 90.0), 'WITHDRAWING_HOOK': (92.0, 96.0), 'CLEARING_WORK_AREA': (96.0, 99.0)}
        key = stage.split(' | ', 1)[0]
        first, last = ranges.get(key, (getattr(self, '_jekko_skill_progress', 0.), getattr(self, '_jekko_skill_progress', 0.)))
        self._jekko_progress_context = (key, 'hook', tuple(self._hook_pose()), (*destination, yaw), first, last)
        start = self._hook_pose()
        if math.dist(start[:3],destination) <= .012 and abs(angle(yaw-start[3])) <= .01:
            self.check(goal,self.loaded)
            return
        vertical = math.dist(start[:2],destination[:2]) < .025 and abs(angle(yaw-start[3])) < .02
        current_hoist = max(0.,self.joints['jekko_hoist_joint'])
        fixed = current_hoist
        if vertical:
            tilt = self.joints['jekko_hook_level_joint']-self.joints['jekko_boom_pitch_joint']
            slew = self.joints['jekko_slew_joint']
            down = rotate((-math.sin(tilt)*math.cos(slew),-math.sin(tilt)*math.sin(slew),-math.cos(tilt)),self.orientation)
            required = current_hoist+(destination[2]-start[2])/down[2]
            if 0 <= required <= 3:
                # Lower/retract the wire alone. Boom targets stay held.
                self.joint_move(goal,{'jekko_hoist_joint':required},stage,tolerance=.003)
                if math.dist(self._hook_pose()[:3],destination) <= .025:
                    return
                fixed = required
            elif required < 0:
                if current_hoist > .005:
                    self.joint_move(goal,{'jekko_hoist_joint':0.},stage+' | RETRACTING_WIRE',tolerance=.003)
                fixed = 0.
            else:
                raise RuntimeError('JEKKO_LIFTING_EYE_EXCEEDS_HOIST_RANGE')
        start = self._hook_pose()
        steps = max(1,math.ceil(max(math.dist(start[:3],destination)/.02,abs(angle(yaw-start[3]))/.015)))
        for i in range(1,steps+1):
            f = i/steps
            point = tuple(a+(b-a)*f for a,b in zip(start[:3],destination))
            heading = start[3]+angle(yaw-start[3])*f
            self.joint_move(goal,self._fixed_hoist_targets(point,heading,fixed),stage,tolerance=.016,settle=False)
        # Refresh IK if support/chassis pose changed while following the path.
        until = time.monotonic()+12.
        while True:
            self.check(goal,self.loaded)
            targets = self._fixed_hoist_targets(destination,yaw,fixed)
            self.joint_move(goal,targets,stage,tolerance=.004,settle=False)
            if math.dist(self._hook_pose()[:3],destination) <= .025:
                return
            if time.monotonic() > until:
                raise RuntimeError('JEKKO_HOOK_POSITION_NOT_CONVERGED')

    def on_joints(self, msg):
        continuous = ('jekko_slew_joint', 'jekko_hook_yaw_joint')
        previous = {name:self.joints[name] for name in continuous if name in self.joints}
        super().on_joints(msg)
        for name, old in previous.items():
            self.joints[name] = old+angle(self.joints[name]-old)
        visual = getattr(self, 'cable_visual', None)
        if visual is not None:
            visual.update(self.pose, self.orientation, self.joints)
        self.joint_velocities = {name:value for name,value in zip(msg.name,msg.velocity) if math.isfinite(value)}

    def hold(self):
        now = time.monotonic()
        if self.busy and (now-self.command_time > .25/self.motion_scale or now-self.pose_time > 2.):
            self.drive()
        if now-self.joints_time > 2.:
            # Freeze at last measurements on feedback loss instead of completing an old trajectory.
            self.targets = {n: (0. if n in self.velocity_joints else self.joints[n])
                            for n in self.targets if n in self.joints}
        for n, value in list(self.targets.items()):
            # CASK_JEKKO_PITCH_FORCE_V1
            if self.robot == 'jekko' and n == 'jekko_boom_pitch_joint':
                # Use simulation time so slow Gazebo cannot accelerate this trajectory.
                value = self.pitch_position_command(value)
            if n == 'jekko_hook_level_joint':
                value = self.level_position_command(value)
            value = self.joint_position_command(n,value)
            if n == 'jekko_hook_level_joint':
                self.applied_hook_level_target = float(value)
            self.joint_command_publishers[n].publish(Float64(data=float(value)))

    def run_operation(self, goal, operation, p):
        pickup, delivery = self.pose_param(p,'pickup'), self.pose_param(p,'delivery')
        if math.dist(self.cargo[:3],pickup[:3]) > .15:
            raise RuntimeError('PICKUP_POSE_MISMATCH')
        if self.loaded:
            raise RuntimeError('LOAD_ALREADY_ATTACHED')
        gap = float(p.get('rigging_gap',.6)) if p.get('final_scene')==1. else .6
        if not math.isfinite(gap) or not .05<=gap<=1.:
            raise RuntimeError('INVALID_RIGGING_GAP')
        safe_z = max(p['target_height'], pickup[2]+.4, delivery[2]+.4)
        self.prepare_drive_to_work(goal, p, pickup, delivery, safe_z, gap)
        # Prevalidate the entire hook workspace before deploying or attaching.
        for point in ((*pickup[:2],pickup[2]+gap),(*pickup[:2],safe_z+gap),
                      (*delivery[:2],safe_z+gap),(*delivery[:2],delivery[2]+gap)):
            jekko_targets(self.pose,point,delivery[3])
        support = {f'jekko_{side}_extension_joint':.22 for side in ('front_left','front_right','rear_left','rear_right')}
        self.joint_move(goal,support,'DEPLOYING_SUPPORTS')
        feet = {f'jekko_{side}_foot_joint':max(0.,min(.21,self.pose[2]+.173))
                for side in ('front_left','front_right','rear_left','rear_right')}
        self.joint_move(goal,feet,'LOWERING_SUPPORT_FEET')
        hook = self._hook_pose()
        approach_z = max(hook[2],safe_z+gap)
        self.move_hook(goal,(*hook[:2],approach_z),hook[3],'RAISING_HOOK')
        self.move_hook(goal,(*pickup[:2],approach_z),pickup[3],'MOVING_TO_PICKUP')
        self.move_hook(goal,self._lifting_eye(gap),pickup[3],'LOWERING_TO_PICKUP | to_component_lifting_eye=true')
        before = self.native_revision
        self.attach_cargo(goal)
        # Conservatively guard fallback from the instant the plugin can attach.
        self.loaded = True
        try:
            self.native_wait(goal,'attached',before)
        except RuntimeError as error:
            if not str(error).startswith('ATTACHMENT_NOT_CONFIRMED'):
                raise
            self.release_attachment(goal)
            before = self.native_revision
            self.attach_cargo(goal)
            self.loaded = True
            self.native_wait(goal,'attached',before)
        self.move_hook(goal,(*pickup[:2],safe_z+gap),pickup[3],'LIFTING')
        if self.cargo[2] < pickup[2]+.20:
            raise RuntimeError('LOAD_CLEARANCE_NOT_VERIFIED')
        self.move_hook(goal,(*delivery[:2],safe_z+gap),delivery[3],'TRANSPORTING')
        self.move_hook(goal,(*delivery[:2],delivery[2]+gap+.008),delivery[3],'LOWERING')
        self.verify_delivery(p)
        self.release_attachment(goal, verify=lambda: self.verify_delivery(p))
        self.move_hook(goal,(*delivery[:2],safe_z+gap),delivery[3],'WITHDRAWING_HOOK')
        # Withdraw the boom from the processing site before another operation begins.
        self.clear_after_delivery(goal, safe_z+gap)
        until, stable = time.monotonic()+6., None
        while time.monotonic() < until:
            self.check(goal, True)
            self.verify_delivery(p)
            if stable is None:
                stable = time.monotonic()
            elif time.monotonic()-stable > .7:
                return
            time.sleep(.05)
        raise RuntimeError('DELIVERY_NOT_VERIFIED')

    def clear_after_delivery(self, goal, clearance):
        # CASK_JEKKO_COORDINATED_STOW_V3: shared, feedback-based unloaded stow.
        self.check(goal, False)
        if self.loaded:
            raise RuntimeError('JEKKO_CANNOT_RETRACT_ATTACHED_LOAD')
        q = dict(self.joints)
        slew = q['jekko_slew_joint']
        slew_goal = slew+angle(-slew)
        yaw_goal = q['jekko_hook_yaw_joint']+angle(-q['jekko_hook_yaw_joint'])
        def hook_height(pitch, length, hoist, bearing, tilt):
            c, s = math.cos(bearing), math.sin(bearing)
            paid = .58+hoist
            point = (-.08+length*math.cos(.20+pitch)*c-paid*math.sin(tilt)*c,
                     length*math.cos(.20+pitch)*s-paid*math.sin(tilt)*s,
                     1.11+length*math.sin(.20+pitch)-paid*math.cos(tilt))
            return self.pose[2]+rotate(point,self.orientation)[2]
        def gravity_tilt(bearing):
            down = rotate((0.,0.,-1.),self.orientation,inverse=True)
            return math.atan2(-(down[0]*math.cos(bearing)+down[1]*math.sin(bearing)),-down[2])
        # Check compact-boom height throughout the shortest return turn.
        chosen = None
        lower = max(.30,q['jekko_boom_pitch_joint'])
        if lower <= 1.19:
            for i in range(91):
                pitch = lower+(1.19-lower)*i/90.
                bearings = [slew+(slew_goal-slew)*j/12. for j in range(13)]
                if all(abs(pitch+gravity_tilt(b)) <= 1.4 and
                       hook_height(pitch,2.12,0.,b,gravity_tilt(b)) >= clearance+.08
                       for b in bearings):
                    chosen = pitch
                    break
        if chosen is None:
            raise RuntimeError('JEKKO_RETRACTION_CLEARANCE_UNREACHABLE')
        final = {'jekko_slew_joint':slew_goal, 'jekko_boom_pitch_joint':.3,
                 'jekko_telescope_1_joint':0., 'jekko_telescope_2_joint':0.,
                 'jekko_hoist_joint':0., 'jekko_hook_level_joint':.3,
                 'jekko_hook_yaw_joint':yaw_goal}
        began, last, turning = time.monotonic(), 0., False
        while True:
            self.check(goal, False)
            if self.loaded:
                raise RuntimeError('JEKKO_CANNOT_RETRACT_ATTACHED_LOAD')
            now = time.monotonic()
            if now-began > self.get_parameter('motion_timeout').value:
                raise RuntimeError('JOINT_MOTION_TIMEOUT: COORDINATED_STOW')
            q = dict(self.joints)
            if any(n not in q or not math.isfinite(q[n]) for n in final):
                raise RuntimeError('JOINT_FEEDBACK_MISSING: COORDINATED_STOW')
            extension = max(0.,q['jekko_telescope_1_joint'])+max(0.,q['jekko_telescope_2_joint'])
            tilt = q['jekko_hook_level_joint']-q['jekko_boom_pitch_joint']
            floor = hook_height(q['jekko_boom_pitch_joint'],2.12,
                                q['jekko_hoist_joint'],q['jekko_slew_joint'],tilt)
            slope = hook_height(q['jekko_boom_pitch_joint'],3.12,
                                q['jekko_hoist_joint'],q['jekko_slew_joint'],tilt)-floor
            # Target only as much shortening as current measured pitch/wire
            # permit. No precision stop between raising and shortening.
            required = (clearance+.05-floor)/slope if slope > 1e-6 else extension
            remaining = min(extension,max(0.,required))
            ratio = remaining/extension if extension > 1e-9 else 0.
            if extension <= .04 and q['jekko_hoist_joint'] <= .015 and floor >= clearance+.025:
                turning = True
            if turning and abs(q['jekko_slew_joint']-slew_goal) <= .03:
                # One final convergence check for all joints, with the original
                # tolerance/settling rules and compensated levelling comparison.
                return self.joint_move(goal,final,'CLEARING_WORK_AREA | FINAL_STOW')
            targets = dict(final)
            targets['jekko_boom_pitch_joint'] = chosen
            targets['jekko_hook_level_joint'] = chosen+gravity_tilt(q['jekko_slew_joint'])
            targets['jekko_slew_joint'] = slew_goal if turning else slew
            targets['jekko_telescope_1_joint'] = 0. if turning else max(0.,q['jekko_telescope_1_joint'])*ratio
            targets['jekko_telescope_2_joint'] = 0. if turning else max(0.,q['jekko_telescope_2_joint'])*ratio
            self.targets.update(targets)
            self.hold()
            if now-last >= .3:
                effective = dict(targets)
                effective['jekko_hook_level_joint'] = getattr(self,'applied_hook_level_target',targets['jekko_hook_level_joint'])
                worst = max(effective,key=lambda n:abs(q[n]-effective[n]))
                error = abs(q[worst]-effective[worst])
                self.feedback(goal,50.,f'CLEARING_WORK_AREA | COORDINATED_STOW | measured_joint_error={error:.5f}'
                    f' | worst_joint={worst} | measured={q[worst]:.5f} | target={effective[worst]:.5f}'
                    f' | extension_remaining={extension:.3f} | returning={str(turning).lower()}')
                last = now
            time.sleep(.04/self.motion_scale)


    def release_attachment(self, goal, verify=None):
        # Observe before publishing. Only a fresh native plugin state may
        # clear carrying_phase; command publication alone is not confirmation.
        before_ros = self.native_revision
        before_native, _ = self.attachment.state_snapshot()
        last_feedback = [0.]

        def confirmed():
            revision, state = self.attachment.state_snapshot()
            return ((self.native_revision > before_ros and self.native_state == 'detached')
                    or (revision > before_native and state == 'detached'))

        def tick():
            self.check(goal, True)
            self.drive()
            self.hold()
            if verify is not None:
                verify()
            now = time.monotonic()
            if now-last_feedback[0] >= .5:
                last_feedback[0] = now
                self.feedback(goal,50.,'RELEASING | awaiting_native_detach=true')

        self.attachment.detach(tick=tick, confirmed=confirmed)
        if not confirmed():
            raise RuntimeError('ATTACHMENT_NOT_CONFIRMED: detached')
        self.native_state = 'detached'
        self.loaded = False

# CASK_ALL_ROBOT_CONTEXT_V1
