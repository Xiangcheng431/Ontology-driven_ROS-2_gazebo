"""Final-scene adapters: reuse the existing Crane and Jekko motion methods."""
import json
import math
import threading
import time
from pathlib import Path
from ros_interfaces.action import ExecuteSkill
from rclpy.action import ActionServer
from std_msgs.msg import String, Float64, Empty
from .crane_lifting_server import CraneLiftingServer
from .jekko_skill_server import JekkoServer, spin
from cask_gazebo_motion.task005_geometry import angle


class CarrierGate:
    def setup_carrier_gate(self):
        from ament_index_python.packages import get_package_share_directory
        default = str(Path(get_package_share_directory('cask_component_manager'))/'config/final_scene.json')
        self.declare_parameter('final_scene_config', default)
        self.final_scene = json.loads(Path(self.get_parameter('final_scene_config').value).read_text())
        self.carrier_state = None
        self.carrier_seen = 0.
        self.create_subscription(String, '/cask/final/carrier_state', self.carrier_update, 10)

    def carrier_update(self, message):
        try:
            data = json.loads(message.data)
            if len(data['bed']) != 4 or not all(math.isfinite(v) for v in data['bed']):
                return
            self.carrier_state, self.carrier_seen = data, time.monotonic()
        except (ValueError, TypeError, KeyError):
            return

    def wait_carrier(self, goal, point, progress, feedback):
        """A fresh measured, stopped trailer is required before lowering onto it."""
        stable, until = None, time.monotonic()+180.
        while time.monotonic() < until:
            if goal.is_cancel_requested:
                from cask_gazebo_motion.gazebo_motion import MotionError
                raise MotionError('CANCELED')
            state = self.carrier_state
            ready = (state and time.monotonic()-self.carrier_seen < 1.
                     and state.get('stationary') and not state.get('cargo_locked')
                     and math.dist(state['bed'][:2], point[:2]) < .08
                     and abs(angle(state['bed'][3]-point[3])) < .06)
            if ready:
                stable = stable or time.monotonic()
                if time.monotonic()-stable > .5:
                    return
            else:
                stable = None
            feedback(goal, progress, 'WAITING_FOR_STATIONARY_TRAILER')
            time.sleep(.1)
        raise RuntimeError('TRAILER_RENDEZVOUS_TIMEOUT')


class FinalCraneServer(CarrierGate, CraneLiftingServer):
    def __init__(self):
        super().__init__()
        self.setup_carrier_gate()
        self.transport_action = ActionServer(self, ExecuteSkill,
            '/crane_01/transporting', self.execute_callback,
            cancel_callback=self.cancel_callback, callback_group=self.action_callback_group)
        self.navigation_action = ActionServer(self, ExecuteSkill,
            '/crane/navigate', self.prepare_unload,
            cancel_callback=self.cancel_callback, callback_group=self.action_callback_group)

    @staticmethod
    def nearest_hook_yaw(target, current):
        """Select an equivalent angle within the existing hook joint limits."""
        lower, upper = -6.2831853, 6.2831853
        equivalents = []
        for turn in range(-2,3):
            candidate = target+turn*2*math.pi
            if lower-1e-7 <= candidate <= upper+1e-7:
                equivalents.append(max(lower,min(upper,candidate)))
        if not equivalents:
            raise ValueError('HOOK_YAW_TARGET_OUT_OF_RANGE')
        return min(equivalents,key=lambda value:abs(value-current))

    def prepare_unload(self, goal):
        """Move the empty hook above the future pickup while transport runs."""
        from cask_gazebo_motion.gazebo_motion import MotionError
        result = ExecuteSkill.Result()
        if not self.motion_lock.acquire(blocking=False):
            return self.abort_goal(goal, 'ROBOT_BUSY: another motion is executing')
        try:
            self._generic_failure_begin(goal)
            request = goal.request
            if (len(request.parameter_names) != len(request.parameter_values)
                    or len(request.parameter_names) != len(request.parameter_units)
                    or len(set(request.parameter_names)) != len(request.parameter_names)
                    or not all(math.isfinite(value) for value in request.parameter_values)):
                raise MotionError('INVALID_PREPARATION_PARAMETERS')
            parameters = dict(zip(request.parameter_names, request.parameter_values))
            if parameters.get('final_scene') != 1.:
                raise MotionError('FINAL_SCENE_PARAMETERS_REQUIRED')
            from cask_gazebo_motion.generated_cargo import select_for_goal
            select_for_goal(self, goal, 'crane')
            self.wait_ready(goal)
            # These are the unloading requirement's future input coordinates,
            # read by Skill Manager from the World Model, not today's cargo XY.
            pickup = tuple(parameters['cargo_pickup_'+axis] for axis in ('x','y','z'))
            gap = parameters['rigging_gap']
            hover_height = max(pickup[2]+gap+.5, parameters['cargo_delivery_z']+gap+.5)
            safe_height = max(self.hook_pose()[2], hover_height)
            targets = self.crane_targets(*pickup[:2], safe_height, parameters['pickup_yaw'])
            self.physical_move(goal, {'hook_position': targets['hook_position']},
                               0., 15., 'PREPARING_UNLOAD_CLEARANCE')
            self.physical_move(goal, {key:value for key,value in targets.items() if key!='hook_position'},
                               15., 85., 'PREPOSITIONING_FOR_UNLOAD')
            self.physical_move(goal, {'hook_position': self.world_height_to_joint_position(hover_height)},
                               85., 99., 'PREPARING_UNLOAD_HOVER')
            self.phase_feedback(goal, 100., 'UNLOAD_POSITION_READY | empty_hook=true')
            goal.succeed()
            result.success = True
            result.message = 'Crane empty hook positioned above unloading pickup'
            return result
        except MotionError as error:
            if str(error) == 'CANCELED':
                goal.canceled()
                return self.canceled_result(goal.request)
            return self.abort_goal(goal, str(error))
        except (TypeError, ValueError, KeyError, RuntimeError, OSError) as error:
            return self.abort_goal(goal, str(error))
        finally:
            self.hold_position()
            self.motion_lock.release()

    def wait_ready(self, goal, need_odom=False):
        """After selecting a cargo context, wait for its first measured samples."""
        super().wait_ready(goal, need_odom)
        from cask_gazebo_motion.gazebo_motion import MotionError
        parameters = dict(zip(goal.request.parameter_names, goal.request.parameter_values))
        if parameters.get('final_scene') != 1.:
            return
        until = time.monotonic() + 10.
        last_report = -math.inf
        while True:
            # Cancellation and joint freshness remain mandatory during startup.
            self.check_feedback(goal, need_odom)
            now = time.monotonic()
            pose_ready = (self.cargo_pose is not None
                          and self.cargo_feedback_time is not None
                          and now-self.cargo_feedback_time <= self.feedback_timeout)
            clearance_ready = (self.clearance_description is not None
                               and self.clearance_received is not None
                               and now-self.clearance_received <= self.feedback_timeout)
            if (pose_ready and clearance_ready
                    and math.dist(self.cargo_pose[:3], self.clearance_description['pose']) <= .05):
                return
            if now >= until:
                raise MotionError(
                    'CARGO_FEEDBACK_NOT_READY: waited 10 seconds for '
                    '/cask/final/cargo_pose and /cask/final/cargo_clearance; '
                    f'pose_fresh={pose_ready}; clearance_fresh={clearance_ready}')
            if now-last_report >= .5:
                self.phase_feedback(goal, 0., 'WAITING_FOR_CARGO_FEEDBACK | '
                    f'pose_fresh={pose_ready} | clearance_fresh={clearance_ready}')
                last_report = now
            time.sleep(.05)

    def physical_move(self, goal, targets, start, end, status):
        p = dict(zip(goal.request.parameter_names, goal.request.parameter_values))
        if p.get('final_scene') == 1. and 'hook_yaw_position' in targets:
            # Delivery targets are computed before pickup by the original motion
            # method. Resolve the equivalent angle NOW, at the phase boundary,
            # against the actual hook angle after the preceding movement.
            targets = dict(targets)
            targets['hook_yaw_position'] = self.nearest_hook_yaw(
                targets['hook_yaw_position'], self.hook_yaw_position)
        # Existing lifting can progress in parallel with the trailer approach.
        # Stop only at the actual cargo handover boundary.
        if p.get('final_scene') == 1. and status == 'LOWERING' and abs(p['cargo_delivery_z']-(self.final_scene['trailer']['deck_height']+self.final_scene['cargo']['size'][2]/2))<.03:
            point = tuple(p['cargo_delivery_'+a] for a in ('x','y','z','yaw'))
            self.wait_carrier(goal, point, start, self.phase_feedback)
        return super().physical_move(goal, targets, start, end, status)


class FinalJekkoServer(CarrierGate, JekkoServer):
    def __init__(self):
        self._joint_command_lock = threading.RLock()
        super().__init__()
        self.setup_carrier_gate()
        from ros_interfaces.action import ExecuteSkill
        from rclpy.action import CancelResponse
        self.navigation_action = ActionServer(self, ExecuteSkill, '/jekko/navigate',
            execute_callback=lambda goal:self.execute(goal,'prepare'),
            goal_callback=self.accept, cancel_callback=lambda _:CancelResponse.ACCEPT,
            callback_group=self.group)
        self._final_prepared = None
        self._final_scene_active = False
        self._final_nav_planner = None
        self._loading_handoff_released = False
        self._motion_speed_factor = .875
        self.motion_scale *= self._motion_speed_factor
        self.declare_parameter('final_hoist_velocity_control',False)
        self._hoist_velocity_enabled = self.get_parameter('final_hoist_velocity_control').value
        self._placement_velocity = None
        self._vertical_level_target = None
        # Establish discovery during bringup, not while holding a placed load.
        self._final_detach_topic = '/cask/jekko/final_steel/detach'
        self._final_detach_pub = self.create_publisher(Empty,self._final_detach_topic,1)
        if self._hoist_velocity_enabled:
            from cask_gazebo_motion.jekko_hoist_servo import HoistServo
            self._hoist_servo = HoistServo()
            self._hoist_velocity_pub = self.create_publisher(Float64,'/jekko/hoist_velocity_cmd',1)
        self.get_logger().info('JEKKO_CONTROL_V3 | speed_factor=0.875 | hoist_velocity_control='
                               +str(self._hoist_velocity_enabled).lower())

    def hold(self):
        # Timer and action callbacks share this method in a reentrant group.
        # Serialize the whole snapshot -> reference update -> publish cycle.
        with self._joint_command_lock:
            result = super().hold()
            if getattr(self,'_hoist_velocity_enabled',False):
                sim = self.get_clock().now().nanoseconds*1e-9
                now = time.monotonic()
                ready = (self.busy and now-self.joints_time<=.25
                         and now-self.pose_time<=.25
                         and 'jekko_hoist_joint' in self.joints)
                if ready:
                    measured = self.joints['jekko_hoist_joint']
                    target = self.targets.get('jekko_hoist_joint',measured)
                    velocity = self._hoist_servo.command(target,measured,sim,self.loaded,
                        self._motion_speed_factor,descent_velocity=self._placement_velocity,
                        measured_velocity=getattr(self,'joint_velocities',{}).get('jekko_hoist_joint',0.))
                else:
                    velocity = self._hoist_servo.stop(sim)
                self._hoist_velocity_pub.publish(Float64(data=velocity))
                self._hoist_velocity_command = velocity
            return result

    def destroy_node(self):
        if getattr(self,'_hoist_velocity_enabled',False):
            self._hoist_velocity_pub.publish(Float64(data=0.))
        return super().destroy_node()

    def feedback(self, goal, progress, status):
        if (getattr(self,'_hoist_velocity_enabled',False)
                and status.split(' | ',1)[0] in (
                    'LOWERING_TO_PICKUP','LIFTING','LOWERING','WITHDRAWING_HOOK')
                and 'wire_velocity_cmd=' not in status):
            status += (f' | wire_measured={self.joints.get("jekko_hoist_joint",0.):.5f}'
                +f' | wire_target={self.targets.get("jekko_hoist_joint",0.):.5f}'
                +f' | wire_velocity_cmd={getattr(self,"_hoist_velocity_command",0.):.5f}')
        return super().feedback(goal,progress,status)

    def level_position_command(self, target):
        if getattr(self,'_vertical_level_target',None) is not None:
            return self._vertical_level_target
        if not (self._final_scene_active and self.loaded):
            return super().level_position_command(target)
        from cask_gazebo_motion.task005_geometry import rotate
        down = rotate((0.,0.,-1.),self.orientation,inverse=True)
        slew = self.joints['jekko_slew_joint']
        tilt = math.atan2(-(down[0]*math.cos(slew)+down[1]*math.sin(slew)),-down[2])
        # Keep gravity levelling, without feeding noisy instantaneous boom
        # velocity into the loaded hook's reference. Shape it with the others.
        return max(-1.4,min(1.4,self.joints['jekko_boom_pitch_joint']+tilt))

    def joint_position_command(self, name, target):
        from cask_gazebo_motion.jekko_loaded_reference import LoadedReference
        if name=='jekko_hoist_joint' and getattr(self,'_hoist_velocity_enabled',False):
            return target
        profiles = {
            'jekko_slew_joint':(.18,.25,(-math.inf,math.inf)),
            'jekko_boom_pitch_joint':(.12,.20,(0.,1.19)),
            'jekko_telescope_1_joint':(.25,.40,(0.,1.15)),
            'jekko_telescope_2_joint':(.25,.40,(0.,1.15)),
            'jekko_hoist_joint':(.30,.40,(0.,3.)),
            'jekko_hook_level_joint':(.20,.30,(-1.4,1.4)),
            'jekko_hook_yaw_joint':(.18,.25,(-math.inf,math.inf))}
        if not self.loaded:
            profiles.update({
                'jekko_slew_joint':(.50,.8,(-math.inf,math.inf)),
                'jekko_boom_pitch_joint':(.36,.6,(0.,1.19)),
                'jekko_telescope_1_joint':(.40,.8,(0.,1.15)),
                'jekko_telescope_2_joint':(.40,.8,(0.,1.15)),
                'jekko_hoist_joint':(.40,.8,(0.,3.)),
                'jekko_hook_level_joint':(.60,1.2,(-1.4,1.4)),
                'jekko_hook_yaw_joint':(.50,.8,(-math.inf,math.inf))})
            for side in ('front_left','front_right','rear_left','rear_right'):
                profiles[f'jekko_{side}_extension_joint'] = (.20,.4,(-.005,.855))
                profiles[f'jekko_{side}_foot_joint'] = (.10,.2,(-.005,.225))
        references = getattr(self,'_loaded_references',None)
        if references is None:
            references = self._loaded_references = {}
        if (self._final_scene_active
                and time.monotonic()-getattr(self,'joints_time',time.monotonic())>2.):
            references.pop(name,None)
            return self.joints.get(name,target)
        if not self._final_scene_active or name not in profiles:
            references.pop(name,None)
            return target
        stamp = self.get_clock().now().nanoseconds*1e-9
        reference = references.get(name)
        if reference is None:
            reference = references[name] = LoadedReference(self.joints[name],stamp)
        factor = getattr(self,'_motion_speed_factor',.875)
        speed,acceleration,bounds = profiles[name]
        end_speed = dict(slow_distance=.02,slow_speed=.06*factor) if name=='jekko_hoist_joint' else {}
        return reference.advance(target,self.joints[name],stamp,speed*factor,
                                 acceleration*factor*factor,bounds,**end_speed)

    def cargo_bottom(self):
        from cask_gazebo_motion.task005_geometry import rotate
        q = getattr(self,'cargo_orientation',(0.,0.,0.,1.))
        size = self.final_scene['cargo']['size']
        axes = ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
        half_height = sum(abs(rotate(axis,q)[2])*length/2 for axis,length in zip(axes,size))
        return self.cargo[2]-half_height

    def support_height(self, p):
        height = p['delivery_z']-self.final_scene['cargo']['size'][2]/2
        if abs(height-self.final_scene['trailer']['deck_height'])<.03:
            state = getattr(self,'carrier_state',None)
            if state and time.monotonic()-self.carrier_seen<1.:
                height = state['bed'][2]
        return height

    def soft_place(self, goal, p, hook):
        self.check(goal,True)
        if not getattr(self,'_hoist_velocity_enabled',False):
            raise RuntimeError('FINAL_HOIST_VELOCITY_CONTROL_REQUIRED: restart with final launch')
        self._final_motion_stage = 'LOWERING'
        # The last ~3cm use one gently braking profile on both supports.
        # Keep a finite terminal rate, retaining the same 2mm stop and settle.
        finish_speed = .003*self._motion_speed_factor
        # During the vertical stroke, keep the levelling axis at its current
        # gravity-aligned position instead of chasing boom velocity/noise.
        with self._joint_command_lock:
            self._vertical_level_target = self.joints['jekko_hook_level_joint']
            getattr(self,'_loaded_references',{}).pop('jekko_hook_level_joint',None)
        gap = self.cargo_bottom()-self.support_height(p)
        if gap>.03:
            super().move_hook(goal,(*hook[:2],hook[2]-(gap-.03)),hook[3],
                              'LOWERING | ABOVE_SUPPORT')
        started = time.monotonic()
        previous = None
        settled = None
        paused = False
        raised_since = None
        last_report = -math.inf
        try:
            while time.monotonic()-started<60.:
                self.check(goal,True)
                sim = self.get_clock().now().nanoseconds*1e-9
                gap = self.cargo_bottom()-self.support_height(p)
                measured = self.joints['jekko_hoist_joint']
                sample = getattr(self,'cargo_time',sim)
                fresh = previous is None or sample!=previous[2]
                speed = math.inf
                if fresh and previous is not None and sim>previous[0]:
                    speed = math.dist(self.cargo[:3],previous[1])/(sim-previous[0])
                if gap<=.002:
                    paused = True
                    raised_since = None
                if paused and gap>.005:
                    raised_since = sim if raised_since is None else raised_since
                    if sim-raised_since>=.3:
                        paused = False
                elif gap<=.005:
                    raised_since = None
                # This is a velocity request, not a new position target pushed
                # below the surface. A transient rise never commands retraction.
                velocity = 0. if paused else min(.030*self._motion_speed_factor,
                                                max(finish_speed,1.4*(gap-.0015)))
                with self._joint_command_lock:
                    self.targets['jekko_hoist_joint'] = measured
                    self._placement_velocity = velocity
                    self.hold()
                if fresh and previous is not None and sim>previous[0]:
                    if paused and -.003<=gap<=.004 and speed<=.01:
                        settled = sim if settled is None else settled
                        if sim-settled>=.30:
                            self.feedback(goal,89.,'LOWERING | SUPPORT_READY | gap='+f'{gap:.4f}')
                            return
                    else:
                        settled = None
                if fresh:
                    previous = sim,tuple(self.cargo[:3]),sample
                if time.monotonic()-last_report>=.3:
                    self.feedback(goal,85.,'LOWERING | GAP_VELOCITY_CONTROL'
                        +f' | gap={gap:.5f} | wire_measured={measured:.5f}'
                        +f' | wire_velocity_cmd={self._hoist_velocity_command:.5f}'
                        +f' | pitch={self.joints["jekko_boom_pitch_joint"]:.5f}'
                        +f' | level={self.joints["jekko_hook_level_joint"]:.5f}'
                        +f' | base_z={self.pose[2]:.5f} | support_z={self.support_height(p):.5f}'
                        +' | contact_paused='+str(paused).lower())
                    last_report = time.monotonic()
                time.sleep(.02)
            raise RuntimeError('JEKKO_SOFT_PLACEMENT_NOT_SETTLED')
        finally:
            with self._joint_command_lock:
                self._placement_velocity = None
                self.targets['jekko_hoist_joint'] = self.joints['jekko_hoist_joint']
                self._hoist_servo.stop(self.get_clock().now().nanoseconds*1e-9)
                self._hoist_velocity_pub.publish(Float64(data=0.))
                self._hoist_velocity_command = 0.

    def attach_cargo(self, goal):
        if not self._final_scene_active:
            return super().attach_cargo(goal)
        before_ros = self.native_revision
        before_native,_ = self.attachment.state_snapshot()
        try:
            return super().attach_cargo(goal)
        except RuntimeError as error:
            # An empty CLI service reply is ambiguous. Do not confuse it with
            # physical rejection when the newly loaded plugin said attached.
            if str(error).strip()!='Gazebo rejected DetachableJoint load:':
                raise
            # The request may already have attached the cargo. Prevent an
            # unsafe fallback while confirming it; never load a second plugin.
            self.loaded = True
            until = time.monotonic()+6.
            while time.monotonic()<until:
                self.check(goal,True)
                self.drive()
                self.hold()
                revision,state = self.attachment.state_snapshot()
                ros_confirmed = self.native_revision>before_ros and self.native_state=='attached'
                if ros_confirmed or (revision>before_native and state=='attached'):
                    # NativeAttachment.loaded means the plugin is installed;
                    # later operations must use /attach, not load it again.
                    self.attachment.loaded = True
                    if not ros_confirmed:
                        # Feed the fresh observer event to the existing native
                        # wait cache. This is an observed physical state.
                        self.native_state = 'attached'
                        self.native_revision += 1
                    self.loaded = True
                    self.feedback(goal,45.,'ATTACHMENT_CONFIRMED | native_attached=true'
                                  +' | service_reply=empty')
                    return
                time.sleep(.02)
            raise RuntimeError('ATTACHMENT_NOT_CONFIRMED: empty plugin-load reply'
                +f'; observed_native_state={state}; observed_native_revision={revision}') from error

    def release_final_native(self, goal, verify=None):
        """Same native acknowledgement as before, without a per-release CLI."""
        before_ros = self.native_revision
        before_native, _ = self.attachment.state_snapshot()
        began = time.monotonic()
        last_feedback = -math.inf

        def confirmed():
            revision,state = self.attachment.state_snapshot()
            return ((self.native_revision>before_ros and self.native_state=='detached')
                    or (revision>before_native and state=='detached'))

        def tick():
            nonlocal last_feedback
            self.check(goal,True)
            self.drive()
            self.hold()
            if verify is not None:
                verify()
            now = time.monotonic()
            if now-last_feedback>=.3:
                self.feedback(goal,90.,'RELEASING | awaiting_native_detach=true'
                    +f' | elapsed={now-began:.3f} | persistent_bridge=true')
                last_feedback = now

        tick()
        publisher = self._final_detach_pub
        # A missing bridge uses the original command path immediately. Keep
        # the same pre-send revision baseline across both paths, so a delayed
        # acknowledgement cannot be lost at the fallback boundary.
        if (self.attachment.TOPIC+'/detach'==self._final_detach_topic
                and publisher.get_subscription_count()>0):
            next_publish = -math.inf
            while time.monotonic()-began<.75:
                tick()
                if confirmed():
                    break
                now = time.monotonic()
                if now>=next_publish:
                    publisher.publish(Empty())
                    next_publish = now+.15
                time.sleep(.01)
        if not confirmed():
            self.attachment.detach(tick=tick,confirmed=confirmed)
        if not confirmed():
            raise RuntimeError('ATTACHMENT_NOT_CONFIRMED: detached')
        self.native_state = 'detached'
        self.loaded = False
        self.feedback(goal,90.,'RELEASING | native_detach_confirmed=true'
                      +f' | elapsed={time.monotonic()-began:.3f}')

    def release_attachment(self, goal, verify=None):
        if not self._final_scene_active:
            return super().release_attachment(goal,verify=verify)
        result = self.release_final_native(goal,verify=verify)
        # Do not withdraw the empty hook while the detached steel is settling.
        stable = None
        previous = None
        started = time.monotonic()
        while time.monotonic()-started<8.:
            self.check(goal,True)
            self.hold()
            sim = self.get_clock().now().nanoseconds*1e-9
            sample = getattr(self,'cargo_time',sim)
            fresh = previous is None or sample!=previous[2]
            if fresh and previous is not None and sim>previous[0]:
                speed = math.dist(self.cargo[:3],previous[1])/(sim-previous[0])
                if speed<=.01:
                    stable = sim if stable is None else stable
                    if sim-stable>=.30:
                        self._vertical_level_target = None
                        return result
                else:
                    stable = None
            if fresh:
                previous = sim,tuple(self.cargo[:3]),sample
            if verify is not None:
                verify()
            time.sleep(.025)
        raise RuntimeError('JEKKO_RELEASED_CARGO_NOT_SETTLED')

    def joint_move(self, goal, targets, stage, **kwargs):
        key = stage.split(' | ',1)[0]
        if self._final_scene_active:
            self._final_motion_stage = key
        if self._final_scene_active and set(targets)=={'jekko_hoist_joint'}:
            previous_verify = kwargs.get('verify')
            def wire_stopped():
                return (abs(getattr(self,'joint_velocities',{}).get('jekko_hoist_joint',0.))<=.01
                    and abs(getattr(self,'_hoist_velocity_command',0.))<=.01
                    and (previous_verify is None or previous_verify()))
            kwargs['verify'] = wire_stopped
            kwargs['tolerance'] = min(kwargs.get('tolerance',.006),.002)
        if self._final_scene_active and stage=='CLEARING_WORK_AREA | FINAL_STOW':
            p = dict(zip(goal.request.parameter_names,goal.request.parameter_values))
            deck = self.final_scene['trailer']['deck_height']+self.final_scene['cargo']['size'][2]/2
            if abs(p['delivery_z']-deck)<.03:
                # The original clearance controller has already shortened the
                # empty boom and returned the slew. Lift the feet while that
                # compact boom settles directly at its next driving angle.
                if self.loaded:
                    raise RuntimeError('JEKKO_CANNOT_RETRACT_ATTACHED_LOAD')
                targets = dict(targets)
                targets.update(jekko_boom_pitch_joint=.6,jekko_hook_level_joint=.6)
                targets.update({f'jekko_{s}_foot_joint':0. for s in
                    ('front_left','front_right','rear_left','rear_right')})
        if self._final_scene_active and key=='RETRACTING_SUPPORTS':
            # Feet have been confirmed up by the preceding original stage.
            # Finish the empty-boom stow concurrently with leg retraction.
            targets = dict(targets)
            targets.update(jekko_slew_joint=0.,jekko_boom_pitch_joint=.6,
                jekko_telescope_1_joint=0.,jekko_telescope_2_joint=0.,
                jekko_hoist_joint=0.,jekko_hook_level_joint=.6,jekko_hook_yaw_joint=0.)
        if self._final_scene_active and key=='STOWING_FOR_DRIVE':
            # A moderate compact stow keeps the hook clear during driving.
            # Navigation includes its complete forward overhang below.
            targets = dict(targets)
            targets['jekko_boom_pitch_joint'] = .6
            targets['jekko_hook_level_joint'] = .6
            previous_verify = kwargs.get('verify')
            def folded():
                return (max(abs(self.joints[n]) for n in ('jekko_telescope_1_joint',
                    'jekko_telescope_2_joint','jekko_hoist_joint'))<=.008
                    and abs(angle(self.joints['jekko_slew_joint']))<=.01
                    and (previous_verify is None or previous_verify()))
            kwargs['verify'] = folded
            kwargs.setdefault('tolerance',.02)
        if self._final_scene_active and key in (
                'RETRACTING_SUPPORT_FEET','RETRACTING_SUPPORTS','STOWING_FOR_DRIVE'):
            self.check(goal,True)
            tolerance = kwargs.get('tolerance',.006)
            velocities = getattr(self,'joint_velocities',{})
            if (all(n in self.joints and abs(self.joints[n]-v)<=tolerance
                    and abs(velocities.get(n,math.inf))<.02 for n,v in targets.items())
                    and (kwargs.get('verify') is None or kwargs['verify']())):
                self.targets.update(targets)
                self.hold()
                self.feedback(goal,5.,key+' | ALREADY_STOWED | measured=true')
                return
        return super().joint_move(goal,targets,stage,**kwargs)

    def navigate(self, goal, target, p, stage):
        if p.get('final_scene') != 1.:
            return super().navigate(goal,target,p,stage)
        def setup(planner):
            q = self.joints
            if (q['jekko_boom_pitch_joint']<.58
                    or max(abs(q[n]) for n in ('jekko_telescope_1_joint',
                        'jekko_telescope_2_joint','jekko_hoist_joint'))>.012
                    or abs(angle(q['jekko_slew_joint']))>.02):
                raise RuntimeError('JEKKO_DRIVE_STOW_NOT_CONFIRMED')
            # Keep the complete chassis/retracted-leg envelope. Only the old
            # low-boom projection is replaced by the measured raised stow.
            planner.forward_boxes = planner.forward_boxes[:-1]+((-.22,1.55,-.18,.18),)
            planner.boxes = planner.forward_boxes
            planner._turn_radius = max(math.hypot(x,y)
                for x0,x1,y0,y1 in planner.boxes
                for x,y in ((x0,y0),(x0,y1),(x1,y0),(x1,y1)))
            planner.prefer_reverse = False
            planner.prefer_left = False
            planner.reuse_clear_path = True
            planner.shortcut_clear_path = True
            b = self.final_scene['workbench']
            planner.static_boxes += ((*b['center'][:2],0.,
                -b['size'][0]/2,b['size'][0]/2,-b['size'][1]/2,b['size'][1]/2),)
            self._final_nav_fixed = planner.static_boxes
            self._final_nav_planner = planner
        try:
            return super().navigate(goal,target,p,stage,planner_setup=setup)
        finally:
            self._final_nav_planner = None

    def scene(self, p):
        if p.get('final_scene')!=1. or self._final_nav_planner is None:
            return super().scene(p)
        revision,shared = self.obstacles.ground_snapshot()
        boxes = []
        peer = self.entities.get('innok223')
        if (not peer or time.monotonic()-peer[1]>2.
                or not self.carrier_state or time.monotonic()-self.carrier_seen>1.):
            raise RuntimeError('SCENE_FEEDBACK_LOST: measured carrier geometry unavailable')
        x,y,_,yaw = peer[0]
        bed = self.carrier_state['bed']
        # Same complete tractor/trailer rectangles used by Innok223 planning;
        # do not turn them into overlapping inflated circles across the aisle.
        boxes.extend(((x,y,yaw,-.52,.82,-.44,.44),
                      (bed[0],bed[1],bed[3],-.96,.96,-.69,.69)))
        if self.cargo is not None and not self.loaded:
            size = self.final_scene['cargo']['size']
            boxes.append((*self.cargo[:2],self.cargo[3],
                          -size[0]/2,size[0]/2,-size[1]/2,size[1]/2))
        self._final_nav_planner.static_boxes = self._final_nav_fixed+tuple(boxes)
        signature = (revision,tuple(round(v,2) for b in boxes for v in b))
        return signature,tuple((x,y,.45) for x,y in shared)

    @staticmethod
    def preparation_key(goal, p):
        return (goal.request.task_id,goal.request.required_capability_id,
                tuple(p['jekko_work_'+a] for a in ('x','y','z','yaw')),
                tuple(p['delivery_'+a] for a in ('x','y','z','yaw')))

    def preparation_ready(self, goal, p):
        ready = self._final_prepared
        if not ready or ready['key']!=self.preparation_key(goal,p) or self.loaded:
            return False
        return (math.dist(self.pose[:2],ready['pose'][:2])<.04
                and abs(angle(self.pose[3]-ready['pose'][3]))<.03
                and math.dist(self._hook_pose()[:3],ready['hover'])<.05
                and all(abs(self.joints[n]-v)<.012 for n,v in ready['supports'].items()))

    def work_pose_reachable(self, p):
        if p.get('final_scene')!=1.:
            return super().work_pose_reachable(p)
        from cask_gazebo_motion.jekko_geometry import jekko_targets
        pickup,delivery = self.pose_param(p,'pickup'),self.pose_param(p,'delivery')
        safe = max(p['target_height'],pickup[2]+.4,delivery[2]+.4)
        gap = float(p.get('rigging_gap',.6))
        try:
            for point in ((*pickup[:2],pickup[2]+gap),(*pickup[:2],safe+gap),
                          (*delivery[:2],safe+gap),(*delivery[:2],delivery[2]+gap)):
                q = jekko_targets(self.pose,point,delivery[3])
                if 2.12+q['jekko_telescope_1_joint']+q['jekko_telescope_2_joint']>4.38:
                    return False
        except ValueError:
            return False
        return True

    def prepare_drive_to_work(self, goal, p, pickup, delivery, safe_z, gap):
        if p.get('final_scene')==1. and self.preparation_ready(goal,p):
            self.feedback(goal,20.,'REUSING_PREPARED_WORK_SITE | supports_and_hook_ready=true')
            return
        return super().prepare_drive_to_work(goal,p,pickup,delivery,safe_z,gap)

    def prepare_work_site(self, goal, p):
        from cask_gazebo_motion.jekko_geometry import jekko_targets
        if self.loaded:
            raise RuntimeError('LOAD_ALREADY_ATTACHED')
        self._final_prepared = None
        pickup,delivery = self.pose_param(p,'pickup'),self.pose_param(p,'delivery')
        safe = max(p['target_height'],pickup[2]+.4,delivery[2]+.4)
        gap = float(p.get('rigging_gap',.6))
        super().prepare_drive_to_work(goal,p,pickup,delivery,safe,gap)
        # Stabilize first; all these actions now overlap the other robot's
        # transport instead of starting after transport has already finished.
        for point in ((*pickup[:2],pickup[2]+gap),(*pickup[:2],safe+gap),
                      (*delivery[:2],safe+gap),(*delivery[:2],delivery[2]+gap)):
            jekko_targets(self.pose,point,delivery[3])
        supports = {f'jekko_{s}_extension_joint':.22 for s in
                    ('front_left','front_right','rear_left','rear_right')}
        self.joint_move(goal,supports,'DEPLOYING_SUPPORTS')
        feet = {f'jekko_{s}_foot_joint':max(0.,min(.21,self.pose[2]+.173)) for s in
                ('front_left','front_right','rear_left','rear_right')}
        self.joint_move(goal,feet,'LOWERING_SUPPORT_FEET')
        hover = (*pickup[:2],safe+gap)
        # Reuse measured-orientation IK and the existing joint controller. A
        # joint-space empty-hook preset avoids a Cartesian chord through the
        # mast's minimum-radius workspace with an artificially fixed wire.
        # One wire payout must admit BOTH ends of the horizontal transport.
        # The hook is much closer to the steel now, so holding an arbitrary
        # zero payout could otherwise push a low boom below its pitch limit.
        # Reserve pitch margin along the complete horizontal path, instead of
        # choosing a wire length that puts the loaded boom at its lower stop.
        hoist = None
        maximum = 3.-max(safe-pickup[2],safe-delivery[2])-.04
        for margin in (.10,.06):
            for i in range(max(0,int(maximum/.02))+1):
                candidate = .02*i
                try:
                    route = [self._fixed_hoist_targets(
                        (pickup[0]+(delivery[0]-pickup[0])*j/20.,
                         pickup[1]+(delivery[1]-pickup[1])*j/20.,safe+gap),
                        pickup[3]+angle(delivery[3]-pickup[3])*j/20.,candidate)
                        for j in range(21)]
                except RuntimeError:
                    continue
                if all(q['jekko_boom_pitch_joint']>=margin and
                       2.12+q['jekko_telescope_1_joint']+q['jekko_telescope_2_joint']<=4.38
                       for q in route):
                    hoist = candidate
                    break
            if hoist is not None:
                break
        if hoist is None:
            raise RuntimeError('JEKKO_STABLE_TRANSPORT_WIRE_UNREACHABLE')
        targets = self._fixed_hoist_targets(hover,pickup[3],hoist)
        self.joint_move(goal,targets,'PREPOSITIONING_EMPTY_HOOK',tolerance=.004)
        super().move_hook(goal,hover,pickup[3],'PREPOSITIONING_EMPTY_HOOK')
        supports.update(feet)
        self._final_prepared = dict(key=self.preparation_key(goal,p),pose=tuple(self.pose),
                                   hover=hover,supports=supports)
        self.feedback(goal,99.,'WORK_SITE_READY | supports_deployed=true | '
            'empty_hook_above_pickup=true | cargo_attached=false')

    def run_operation(self, goal, operation, p):
        if p.get('final_scene')!=1.:
            return super().run_operation(goal,operation,p)
        self._final_scene_active = True
        self._loading_handoff_released = False
        try:
            self.feedback(goal,0.,'JEKKO_CONTROL_V3 | speed_factor=0.875 | hoist_velocity_control='
                          +str(self._hoist_velocity_enabled).lower())
            if not self.preparation_ready(goal,p):
                self.prepare_work_site(goal,p)
            if operation == 'prepare':
                return
            try:
                return super().run_operation(goal,operation,p)
            finally:
                self._final_prepared = None
        finally:
            self._loading_handoff_released = False
            self._placement_velocity = None
            self._vertical_level_target = None
            self._final_scene_active = False

    def clear_after_delivery(self, goal, clearance):
        p = dict(zip(goal.request.parameter_names,goal.request.parameter_values))
        deck = self.final_scene['trailer']['deck_height']+self.final_scene['cargo']['size'][2]/2
        if self._final_scene_active and abs(p['delivery_z']-deck)<.03:
            self.check(goal,True)
            if self.loaded or self.native_state!='detached':
                raise RuntimeError('LOADING_HANDOFF_REQUIRES_NATIVE_DETACH')
            super().verify_delivery(p)
            # The ordinary hook withdrawal has finished. The truck may leave
            # while the existing boom/leg stow completes, never under the hook.
            cargo_top = 2.*self.cargo[2]-self.cargo_bottom()
            if self._hook_pose()[2]-cargo_top<.12:
                raise RuntimeError('LOADING_HANDOFF_HOOK_NOT_CLEAR')
            self._loading_handoff_released = True
            self.feedback(goal,96.,'CARGO_READY_FOR_TRANSPORT | cargo_released=true'
                          +' | hook_clear=true | boom_stow_running=true')
        return super().clear_after_delivery(goal,clearance)

    def verify_delivery(self, p):
        if self._final_scene_active and self._loading_handoff_released:
            # Original final checks continue after stowing. Once the carrier
            # is moving, verify against its measured bed rather than the bay
            # coordinates which the steel has correctly left behind.
            state = self.carrier_state
            if state and time.monotonic()-self.carrier_seen<1.:
                bed = state['bed']
                point = (*bed[:2],bed[2]+self.final_scene['cargo']['size'][2]/2)
                if (math.dist(self.cargo[:3],point)<=.13
                        and abs(angle(self.cargo[3]-bed[3]))<=.08):
                    return
            raise RuntimeError('LOADING_HANDOFF_CARGO_NOT_ON_MEASURED_TRAILER')
        return super().verify_delivery(p)

    def move_hook(self, goal, destination, yaw, stage):
        p = dict(zip(goal.request.parameter_names, goal.request.parameter_values))
        if p.get('final_scene')==1. and self.loaded and stage=='LOWERING':
            deck = self.final_scene['trailer']['deck_height']+self.final_scene['cargo']['size'][2]/2
            if abs(p['delivery_z']-deck)<.03:
                point = tuple(p['delivery_'+a] for a in ('x','y','z','yaw'))
                self.wait_carrier(goal,point,85.,self.feedback)
            hook = self._hook_pose()
            if math.dist(hook[:2],destination[:2])>.04 or abs(angle(hook[3]-yaw))>.02:
                super().move_hook(goal,(*destination[:2],max(hook[2],destination[2]+.25)),
                                  yaw,'ALIGNING_ABOVE_DELIVERY')
                hook = self._hook_pose()
            if math.dist(hook[:2],destination[:2])>.05:
                raise RuntimeError('JEKKO_DELIVERY_ALIGNMENT_NOT_VERIFIED')
            return self.soft_place(goal,p,hook)
        if p.get('final_scene')==1. and stage=='LIFTING':
            hook = self._hook_pose()
            if abs(angle(hook[3]-yaw))>.01:
                super().move_hook(goal,hook[:3],yaw,'ALIGNING_BEFORE_VERTICAL_LIFT')
                hook = self._hook_pose()
            # Measured steel need not be at the nominal input XY to the last
            # centimetre. Retract the wire vertically above the actual load
            # before correcting XY at transport height.
            super().move_hook(goal,(*hook[:2],destination[2]),yaw,stage)
            result = super().move_hook(goal,destination,yaw,'ALIGNING_AT_TRANSPORT_HEIGHT')
            self.joint_move(goal,{'jekko_hoist_joint':self.targets.get(
                'jekko_hoist_joint',self.joints['jekko_hoist_joint'])},
                stage+' | VERIFYING_VERTICAL_STOP',tolerance=.002)
            return result
        if p.get('final_scene')==1. and stage.startswith('LOWERING_TO_PICKUP'):
            hook = self._hook_pose()
            if math.dist(hook[:2],destination[:2])>.012 or abs(angle(hook[3]-yaw))>.01:
                super().move_hook(goal,(*destination[:2],max(hook[2],destination[2]+.4)),
                                  yaw,'ALIGNING_ABOVE_MEASURED_CARGO')
        if p.get('final_scene') == 1. and stage == 'LOWERING' and abs(p['delivery_z']-(self.final_scene['trailer']['deck_height']+self.final_scene['cargo']['size'][2]/2))<.03:
            point = tuple(p['delivery_'+a] for a in ('x','y','z','yaw'))
            self.wait_carrier(goal,point,85.,self.feedback)
        result = super().move_hook(goal,destination,yaw,stage)
        if p.get('final_scene')==1. and stage.startswith('LOWERING_TO_PICKUP'):
            self.joint_move(goal,{'jekko_hoist_joint':self.targets.get(
                'jekko_hoist_joint',self.joints['jekko_hoist_joint'])},
                stage+' | VERIFYING_VERTICAL_STOP',tolerance=.002)
        return result


def crane_main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = FinalCraneServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.hold_position()
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def jekko_main(args=None):
    spin(FinalJekkoServer,args)
