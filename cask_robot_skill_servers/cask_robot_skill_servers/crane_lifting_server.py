import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cask_gazebo_motion.gazebo_motion import MotionError
from ros_interfaces.action import ExecuteSkill
from cask_gazebo_motion.crane_motion import CraneGazeboMotion

class CraneLiftingServer(CraneGazeboMotion, Node):
    'Execute the Crane lifting skill and publish its observable state.\n\n    In legacy visual mode, ``target_height`` is the world-frame height of\n    ``crane_hook_tip_link``. In the current URDF, ``crane_hook_joint`` moves\n    along -Z, so a larger joint value lowers the physical hook point.\n    Physical mode treats target_height as the cargo target, with a separate\n    temporary transport height computed for pickup separation.\n    '

    def _generic_failure_initialize(self):
        # CASK_GENERIC_ROBOT_FAILURE_V1
        for name, default in {
                'fail_at_progress': -1.0,
                'fail_before_pickup_at_progress': -1.0,
                'fail_loaded_at_progress': -1.0,
                'failure_task_id': '',
                'failure_skill_id': ''}.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        for name in ('lifting_available',):
            if not self.has_parameter(name):
                self.declare_parameter(name, True)
        import re
        # CASK_GENERIC_FAILURE_HUMBLE_STARTUP_V2
        self._generic_failure_aliases = tuple(
            name for name in self.get_parameters_by_prefix('')
            if re.fullmatch(r'task\d+_fail_(?:before_pickup|loaded)_at_progress', name))
        self._generic_failure_fired = False
        self._generic_failure_progress = 0.0
        self._generic_failure_had_load = False

    def _generic_failure_begin(self, goal):
        self._generic_failure_fired = False
        self._generic_failure_progress = 0.0
        self._generic_failure_had_load = False
        if any(not bool(self.get_parameter(name).value) for name in ('lifting_available',)):
            raise MotionError('ROBOT_UNAVAILABLE: robot is disabled')

    def _generic_failure_limit(self, name, goal):
        import math
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value > 100.0:
            raise MotionError('INVALID_FAILURE_PARAMETER: ' + name + ' must be negative (disabled) or within [0,100]')
        if value >= 0.0:
            return value
        task = str(goal.request.task_id).replace('_', '').lower()
        matching = task + '_' + name
        if matching in self._generic_failure_aliases:
            value = float(self.get_parameter(matching).value)
            if not math.isfinite(value) or value > 100.0:
                raise MotionError('INVALID_FAILURE_PARAMETER: ' + matching)
            return value
        return -1.0

    def _generic_failure_check(self, goal, progress, status):
        import math
        if self._generic_failure_fired:
            return
        stage = str(status).split(' | ', 1)[0].upper()
        if stage.startswith(('SUCCESS', 'FAILURE', 'FAILED', 'CANCELED', 'CANCELLED', 'ABORTED')):
            return
        task_filter = str(self.get_parameter('failure_task_id').value)
        skill_filter = str(self.get_parameter('failure_skill_id').value)
        if task_filter and task_filter != str(goal.request.task_id):
            return
        if skill_filter and skill_filter != str(getattr(goal.request, 'skill_id', '')):
            return
        value = float(progress)
        if not math.isfinite(value):
            return
        self._generic_failure_progress = max(self._generic_failure_progress, max(0.0, min(100.0, value)))
        loaded = bool(getattr(self,'object_attached',False))
        self._generic_failure_had_load = self._generic_failure_had_load or loaded
        checks = [('fail_at_progress', True)]
        if True:
            checks += [('fail_loaded_at_progress', loaded),
                       ('fail_before_pickup_at_progress', not loaded and not self._generic_failure_had_load)]
        for name, applicable in checks:
            if not applicable:
                continue
            threshold = self._generic_failure_limit(name, goal)
            if threshold < 0.0 or self._generic_failure_progress < threshold:
                continue
            # Latch before publishing failure or updating availability. Failure
            # feedback from existing exception handlers must never re-inject.
            self._generic_failure_fired = True
            from rclpy.parameter import Parameter
            updates = [Parameter(name, value=False) for name in ('lifting_available',)]
            results = self.set_parameters(updates)
            if not all(result.successful for result in results):
                self.get_logger().error('Failure injected; an availability parameter update was rejected')
            raise MotionError(
                'ROBOT_UNAVAILABLE: injected failure; task=' + str(goal.request.task_id)
                + '; skill_progress=' + format(self._generic_failure_progress, '.2f')
                + '; threshold=' + format(threshold, '.2f') + '; parameter=' + name
                + '; carrying_phase=' + str(loaded).lower())

    def __init__(self):
        super().__init__('crane_lifting_server')
        self._gazebo_craneliftingserver_setup_0()
        self.declare_parameter('lifting_available', True)
        self._gazebo_craneliftingserver_setup_1()
        self.action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, ExecuteSkill, '/crane_01/lifting', self.execute_callback, cancel_callback=self.cancel_callback, callback_group=self.action_callback_group)
        self._gazebo_craneliftingserver_setup_2()
        self._generic_failure_initialize()

    @staticmethod
    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def parameter_map(request):
        return {
            name.replace('_', '').lower(): float(value)
            for name, value in zip(
                request.parameter_names,
                request.parameter_values,
            )
        }

    @staticmethod
    def required_parameter(parameters, name):
        normalized_name = name.replace('_', '').lower()
        if normalized_name not in parameters:
            raise ValueError(
                f'Action Goal does not contain required parameter {name}'
            )
        return parameters[normalized_name]

    def execute_callback(self, goal_handle):
        if not self.motion_lock.acquire(blocking=False):
            return self.abort_goal(goal_handle, 'ROBOT_BUSY: another motion is executing')
        try:
            self._generic_failure_begin(goal_handle)
            request = goal_handle.request
            if self.physical_cargo_mode:
                from cask_gazebo_motion.generated_cargo import select_for_goal
                select_for_goal(self, goal_handle, 'crane')
            self.wait_ready(goal_handle)
            if not bool(self.get_parameter('lifting_available').value):
                raise MotionError('ROBOT_UNAVAILABLE: crane lifting disabled')
            if self.physical_cargo_mode:
                return self.execute_physical_task001(goal_handle)
            parameters = self.parameter_map(request)
            self._gazebo_craneliftingserver_visual_lifting(goal_handle, parameters)
            goal_handle.succeed()
            result = ExecuteSkill.Result()
            result.success = True
            result.message = f'Crane_01 completed skill {request.skill_id} for task {request.task_id}; tower_yaw={self.tower_position:.4f}; hook_world_z={self.hook_zero_world_z - self.hook_position:.3f}; object_attached=true'
            return result
        except MotionError as error:
            if str(error) == 'CANCELED':
                goal_handle.canceled()
                return self.canceled_result(goal_handle.request)
            return self.abort_goal(goal_handle, str(error))
        except (TypeError, ValueError, RuntimeError, OSError) as error:
            return self.abort_goal(goal_handle, str(error))
        finally:
            self.hold_position()
            self.motion_lock.release()

    def abort_goal(self, goal_handle, reason):
        if self.object_attached and 'carrying_phase=true' not in reason:
            reason += '; carrying_phase=true'
        # Never select another provider while a suspended handoff load remains
        # attached, including failures during pickup/rotation before HOLD_READY.
        if (self.parameter_map(goal_handle.request).get('handoffmode') == 1.
                and self.object_attached and 'carrying_phase=true' not in reason):
            reason += '; carrying_phase=true'
        self.get_logger().error(f'Lifting goal aborted: {reason}')
        goal_handle.abort()
        result = ExecuteSkill.Result()
        result.success = False
        result.message = reason
        return result

    @staticmethod
    def canceled_result(request):
        result = ExecuteSkill.Result()
        result.success = False
        result.message = (
            f'Crane lifting canceled for task {request.task_id}'
        )
        return result

    def destroy_node(self):
        self._gazebo_craneliftingserver_close_physics()
        self.action_server.destroy()
        return super().destroy_node()

    def _complete_crane_delivery(self, goal):
        goal.succeed()
        result = ExecuteSkill.Result()
        result.success = True
        result.message = f'Crane_01 completed {goal.request.task_id}; native joint released; measured cargo delivery verified'
        return result

    def phase_feedback(self, goal, progress, status):
        super().phase_feedback(goal, progress, status)
        self._generic_failure_check(goal, progress, status)

def main(args=None):
    rclpy.init(args=args)
    node = CraneLiftingServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

# CASK_ALL_ROBOT_CONTEXT_V1
