import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ros_interfaces.action import ExecuteSkill
from cask_gazebo_motion.jekko_motion import JekkoGazeboMotionBase, JekkoGazeboMotion

class JekkoSkillServerBase(JekkoGazeboMotionBase, Node):
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
        for name in ('available',):
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
        if any(not bool(self.get_parameter(name).value) for name in ('available',)):
            raise RuntimeError('ROBOT_UNAVAILABLE: robot is disabled')

    def _generic_failure_limit(self, name, goal):
        import math
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value > 100.0:
            raise RuntimeError('INVALID_FAILURE_PARAMETER: ' + name + ' must be negative (disabled) or within [0,100]')
        if value >= 0.0:
            return value
        task = str(goal.request.task_id).replace('_', '').lower()
        matching = task + '_' + name
        if matching in self._generic_failure_aliases:
            value = float(self.get_parameter(matching).value)
            if not math.isfinite(value) or value > 100.0:
                raise RuntimeError('INVALID_FAILURE_PARAMETER: ' + matching)
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
        loaded = bool(getattr(self,'loaded',False))
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
            updates = [Parameter(name, value=False) for name in ('available',)]
            results = self.set_parameters(updates)
            if not all(result.successful for result in results):
                self.get_logger().error('Failure injected; an availability parameter update was rejected')
            raise RuntimeError(
                'ROBOT_UNAVAILABLE: injected failure; task=' + str(goal.request.task_id)
                + '; skill_progress=' + format(self._generic_failure_progress, '.2f')
                + '; threshold=' + format(threshold, '.2f') + '; parameter=' + name
                + '; carrying_phase=' + str(loaded).lower())

    def __init__(self, robot, node_name, endpoints, commands):
        super().__init__(node_name)
        self._gazebo_jekkoskillserverbase_setup_0(robot)
        self.declare_parameter('available', True)
        self.declare_parameter('task005_fail_before_pickup_at_progress', -1.0)
        self.declare_parameter('task005_fail_loaded_at_progress', -1.0)
        self._gazebo_jekkoskillserverbase_setup_1()
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self._gazebo_jekkoskillserverbase_setup_2(commands, robot)
        self.servers = [ActionServer(self, ExecuteSkill, endpoint, execute_callback=lambda goal, op=operation: self.execute(goal, op), goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group) for (operation, endpoint) in endpoints.items()]
        self._gazebo_jekkoskillserverbase_setup_3()
        self._generic_failure_initialize()

    def accept(self, request):
        # CASK_GENERATED_IOC_V1
        from cask_gazebo_motion.generated_cargo import accepts_numeric
        return GoalResponse.ACCEPT if accepts_numeric(self, request) else GoalResponse.REJECT

    def check(self, goal, cargo=False):
        if goal.is_cancel_requested:
            raise RuntimeError('CANCELED')
        if not self.get_parameter('available').value:
            raise RuntimeError('ROBOT_UNAVAILABLE')
        self._gazebo_jekkoskillserverbase_check_feedback(goal, cargo)

    def feedback(self, goal, progress, status):
        # CASK_JEKKO_PREPARATION_FAILURE_V2
        stage = status.split(' | ', 1)[0]
        value = getattr(self, '_jekko_skill_progress', 0.)
        context = getattr(self, '_jekko_progress_context', None)
        if context is not None and context[0] == stage:
            _, kind, start, target, first, last = context
            if kind == 'drive':
                initial = math.dist(start[:2], target[:2])
                remaining = math.dist(self.pose[:2], target[:2])
            elif kind == 'hook':
                hook = self._hook_pose()
                initial = math.dist(start[:3], target[:3]) + .3 * abs(math.atan2(math.sin(target[3]-start[3]), math.cos(target[3]-start[3])))
                remaining = math.dist(hook[:3], target[:3]) + .3 * abs(math.atan2(math.sin(target[3]-hook[3]), math.cos(target[3]-hook[3])))
            else:
                initial = max((abs(start[n]-v) for n,v in target.items()), default=0.)
                remaining = max((abs(self.joints.get(n,start[n])-v) for n,v in target.items()), default=0.)
            fraction = 1. if initial <= 1e-9 else max(0., min(1., 1.-remaining/initial))
            value = max(value, first+(last-first)*fraction)
        elif stage == 'WORK_AREA_REACHED':
            value = max(value, 20.)
        elif stage == 'RELEASING':
            value = max(value, 90.)
        elif stage == 'SUCCESS':
            value = 100.
        self._jekko_skill_progress = value
        msg = ExecuteSkill.Feedback()
        msg.progress = float(value)
        msg.status = status + f' | jekko_skill_progress={value:.2f}'
        goal.publish_feedback(msg)
        self._generic_failure_check(goal, msg.progress, msg.status)

    def execute(self, goal, operation):
        result = ExecuteSkill.Result()
        if not self.lock.acquire(False):
            goal.abort()
            result.message = 'ROBOT_BUSY'
            return result
        self.busy = True
        self._jekko_skill_progress = 0.
        self._jekko_progress_context = None
        try:
            self._generic_failure_begin(goal)
            p = dict(zip(goal.request.parameter_names, goal.request.parameter_values))
            if len(p) != len(goal.request.parameter_names) or not all((math.isfinite(v) for v in p.values())):
                raise RuntimeError('INVALID_PARAMETERS')
            from cask_gazebo_motion.generated_cargo import select_jekko
            select_jekko(self, goal)
            self._gazebo_jekkoskillserverbase_wait_scene(goal)
            self.check(goal, True)
            self.feedback(goal, 0.0, 'RUNNING')
            self.run_operation(goal, operation, p)
            self.feedback(goal, 100.0, 'SUCCESS | measured_motion_verified=true | process_simulated=true')
            goal.succeed()
            result.success = True
            result.message = self.robot + ' completed ' + operation + '; Gazebo process simulation'
        except Exception as exc:
            self._gazebo_jekkoskillserverbase_freeze_targets()
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
            result.success = False
            result.message = str(exc) + f'; carrying_phase={str(self.loaded).lower()}'
            self.get_logger().error(result.message)
        finally:
            self._gazebo_jekkoskillserverbase_stop()
            self.busy = False
            self.lock.release()
        return result

def spin(server_type, args=None):
    rclpy.init(args=args)
    node = server_type()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.drive()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

class JekkoServer(JekkoGazeboMotion, JekkoSkillServerBase):
    def __init__(self):
        (commands,) = self._gazebo_jekkoserver_setup_0()
        super().__init__('jekko', 'jekko_lifting_server', {'lifting': '/jekko_01/lifting'}, commands)
        self._gazebo_jekkoserver_setup_1()

    def destroy_node(self):
        self._gazebo_jekkoserver_close_physics()
        return super().destroy_node()

def main(args=None):
    spin(JekkoServer, args)

# CASK_ALL_ROBOT_CONTEXT_V1
