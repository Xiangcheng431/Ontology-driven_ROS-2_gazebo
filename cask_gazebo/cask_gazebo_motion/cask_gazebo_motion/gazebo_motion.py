"""Feedback-driven motion helpers for Gazebo; no simulated joint-state writers."""
import math
import threading
import time

from rclpy.action import GoalResponse


class MotionError(RuntimeError):
    pass


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class MotionSupport:
    def setup_motion(self):
        self.motion_lock = threading.Lock()
        self.declare_parameter('feedback_timeout', 2.0)
        self.declare_parameter('motion_timeout', 180.0)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout').value)
        self.motion_timeout = float(self.get_parameter('motion_timeout').value)
        self.last_joint_feedback = None

    @staticmethod
    def finite(*values):
        if not all(math.isfinite(v) for v in values):
            raise MotionError('INVALID_PARAMETER: non-finite motion target')

    @staticmethod
    def accepted_goal(_request):
        return GoalResponse.ACCEPT

    def check_feedback(self, goal, need_odom=False):
        if goal.is_cancel_requested:
            raise MotionError('CANCELED')
        now = time.monotonic()
        timestamps = [self.last_joint_feedback]
        if need_odom:
            timestamps.append(self.last_odom_feedback)
        if any(t is None or now - t > self.feedback_timeout for t in timestamps):
            raise MotionError('FEEDBACK_LOST: check Gazebo playback and ROS bridge')

    def wait_ready(self, goal, need_odom=False):
        until = time.monotonic() + 10.0
        while time.monotonic() < until:
            try:
                self.check_feedback(goal, need_odom)
                return
            except MotionError as exc:
                if str(exc) == 'CANCELED':
                    raise
                time.sleep(0.1)
        raise MotionError('FEEDBACK_LOST: no fresh Gazebo feedback within 10 seconds')

    def wait_positions(self, goal, targets, command, feedback):
        """Complete only after fresh measurements reach and hold the target."""
        self.finite(*targets.values())
        start = time.monotonic()
        stable_since = None
        initial = {name: abs(getattr(self, name) - target)
                   for name, target in targets.items()}
        while True:
            self.check_feedback(goal)
            if time.monotonic() - start > self.motion_timeout:
                raise MotionError('MOTION_TIMEOUT: joint target not reached')
            command()
            errors = {name: abs(angle(getattr(self, name) - target))
                      if name == 'tower_position' else abs(getattr(self, name) - target)
                      for name, target in targets.items()}
            fraction = min(1.0, max(0.0, 1.0 - max(
                errors[name] / max(initial[name], 0.03) for name in targets)))
            feedback(fraction)
            if all(error <= 0.025 for error in errors.values()):
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.4:
                    feedback(1.0)
                    return
            else:
                stable_since = None
            time.sleep(0.1)
