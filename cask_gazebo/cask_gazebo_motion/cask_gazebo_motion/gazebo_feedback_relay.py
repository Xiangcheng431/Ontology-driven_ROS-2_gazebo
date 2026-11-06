"""Bound ROS feedback fan-out without changing Gazebo's physics time step.

Raw messages remain Gazebo measurements. Copy them to their original public
topics at 200 Hz in simulation time. No interpolation or synthetic timestamps.
The relay uses wall time internally and never subscribes to its own /clock.
"""

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


ROBOTS = ('crane', 'jekko', 'innok223')
RAW = '/cask_gazebo_motion/raw'


class GazeboFeedbackRelay(Node):
    def __init__(self):
        super().__init__('gazebo_feedback_relay')
        try:
            hz = float(os.environ.get('CASK_GAZEBO_FEEDBACK_HZ', '200'))
        except ValueError as error:
            raise ValueError('CASK_GAZEBO_FEEDBACK_HZ must be between 200 and 1000') from error
        if not math.isfinite(hz) or not 200 <= hz <= 1000:
            raise ValueError('CASK_GAZEBO_FEEDBACK_HZ must be between 200 and 1000')
        self.period_ns = round(1_000_000_000 / hz)
        incoming = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        joint_output = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE)
        clock_output = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        self.last_stamps = {}
        self.robot_publishers = {
            robot: self.create_publisher(JointState, f'/{robot}/joint_states', joint_output)
            for robot in ROBOTS
        }
        self.clock_publisher = self.create_publisher(Clock, '/clock', clock_output)
        self.robot_subscriptions = [
            self.create_subscription(JointState, f'{RAW}/{robot}/joint_states',
                                     lambda message, robot=robot: self.on_joints(robot, message), incoming)
            for robot in ROBOTS
        ]
        self.clock_subscription = self.create_subscription(Clock, f'{RAW}/clock', self.on_clock, incoming)
        self.get_logger().info(f'ROS clock and three robot JointState topics: {hz:g} Hz in simulation time; physics unchanged')

    def ready(self, key, stamp):
        now = stamp.sec * 1_000_000_000 + stamp.nanosec
        previous = self.last_stamps.get(key)
        # Reset clocks start a fresh window. Do not synthesize time or replay
        # queued old messages. KEEP_LAST(1) keeps feedback recent under load.
        if previous is None or now < previous or now-previous >= self.period_ns:
            self.last_stamps[key] = now
            return True
        return False

    def on_joints(self, robot, message):
        if self.ready(robot, message.header.stamp):
            self.robot_publishers[robot].publish(message)

    def on_clock(self, message):
        if self.ready('clock', message.clock):
            self.clock_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboFeedbackRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
