"""Avoid resending identical commands to Gazebo's latched joint controllers.

Both Fortress JointPositionController and JointController cache the latest
scalar target and apply it each physics step. Never coalesce different values,
including arbitrarily small corrections. Do not wrap cmd_vel publishers.
"""

import os
import threading
import time


class HeldJointCommandPublisher:
    def __init__(self, publisher):
        self._publisher = publisher
        self._lock = threading.Lock()
        self._last_value = None
        self._last_sent = float('-inf')
        self._last_connections = None

    def publish(self, message):
        with self._lock:
            now = time.monotonic()
            connections = self._publisher.get_subscription_count()
            value = message.data
            if (self._last_connections is None or value != self._last_value
                    or connections != self._last_connections
                    or now - self._last_sent >= 0.2):
                # Keep the original message, topic, QoS, value and ordering.
                # Update cache only after a successful publish.
                self._publisher.publish(message)
                self._last_value = value
                self._last_sent = now
                self._last_connections = connections

    def __getattr__(self, name):
        return getattr(self._publisher, name)


def create_joint_command_publisher(node, message_type, topic, qos_profile):
    publisher = node.create_publisher(message_type, topic, qos_profile)
    enabled = os.environ.get('CASK_GAZEBO_DEDUP_JOINT_COMMANDS', '1').strip().lower()
    if enabled in ('0', 'false', 'off'):
        return publisher
    return HeldJointCommandPublisher(publisher)
