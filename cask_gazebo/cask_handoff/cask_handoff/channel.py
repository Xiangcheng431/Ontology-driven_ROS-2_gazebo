"""Session-scoped, fresh ROS handoff events; no robot selection in this module."""

import json
import math
import threading
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from .timing import ReceiverProgress


class HandoffError(RuntimeError):
    pass


class Channel:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.messages = {}
        self.sequence = 0
        self.publisher = node.create_publisher(String, "/cask/handoff/events", 50)
        self.subscription = node.create_subscription(
            String,
            "/cask/handoff/events",
            self.receive,
            50,
            callback_group=ReentrantCallbackGroup(),
        )

    def receive(self, message):
        try:
            item = json.loads(message.data)
            if (
                not isinstance(item, dict)
                or not all(
                    isinstance(item.get(k), str) and 0 < len(item[k]) < 256
                    for k in ("session", "role", "actor", "stage")
                )
                or item["role"] not in ("coordinator", "provider", "receiver")
                or not isinstance(item.get("sequence"), int)
                or not isinstance(item.get("data"), dict)
            ):
                return
            key = (item["session"], item["role"], item["actor"])
            with self.lock:
                old = self.messages.get(key)
                if old and item["sequence"] <= old[1]["sequence"]:
                    return
                self.messages[key] = (time.monotonic(), item)
                if len(self.messages) > 256:
                    oldest = min(self.messages, key=lambda k: self.messages[k][0])
                    self.messages.pop(oldest)
        except (ValueError, TypeError):
            return

    def send(self, session, role, actor, stage, **data):
        with self.lock:
            self.sequence += 1
            sequence = self.sequence
        message = String(
            data=json.dumps(
                dict(
                    session=session,
                    role=role,
                    actor=actor,
                    stage=stage,
                    data=data,
                    sequence=sequence,
                ),
                allow_nan=False,
            )
        )
        self.publisher.publish(message)

    def get(self, session, role, actor, max_age=2.0):
        with self.lock:
            item = self.messages.get((session, role, actor))
        return item[1] if item and time.monotonic() - item[0] <= max_age else None


class Participant:
    def __init__(self, channel, request, role, health):
        self.channel = channel
        self.session = request.execution_id
        self.actor = request.skill_id
        self.role = role
        self.health = health
        self.began = time.monotonic()
        self.transfer_started = False

    def check(self):
        self.health()
        binding = self.channel.get(self.session, "coordinator", "task_manager")
        if binding is None:
            raise HandoffError("HANDOFF_COORDINATOR_LOST: hold cargo and stop receiver")
        if binding["stage"] == "STOP":
            raise HandoffError("HANDOFF_STOPPED")
        if binding["data"].get(self.role) != self.actor:
            raise HandoffError("HANDOFF_BINDING_CHANGED")
        return binding["data"]

    def emit(self, stage, **data):
        self.channel.send(self.session, self.role, self.actor, stage, **data)

    def peer(self):
        binding = self.check()
        role = "receiver" if self.role == "provider" else "provider"
        return self.channel.get(self.session, role, binding.get(role, ""))

    def wait(
        self,
        stages,
        stage,
        timeout=180.0,
        tick=None,
        allow_pretransfer_retry=False,
        progress_timeout=None,
        **data
    ):
        began = time.monotonic()
        progress = ReceiverProgress(progress_timeout) if progress_timeout is not None else None
        while time.monotonic() - began < timeout:
            peer = self.peer()
            self.emit(stage, **data)
            if peer and peer["stage"] in stages:
                return peer
            if peer and peer["stage"] == "FAILED":
                # While holding, let the coordinator try another compatible
                # receiver only if the failed receiver never entered the dock.
                safe_retry = (
                    allow_pretransfer_retry
                    and peer["data"].get("transfer_started") is False
                )
                if not safe_retry:
                    raise HandoffError("HANDOFF_PEER_FAILED")
            if progress is not None and not progress.observe(peer, time.monotonic()):
                raise HandoffError("HANDOFF_RECEIVER_STALLED: no measured approach/lift progress")
            if tick:
                tick()
            time.sleep(0.1)
        raise HandoffError("HANDOFF_TIMEOUT: " + stage)
