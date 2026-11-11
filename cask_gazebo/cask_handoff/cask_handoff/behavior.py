"""A cooperative BT group. Candidate selection remains in the existing fallback.

The provider remains RUNNING until physical release; receiver execution begins
only after HOLD_READY. Both goals are owned and joined by this group.
"""

import time
import uuid
from rclpy.task import Future
from .channel import Channel
from .timing import GROUP_TIMEOUT


async def pause(node, seconds=0.05):
    future = Future()

    def wake():
        if not future.done():
            future.set_result(None)

    timer = node.create_timer(seconds, wake, callback_group=node.callback_group)
    try:
        await future
    finally:
        node.destroy_timer(timer)


class HandoffPair:
    def __init__(self, provider, receiver, status):
        self.provider = provider
        self.receiver = receiver
        self.states = status
        self.status = status.IDLE
        self.span = 2

    @property
    def children(self):
        return self.provider.children + self.receiver.children

    async def tick(
        self, manager, task_goal_handle, execution_id, task_id, index, total
    ):
        session = execution_id + ":" + uuid.uuid4().hex
        channel = manager.handoff_channel
        binding = {"provider": "", "receiver": ""}
        stage = "BIND"

        def pulse():
            channel.send(session, "coordinator", "task_manager", stage, **binding)

        def bind_provider(candidate):
            binding["provider"] = candidate["skill_name"]
            pulse()

        def bind_receiver(candidate):
            peer = channel.get(session, "provider", binding["provider"])
            if not peer or peer["stage"] != "HOLD_READY":
                raise RuntimeError(
                    "Cannot replace receiver after handoff descent has begun"
                )
            binding["receiver"] = candidate["skill_name"]
            pulse()

        self.provider.before_candidate = bind_provider
        self.receiver.before_candidate = bind_receiver
        self.status = self.states.RUNNING

        def run(child, offset):
            return manager.executor.create_task(
                child.tick(
                    manager, task_goal_handle, session, task_id, index + offset, total
                )
            )

        provider = run(self.provider, 0)
        receiver = None
        began = time.monotonic()
        try:
            while True:
                pulse()
                if task_goal_handle.is_cancel_requested:
                    raise RuntimeError("Handoff canceled")
                if time.monotonic() - began > GROUP_TIMEOUT:
                    raise RuntimeError("Handoff group timed out")
                peer = channel.get(session, "provider", binding["provider"])
                if receiver is None and peer and peer["stage"] == "HOLD_READY":
                    receiver = run(self.receiver, 1)
                if provider.done():
                    if provider.result() != self.states.SUCCESS:
                        raise RuntimeError("Handoff provider failed")
                    if receiver is None:
                        raise RuntimeError(
                            "Provider finished without handoff readiness"
                        )
                if receiver is not None and receiver.done():
                    if receiver.result() != self.states.SUCCESS:
                        raise RuntimeError("Handoff receiver failed")
                    if provider.done():
                        self.status = self.states.SUCCESS
                        return self.status
                await pause(manager)
        except Exception as error:
            manager.get_logger().error(str(error))
            self.status = self.states.FAILURE
            stage = "STOP"
            pulse()
            manager.cancel_active_skill()
            # Never leave a provider coroutine running after the task returns.
            pending = [f for f in (provider, receiver) if f is not None]
            until = time.monotonic() + 15
            while any(not f.done() for f in pending) and time.monotonic() < until:
                pulse()
                await pause(manager)
            if any(not f.done() for f in pending):
                manager.handoff_drain_pending = pending
                manager.get_logger().error(
                    "HANDOFF_DRAIN_PENDING: reject new tasks until skills stop"
                )
            return self.status
        finally:
            self.provider.before_candidate = self.receiver.before_candidate = None
