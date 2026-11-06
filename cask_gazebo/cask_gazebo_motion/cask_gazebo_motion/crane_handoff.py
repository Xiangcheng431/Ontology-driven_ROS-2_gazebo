"""Physical suspended-load provider adapter; matching lives in Task Manager."""

import math
from .motion_speed import crane_speed, crane_period
import time
from types import SimpleNamespace
from cask_handoff.channel import Participant, HandoffError
from cask_handoff.timing import RECEIVER_APPROACH_TIMEOUT, RECEIVER_PROGRESS_TIMEOUT
from .gazebo_motion import MotionError, angle
from ros_interfaces.action import ExecuteSkill


def descent_command(command_z, cargo_z, receive_z, support_z, dt, truck=False):
    """Keep the existing slow landing, use the native hook cap in free air."""
    slow = crane_speed(.06)
    floor = max(receive_z-.008, support_z-.008)
    if truck and cargo_z-support_z > .12:
        # Stop the faster command ramp above the bed. Switch on measured
        # height, so a lagging native joint cannot be rushed into contact.
        floor = max(floor, support_z+.08)
        rate = crane_speed(.33)
    else:
        rate = slow
    return max(floor, command_z-rate*dt), rate


def finish_handoff(node, goal, parameters, delivery):
    """Called after the normal pickup and rotation, while native joint is attached."""
    part = Participant(
        node.handoff_channel,
        goal.request,
        "provider",
        lambda: node.check_feedback(goal),
    )
    try:
        return _finish(node, goal, parameters, delivery, part)
    except (HandoffError, MotionError, KeyError, TypeError, ValueError, RuntimeError, OSError) as error:
        part.emit("FAILED", attached=node.object_attached)
        node.hold_position()
        raise MotionError(str(error) + "; carrying_phase=true") from error


def _finish(node, goal, p, delivery, part):
    middle = p["handoffmiddlez"]
    receive = p["handoffreceivez"]
    hold = node.crane_targets(
        delivery[0], delivery[1], middle + node.attached_gap, delivery[3]
    )
    node.physical_move(goal, hold, 85, 88, "MOVING_TO_HANDOFF_HOLD")
    node.hold_position()

    last_hold_feedback = [-math.inf]

    def holding():
        cargo = node.measured_cargo(goal)
        if (
            math.dist(cargo[:2], delivery[:2]) > 0.12
            or abs(cargo[2] - middle) > 0.08
            or node.native_attachment_state != "attached"
        ):
            raise MotionError("HANDOFF_HOLD_LOST")
        if time.monotonic()-last_hold_feedback[0] >= .5:
            last_hold_feedback[0] = time.monotonic()
            node.phase_feedback(goal, 88, "HOLD_READY | native_joint=attached")

    holding()
    ready = part.wait(
        {"RECEIVER_READY"}, "HOLD_READY", timeout=RECEIVER_APPROACH_TIMEOUT,
        tick=holding, allow_pretransfer_retry=True, progress_timeout=RECEIVER_PROGRESS_TIMEOUT
    )
    # A fixed Truck bed and movable Forklift forks have different support
    # heights. The selected receiver advertises its ontology-derived target.
    # Older Forklift events retain the original After-state height.
    receive = float(ready['data'].get('receive_z', receive))
    if not math.isfinite(receive) or receive < 0 or receive + .25 > middle:
        raise MotionError('HANDOFF_RECEIVER_HEIGHT_INVALID')
    receiver_model = ready['data']['model']
    receiver_support = ready['data']['support_link']
    part.transfer_started = True
    began = time.monotonic()
    stable = None
    command_z = node.measured_cargo(goal)[2]
    descent_timeout = max(30.0, abs(middle - receive) / crane_speed(0.06) + 15.0)
    previous = began
    while time.monotonic() - began < descent_timeout:
        now = time.monotonic()
        dt = max(0., min(crane_period(.2), now-previous))
        previous = now
        peer = part.peer()
        if not peer or peer["stage"] != "RECEIVER_READY":
            raise MotionError("RECEIVER_NOT_READY: hold load")
        data = peer["data"]
        if (data['model'] != receiver_model or data['support_link'] != receiver_support
                or abs(float(data.get('receive_z', receive))-receive) > .001):
            raise MotionError('HANDOFF_RECEIVER_PROFILE_CHANGED')
        if (
            math.hypot(data["cargo_x"] - delivery[0], data["cargo_y"] - delivery[1])
            > 0.06
            or abs(angle(data["yaw"] - delivery[3])) > 0.04
        ):
            raise MotionError("RECEIVER_ALIGNMENT_LOST: hold load")
        cargo = node.measured_cargo(goal)
        if node.native_attachment_state != "attached":
            raise MotionError("HANDOFF_ATTACHMENT_LOST")
        if math.dist(cargo[:2], delivery[:2]) > 0.12:
            raise MotionError("HANDOFF_CARGO_DRIFT")
        # Stop descending while the receiver corrects a load-induced height
        # offset. Clear its forks instead of chasing the sinking support down.
        if not data.get("height_stable", True) or abs(data["support_z"] - receive) > 0.03:
            stable = None
            command_z = receive + 0.03
            node.command_positions({
                "hook_position": node.world_height_to_joint_position(
                    command_z + node.attached_gap)
            })
            part.emit("LOWERING", cargo_z=cargo[2])
            node.phase_feedback(goal, 90, "HOLDING_FOR_RECEIVER_HEIGHT | native_joint=attached")
            time.sleep(crane_period(0.1))
            continue
        contact = node.handoff_contacts.supported(data["model"], data["support_link"])
        supported = contact and abs(cargo[2] - data["support_z"]) < 0.04
        part.emit("LOWERING", cargo_z=cargo[2])
        node.phase_feedback(
            goal,
            90,
            f"LOWERING_TO_RECEIVER | cargo_z={cargo[2]:.3f} | contact={contact}"
            f" | contact_status={node.handoff_contacts.status(data['model'], data['support_link'])}"
            f" | support_error={cargo[2]-data['support_z']:.3f}"
            f" | elapsed={now-began:.1f}s | stable_for={0. if stable is None else now-stable:.2f}s",
        )
        if supported:
            node.hold_position()
            if stable is None:
                stable = time.monotonic()
            elif time.monotonic() - stable > 0.5:
                break
        else:
            stable = None
            command_z, _ = descent_command(command_z, cargo[2], receive,
                                           data['support_z'], dt,
                                           truck=receiver_model == 'truck')
            node.command_positions(
                {
                    "hook_position": node.world_height_to_joint_position(
                        command_z + node.attached_gap
                    )
                }
            )
        time.sleep(crane_period(0.1))
    else:
        raise MotionError('HANDOFF_SUPPORT_TIMEOUT: load remains attached'
                          f'; receiver={receiver_model}; cargo_z={cargo[2]:.3f}'
                          f'; support_z={data["support_z"]:.3f}'
                          f'; contact_status={node.handoff_contacts.status(receiver_model,receiver_support)}')
    # Re-check current authorization and contact immediately before release.
    peer = part.peer()
    if (
        not peer
        or peer["stage"] != "RECEIVER_READY"
        or not peer["data"].get("height_stable", True)
        or abs(peer["data"]["support_z"] - receive) > 0.03
        or peer['data']['model'] != receiver_model
        or peer['data']['support_link'] != receiver_support
        or abs(float(peer['data'].get('receive_z', receive))-receive) > .001
        or not node.handoff_contacts.supported(
            peer["data"]["model"], peer["data"]["support_link"]
        )
    ):
        raise MotionError("HANDOFF_RELEASE_NOT_AUTHORIZED")
    previous = node.native_state_revision
    observed_revision, observed_state = node.native_attachment.state_snapshot()
    last_release_feedback = [-math.inf]

    def release_tick():
        peer = part.peer()  # Fresh coordinator lease and receiver authorization.
        if (not peer or peer['stage'] != 'RECEIVER_READY'
                or peer['data']['model'] != receiver_model
                or peer['data']['support_link'] != receiver_support
                or not peer['data'].get('height_stable', True)
                or abs(peer['data']['support_z']-receive) > .03
                or abs(float(peer['data'].get('receive_z',receive))-receive) > .001
                or math.hypot(peer['data']['cargo_x']-delivery[0],
                              peer['data']['cargo_y']-delivery[1]) > .06
                or abs(angle(peer['data']['yaw']-delivery[3])) > .04
                or not node.handoff_contacts.supported(receiver_model,receiver_support)):
            raise MotionError('HANDOFF_RELEASE_SUPPORT_LOST: receiver must hold cargo')
        cargo = node.measured_cargo(goal)
        if math.dist(cargo[:2],delivery[:2]) > .12 or abs(cargo[2]-peer['data']['support_z']) > .04:
            raise MotionError('HANDOFF_RELEASE_CARGO_MOVED')
        node.hold_position()
        # Both existing receivers stay stopped in LOWERING. Only announce
        # RELEASED after a new native detached state has actually been read.
        part.emit('LOWERING', cargo_z=cargo[2])
        if time.monotonic()-last_release_feedback[0] >= .5:
            last_release_feedback[0] = time.monotonic()
            node.phase_feedback(goal, 92, 'RELEASING_TO_RECEIVER | awaiting_native_detach=true')

    def detach_confirmed():
        if node.native_state_revision > previous and node.native_attachment_state == 'detached':
            return True
        revision, state = node.native_attachment.state_snapshot()
        # Require an attached baseline and a newer native event from the same
        # Task002 plugin. No height/elapsed-time estimate can authorize release.
        if observed_state == 'attached' and revision > observed_revision and state == 'detached':
            node.measured_attachment_state(SimpleNamespace(data=state))
            node.phase_feedback(goal, 92, 'NATIVE_DETACH_CONFIRMED | source=gazebo_transport')
            return True
        return False

    node.native_attachment.detach(tick=release_tick, confirmed=detach_confirmed)
    node.wait_native_state(goal, 'detached', previous, tick=release_tick)
    node.object_attached = False
    node.publish_attachment_state()
    part.wait(
        {"LOAD_ACCEPTED"},
        "RELEASED",
        timeout=15,
        tick=lambda: node.phase_feedback(
            goal, 95, "RELEASED | verifying receiver support"
        ),
    )
    # Keep publishing RELEASED while moving the hook so the receiver continues
    # to hold position. Motion health also checks the coordinator lease.
    original = node.check_feedback

    def guarded(g, **kwargs):
        original(g, **kwargs)
        binding = node.handoff_channel.get(part.session, "coordinator", "task_manager")
        if binding is None or binding["stage"] == "STOP":
            raise MotionError("HANDOFF_STOPPED")
        part.emit("RELEASED")

    node.check_feedback = guarded
    try:
        node.physical_move(
            goal,
            {
                "hook_position": node.world_height_to_joint_position(
                    receive + node.attached_gap + 0.5
                )
            },
            95,
            99,
            "WITHDRAWING_HANDOFF_HOOK",
        )
    finally:
        node.check_feedback = original
    part.wait({"TRANSFER_COMPLETE"}, "HOOK_CLEAR", timeout=15)
    # The receiver acknowledges the hook-clear event before any base command.
    goal.succeed()
    node.phase_feedback(goal, 100, "SUCCESS | suspended_handoff_verified=true")
    result = ExecuteSkill.Result()
    result.success = True
    result.message = (
        "Crane handoff completed; contact verified; native joint detached; hook clear"
    )
    return result
