"""Measured, bounded-position-lead hoist motion using the existing interface."""
import math
import time


def vertical_move(node, goal, target, stage, tolerance=.002, verify=None, settle=True):
    name = 'jekko_hoist_joint'
    if not math.isfinite(target) or not 0.<=target<=3.:
        raise RuntimeError('INVALID_HOIST_TARGET')
    factor = node._motion_speed_factor
    speed = (.30 if node.loaded else .40)*factor
    node.check(goal,True)
    initial = node.joints[name]
    direction = 1. if target>=initial else -1.
    command = initial
    began = time.monotonic()
    last_sim = node.get_clock().now().nanoseconds*1e-9
    last_report = -math.inf
    stable = None
    previous = None
    completed = False
    with node._joint_command_lock:
        node._vertical_control_active = True
        getattr(node,'_loaded_references',{}).pop(name,None)
        node.targets[name] = command
    try:
        while time.monotonic()-began<node.get_parameter('motion_timeout').value:
            node.check(goal,True)
            sim = node.get_clock().now().nanoseconds*1e-9
            dt = min(.05,max(0.,sim-last_sim))
            last_sim = sim
            measured = node.joints[name]
            remaining = abs(target-measured)
            limit = min(speed,.06*factor) if remaining<.20 else speed
            limit = min(limit,2.*remaining)
            # One monotone command stream; delayed feedback cannot create a
            # virtual trajectory far ahead of the actual joint.
            proposed = command+direction*limit*dt
            if direction>0:
                command = max(command,min(target,proposed,measured+.004))
            else:
                command = min(command,max(target,proposed,measured-.004))
            with node._joint_command_lock:
                node.targets[name] = command
                node.hold()
            sample = node.joints_time
            if previous is not None and sample!=previous[2] and sim>previous[0]:
                estimated = abs(measured-previous[1])/(sim-previous[0])
                reported = abs(getattr(node,'joint_velocities',{}).get(name,0.))
                ready = (remaining<=min(tolerance,.002) and abs(command-target)<=.001
                         and estimated<=.008 and reported<=.01
                         and (verify is None or verify()))
                if ready:
                    stable = sim if stable is None else stable
                    if sim-stable>=(.20 if settle else .025):
                        completed = True
                        return
                else:
                    stable = None
            if previous is None or sample!=previous[2]:
                previous = sim,measured,sample
            if time.monotonic()-last_report>=.3:
                node.feedback(goal,50.,stage+' | MEASURED_HOIST_CONTROL'
                    +f' | measured={measured:.5f} | command={command:.5f} | target={target:.5f}'
                    +f' | speed_cap={limit:.4f} | speed_factor={factor:.3f}')
                last_report = time.monotonic()
            time.sleep(.015)
        raise RuntimeError('HOIST_MOTION_TIMEOUT: '+stage)
    finally:
        with node._joint_command_lock:
            node.targets[name] = target if completed else node.joints[name]
            node._vertical_control_active = False
            getattr(node,'_loaded_references',{}).pop(name,None)

