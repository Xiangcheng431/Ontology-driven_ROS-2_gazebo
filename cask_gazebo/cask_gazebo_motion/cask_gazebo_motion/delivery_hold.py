"""Continuous measured cargo hold, timed using the caller's simulation clock."""
import math
import time


def hold_delivery(read_pose, command_hold, sim_seconds, report, target, duration,
                  timeout=60.0, sleep=time.sleep, wall=time.monotonic):
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('Invalid hold duration')
    if duration == 0:
        return
    start = None
    previous = None
    began = wall()
    while True:
        command_hold()
        pose = read_pose()  # caller checks cancellation and fresh measured data
        now = sim_seconds()
        if not math.isfinite(now):
            raise RuntimeError('HOLD_CLOCK_INVALID')
        if previous is not None and now < previous:
            start = None
        previous = now
        yaw_error = math.atan2(math.sin(pose[3]-target[3]),math.cos(pose[3]-target[3]))
        valid = (all(math.isfinite(v) for v in pose)
                 and math.dist(pose[:2],target[:2]) <= .15
                 and abs(pose[2]-target[2]) <= .05 and abs(yaw_error) <= .06)
        if not valid:
            start = None
        elif start is None:
            start = now
        elapsed = 0. if start is None else now-start
        report(f'HOLDING_AT_DELIVERY | measured_cargo_z={pose[2]:.3f} | '
               f'held_seconds={elapsed:.3f} | required_seconds={duration:.3f}')
        if valid and elapsed >= duration:
            report(f'HOLD_VERIFIED | held_seconds={elapsed:.3f}')
            return
        if wall()-began > max(timeout,duration*4+10):
            raise RuntimeError('HOLD_TIMEOUT: no continuous verified interval')
        sleep(.1)
