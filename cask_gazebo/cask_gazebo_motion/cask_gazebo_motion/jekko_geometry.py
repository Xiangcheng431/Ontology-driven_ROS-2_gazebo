import math
from .task005_geometry import angle

def jekko_targets(base, hook, cargo_yaw, length=2.9):
    """Stationary slew/telescope/hoist inverse kinematics; metres and radians."""
    x, y, z, yaw = base
    pivot = (x-.08*math.cos(yaw), y-.08*math.sin(yaw))
    radius = math.dist(pivot, hook[:2])
    # CASK_TASK005_SOUTH_SPEED_V1: include vertical reach, not only XY radius.
    rise = hook[2]+.58-z-1.11
    needed = max(2.12, length, radius/math.cos(.20),
                 math.hypot(radius, max(0., rise+.10)))
    if needed > 4.42+1e-9:
        raise ValueError('JEKKO_TARGET_OUT_OF_REACH')
    length = min(4.42, needed)
    if radius > length or radius < 2.12*math.cos(1.39):
        raise ValueError('JEKKO_TARGET_OUT_OF_REACH')
    elevation = math.acos(radius/length)
    pitch = elevation-.20
    # CASK_GENERATED_IOC_V1: tolerate only floating-point roundoff at the lower bound.
    if -1e-9 < pitch < 0.: pitch = 0.
    hoist = z+1.11+length*math.sin(elevation)-.58-hook[2]
    if -1e-9 < hoist < 0.: hoist = 0.
    if not (0 <= pitch <= 1.19 and 0 <= hoist <= 3):
        raise ValueError('JEKKO_TARGET_EXCEEDS_JOINT_LIMITS')
    slew = angle(math.atan2(hook[1]-pivot[1], hook[0]-pivot[0])-yaw)
    extension = (length-2.12)/2
    return {'jekko_slew_joint': slew, 'jekko_boom_pitch_joint': pitch,
            'jekko_telescope_1_joint': extension, 'jekko_telescope_2_joint': extension,
            'jekko_hook_level_joint': pitch, 'jekko_hoist_joint': hoist,
            'jekko_hook_yaw_joint': angle(cargo_yaw-yaw-slew)}



def jekko_hook(base, q):
    x, y, z, yaw = base
    length = 2.12+q['jekko_telescope_1_joint']+q['jekko_telescope_2_joint']
    elevation = .20+q['jekko_boom_pitch_joint']
    bearing = yaw+q['jekko_slew_joint']
    return (x-.08*math.cos(yaw)+length*math.cos(elevation)*math.cos(bearing),
            y-.08*math.sin(yaw)+length*math.cos(elevation)*math.sin(bearing),
            z+1.11+length*math.sin(elevation)-q['jekko_hoist_joint']-.58,
            angle(bearing+q['jekko_hook_yaw_joint']))

