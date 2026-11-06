"""Three-wheel tractor geometry; positive steering turns left when driving forward."""
import math

WHEELBASE = .55
STEERING_LIMIT = math.radians(85.)
MAX_YAW_RATE = .23


def steering_commands(linear, angular, measured, limit=STEERING_LIMIT):
    """Match rear-wheel speeds to the measured front steering, including reverse.

    Steering changes while rolling. A zero linear command always stops yaw;
    the tractor cannot execute the old in-place rotation primitive.
    """
    if not all(math.isfinite(value) for value in (linear,angular,measured,limit)):
        raise ValueError('NON_FINITE_STEERING_COMMAND')
    if not 0. < limit <= STEERING_LIMIT:
        raise ValueError('STEERING_LIMIT_MUST_BE_WITHIN_85_DEGREES')
    measured=max(-limit,min(limit,measured))
    if abs(linear)<1e-6:
        return measured,0.,0.
    desired=max(-limit,min(limit,math.atan(WHEELBASE*angular/linear)))
    curvature=math.tan(measured)/WHEELBASE
    # Slow in tight turns rather than demanding rear-wheel slip or stationary yaw.
    speed=math.copysign(min(abs(linear),MAX_YAW_RATE/max(abs(curvature),1e-9)),linear)
    return desired,speed,speed*curvature
