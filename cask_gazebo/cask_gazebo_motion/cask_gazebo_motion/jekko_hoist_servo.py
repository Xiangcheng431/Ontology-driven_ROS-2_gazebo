"""Position goals -> bounded hoist velocity, using measured feedback.

The final launch selects Fortress JointController for this joint only. Targets
elsewhere remain positions; the conversion occurs at the actuator boundary.
"""
import math


class HoistServo:
    def __init__(self):
        self.stamp = None
        self.velocity = 0.

    def stop(self, stamp):
        self.stamp, self.velocity = stamp, 0.
        return 0.

    def command(self, target, measured, stamp, loaded, factor=.875,
                descent_velocity=None, measured_velocity=0.):
        if not all(math.isfinite(v) for v in (target, measured, stamp, factor, measured_velocity)):
            raise ValueError('INVALID_HOIST_SERVO_INPUT')
        if not 0. <= target <= 3. or not 0. < factor <= 1.:
            raise ValueError('INVALID_HOIST_SERVO_TARGET')
        if self.stamp is None or stamp < self.stamp:
            return self.stop(stamp)
        dt = min(.05, max(0., stamp-self.stamp))
        self.stamp = stamp
        speed = (.30 if loaded else .40)*factor
        acceleration = (.60 if loaded else .80)*factor*factor
        error = target-measured
        if descent_velocity is None:
            if abs(error) <= .0006:
                return self.stop(stamp)
            # Reserve braking distance for actuator/feedback delay. Using the
            # full acceleration in sqrt(2*a*d) brakes too late once the motor
            # and the ROS measurement are delayed.
            braking = .4*acceleration
            distance = max(0., abs(error)-.12*max(0.,math.copysign(1.,error)*measured_velocity))
            limit = math.sqrt((braking*.12)**2+2.*braking*distance)-braking*.12
            desired = math.copysign(min(speed, 4.*distance, limit), error)
        else:
            if not math.isfinite(descent_velocity) or descent_velocity < 0.:
                raise ValueError('INVALID_PLACEMENT_VELOCITY')
            desired = min(speed, descent_velocity)
            # Gap control never retracts the load for a noisy gap sample.
            if desired == 0.:
                return self.stop(stamp)
        # Never retain a velocity pointing away from a newly reached target.
        if descent_velocity is None and self.velocity*error < 0.:
            self.velocity = 0.
        change = max(-acceleration*dt, min(acceleration*dt, desired-self.velocity))
        self.velocity = max(-speed, min(speed, self.velocity+change))
        if (measured <= .0006 and self.velocity < 0.
                or measured >= 2.9994 and self.velocity > 0.):
            return self.stop(stamp)
        return self.velocity
