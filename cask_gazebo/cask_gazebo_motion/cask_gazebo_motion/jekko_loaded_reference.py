"""Simulation-time Jekko reference shaping with braking-distance-aware arrival."""
import math


class LoadedReference:
    def __init__(self, measured, stamp):
        self.position, self.velocity, self.stamp = measured, 0., stamp

    def advance(self, target, measured, stamp, speed, acceleration, bounds,
                slow_distance=0., slow_speed=None):
        if stamp < self.stamp:
            self.position, self.velocity, self.stamp = measured, 0., stamp
            return measured
        dt = min(.1, stamp-self.stamp)
        self.stamp = stamp
        steps = max(1, math.ceil(dt/.01))
        step = dt/steps
        for _ in range(steps):
            distance = target-self.position
            limit = speed
            if slow_speed is not None and abs(distance)<=slow_distance:
                limit = min(limit,slow_speed)
            # Reserve both the next integration step and the full braking
            # distance. Clipping a spring acceleration alone brakes too late.
            braking = math.sqrt((acceleration*step)**2+2.*acceleration*abs(distance))-acceleration*step
            desired = math.copysign(min(limit,4.*abs(distance),braking),distance)
            change = max(-acceleration*step,min(acceleration*step,desired-self.velocity))
            self.velocity = max(-speed,min(speed,self.velocity+change))
            self.position += self.velocity*step
            clipped = max(bounds[0],min(bounds[1],self.position))
            if clipped != self.position:
                self.position, self.velocity = clipped, 0.
        return self.position
