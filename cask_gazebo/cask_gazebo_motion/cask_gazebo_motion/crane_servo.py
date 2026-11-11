"""Crane command conditioning; task targets and completion rules stay upstream."""
import math


class AngularServo:
    # Both thresholds remain inside physical_move's existing 0.004 rad limit.
    ENTER = .0015
    EXIT = .003
    ACCELERATION = .30  # rad/s^2, measured in simulation time

    def __init__(self):
        self.target = None
        self.holding = False
        self.speed = 0.0
        self.stamp = None

    def stop(self):
        self.speed = 0.0
        self.stamp = None

    def step(self, current, target, desired_speed, sim_seconds):
        if not all(math.isfinite(v) for v in (current, target, desired_speed, sim_seconds)):
            self.stop()
            return 0.0
        if target != self.target:
            self.target = target
            self.holding = False
        error = abs(target-current)  # Keep the existing raw-angle rotation path.
        if self.holding and error > self.EXIT:
            self.holding = False
        elif not self.holding and error <= self.ENTER:
            self.holding = True
        if self.stamp is None or sim_seconds < self.stamp:
            self.speed = 0.0
            dt = 0.0
        else:
            dt = min(.05, sim_seconds-self.stamp)
        self.stamp = sim_seconds
        if self.holding:
            # At this distance P feedback is already almost zero. Brake rather
            # than alternating tiny positive/negative commands around the goal.
            self.speed = 0.0
        else:
            change = self.ACCELERATION*dt
            self.speed += max(-change, min(change, desired_speed-self.speed))
        return self.speed


class TrolleyServo(AngularServo):
    # Control the real joint with bounded velocity, rather than the native
    # position plugin's timestep-dependent perfect-position correction.
    ENTER = .00075
    EXIT = .002
    ACCELERATION = .75  # m/s^2, measured in simulation time
    GAIN = 1.5
    MAX_SPEED = .4125  # Existing crane 1.5x speed limit, in m/s.

    def command(self, current, target, sim_seconds):
        desired = max(-self.MAX_SPEED, min(self.MAX_SPEED, self.GAIN*(target-current)))
        return self.step(current, target, desired, sim_seconds)
