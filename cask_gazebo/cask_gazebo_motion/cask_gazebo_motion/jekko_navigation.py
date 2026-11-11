"""Measured-pose approach control for the mobile Jekko model only."""
import math
from .ground_navigation import PoseController
from .task005_geometry import angle


class JekkoPoseController(PoseController):
    def command(self, pose, waypoint, target, terminal):
        # The superstructure compensates chassis yaw; stop at the original XY tolerance.
        x, y, yaw = pose
        if terminal:
            waypoint = target[:2]
        rho = math.dist((x, y), waypoint)
        if terminal and rho <= self.arrival_tolerance[0]:
            return 0., 0., 'WORK_POSITION_REACHED'
        heading = angle(math.atan2(waypoint[1]-y, waypoint[0]-x)-yaw)
        direction = 1.
        if terminal and rho < .7 and abs(heading) > math.pi/2:
            direction, heading = -1., angle(heading-math.pi)
        omega = self.clamp(1.2*heading)
        # Retain fine alignment at the last centimetres, without capping a large
        # heading correction at .08 rad/s throughout the final half metre.
        if terminal and rho < .10:
            omega = max(-.08, min(.08, omega))
        limit = min(self.speed, .16) if terminal and rho < .7 else self.speed
        velocity = direction*min(limit, .7*rho)*max(0., math.cos(heading))
        heading_limit = .35 if not terminal or rho >= .7 else (.20 if rho < .10 else .45)
        if abs(heading) > heading_limit:
            velocity = 0.
        # Task005 terminal speed +25%; preserve existing decisions and curvature.
        if terminal and math.dist(pose[:2], target[:2]) < .7:
            final_speed_factor = 1.25
            if abs(velocity) > 1e-12:
                final_speed_factor = min(final_speed_factor, self.speed/abs(velocity))
            if abs(omega) > 1e-12:
                final_speed_factor = min(final_speed_factor, self.angular_speed/abs(omega))
            velocity, omega = velocity*final_speed_factor, omega*final_speed_factor
        return velocity, omega, ('ALIGNING_HEADING' if velocity == 0. else 'FOLLOWING_PATH')
