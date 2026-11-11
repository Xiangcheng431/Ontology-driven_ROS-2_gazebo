"""Robot-sized footprint planning and measured-pose steering, independent of ROS."""
import math
from .physical_path_planner import FootprintPlanner, angle


FOOTPRINTS = {'jekko': (-.94, 1.92, -.79, .79),
              'innok223': (-2.7, .75, -.75, .75)}


def approach_pose(x, y, yaw, forward_offset):
    """Product/site coordinates -> base goal, along the ontology-specified heading."""
    if not all(math.isfinite(v) for v in (x, y, yaw, forward_offset)):
        raise ValueError('Non-finite navigation target')
    return (x-forward_offset*math.cos(yaw), y-forward_offset*math.sin(yaw), angle(yaw))


class VehiclePlanner(FootprintPlanner):
    """Reuse the shared oriented A* and smoothing with vehicle-specific geometry."""
    WHEELS = ()
    LOAD = ()

    def __init__(self, robot, bounds, safety_margin=.25, resolution=.75, health=None, pivot=0.):
        self.BODY = (FOOTPRINTS[robot],)
        super().__init__(bounds, resolution=resolution, safety_margin=safety_margin,
                         obstacle_radius=0., pivot=pivot)
        self.health = health
        self.calls = 0
        self._clearance_cache = None
        self.prefer_left = False

    def edge_penalty(self, first, second, start, goal):
        if not self.prefer_left:
            return 0.
        dx,dy = goal[0]-start[0],goal[1]-start[1]
        length = math.hypot(dx,dy)
        if length < 1e-9:
            return 0.
        mx,my = (first[0]+second[0])/2-start[0],(first[1]+second[1])/2-start[1]
        side = (dx*my-dy*mx)/length
        # Left is relative to the current start->goal direction. Both sides
        # retain full collision checks; the right remains available if needed.
        return .8*max(0.,-side)*math.dist(first,second)

    def plan(self, start, goal, obstacles):
        # A* revisits the same grid edge with different incoming headings.
        # Reuse exact checks only within this immutable obstacle snapshot.
        self._clearance_cache = {}
        try:
            if self.health:
                self.health()
            return super().plan(start, goal, tuple(obstacles))
        finally:
            self._clearance_cache = None

    def _checked(self, method, *args):
        if self.health:
            self.health()  # Cached checks must still honour cancellation/deadline.
        cache = self._clearance_cache
        if cache is None:
            return getattr(super(), method)(*args)
        key = (method, *args[:-1])
        if key not in cache:
            cache[key] = getattr(super(), method)(*args)
        return cache[key]

    def turn_clear(self, xy, first_yaw, last_yaw, obstacles):
        # Most grid cells are far from obstacles. An enclosing swept circle
        # proves the entire turn clear without sampling every intermediate pose.
        for pivot in (0., self.pivot):
            cx = xy[0]+pivot*math.cos(first_yaw)
            cy = xy[1]+pivot*math.sin(first_yaw)
            radius = max(math.hypot(px-pivot, py) for xmin,xmax,ymin,ymax in self.boxes
                         for px,py in ((xmin,ymin),(xmin,ymax),(xmax,ymin),(xmax,ymax)))
            if (cx-radius < self.min_x or cx+radius > self.max_x or
                    cy-radius < self.min_y or cy+radius > self.max_y or
                    any(math.hypot(ox-cx,oy-cy) <= radius+r+self.radius
                        for ox,oy,r in obstacles)):
                break
        else:
            if self.health:
                self.health()
            return True
        return self._checked('turn_clear', tuple(xy), first_yaw, last_yaw, obstacles)

    def segment_clear(self, first, second, obstacles):
        return self._checked('segment_clear', tuple(first), tuple(second), obstacles)

    def pose_clear(self, pose, obstacles):
        self.calls += 1
        if self.health and self.calls % 128 == 0:
            self.health()
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        for xmin, xmax, ymin, ymax in self.boxes:
            # Unlike a centre-only bounds check, keep the full body within the map.
            for px, py in ((xmin,ymin),(xmin,ymax),(xmax,ymin),(xmax,ymax)):
                wx, wy = x+c*px-s*py, y+s*px+c*py
                if not (self.min_x <= wx <= self.max_x and self.min_y <= wy <= self.max_y):
                    return False
            for ox, oy, radius in obstacles:
                dx, dy = ox-x, oy-y
                lx, ly = c*dx+s*dy, -s*dx+c*dy
                ex, ey = max(xmin-lx,0.,lx-xmax), max(ymin-ly,0.,ly-ymax)
                if math.hypot(ex,ey) <= radius+self.radius:
                    return False
        return True


class PoseController:
    """Align, drive, slow near goal, latch final yaw; never integrate an invented pose."""
    def __init__(self, speed=.35, angular_speed=.20, arrival_tolerance=None, fixed_pivot=None):
        self.speed, self.angular_speed = speed, angular_speed
        self.arrival_tolerance = arrival_tolerance
        self.align_final = False
        self.fixed_pivot = fixed_pivot
        self.pivot = 0. if fixed_pivot is None else fixed_pivot
        self.sample = None

    def observe(self, pose):
        if self.fixed_pivot is not None:
            return
        if self.sample is None:
            self.sample = pose
            return
        old = self.sample
        da = angle(pose[2]-old[2])
        if abs(da) >= .035:
            mid = old[2]+da/2
            lateral = -(pose[0]-old[0])*math.sin(mid)+(pose[1]-old[1])*math.cos(mid)
            estimate = -lateral/(2*math.sin(da/2))
            if -1.5 <= estimate <= 1.5:
                self.pivot = .7*self.pivot+.3*estimate
            self.sample = tuple(pose)
        elif math.dist(pose[:2],old[:2]) > .1:
            self.sample = tuple(pose)

    def command(self, pose, waypoint, target, terminal):
        base_distance = math.dist(pose[:2],target[:2])
        # Near a work pose, slip during small turns is a poor pivot measurement.
        # Keep the estimate from the approach instead of moving the control
        # reference on every noisy centimetre correction.
        if self.arrival_tolerance is None or base_distance > .5:
            self.observe(pose)
        x, y, yaw = pose
        if terminal:
            x += self.pivot*math.cos(yaw)
            y += self.pivot*math.sin(yaw)
            waypoint = (target[0]+self.pivot*math.cos(target[2]),
                        target[1]+self.pivot*math.sin(target[2]))
        rho = math.dist((x,y),waypoint)
        yaw_error = angle(target[2]-yaw)
        if terminal and self.arrival_tolerance is not None:
            position_tolerance,yaw_tolerance=self.arrival_tolerance
            # An almost opposite heading can shift the base by twice the
            # wheel-pivot offset. In that case finish centre position first.
            if (base_distance <= position_tolerance and abs(yaw_error)<=.35) or rho <= .025:
                self.align_final=True
            if self.align_final:
                # Finish yaw before reconsidering position; the completion
                # check uses the actual base pose, not the estimated centre.
                if abs(yaw_error)<=yaw_tolerance/2 and base_distance>position_tolerance:
                    self.align_final=False
                else:
                    omega=max(-.08,min(.08,self.clamp(.8*yaw_error)))
                    return 0.,omega,'ALIGNING_WORK_YAW'
        if terminal and rho <= .025:
            self.align_final = True
        if self.align_final:
            # Hysteresis prevents toggling steering when final yaw rotates the body slightly.
            if abs(yaw_error) < .025 and rho > .025:
                self.align_final = False
            else:
                return 0., self.clamp(.9*yaw_error), 'ALIGNING_YAW'
        heading = angle(math.atan2(waypoint[1]-y,waypoint[0]-x)-yaw)
        direction = 1.
        if terminal and rho < .7 and abs(heading) > math.pi/2:
            direction, heading = -1., angle(heading-math.pi)
        omega = self.clamp(1.2*heading)
        if self.arrival_tolerance is not None and terminal and base_distance<.5:
            omega=max(-.08,min(.08,omega))
        limit = min(self.speed,.08) if terminal and rho < .7 else self.speed
        velocity = direction*min(limit,.5*rho)*max(0.,math.cos(heading))
        if abs(heading) > (.12 if terminal and rho < .7 else .35):
            velocity = 0.
        return velocity,omega,('ALIGNING_HEADING' if velocity == 0. else 'FOLLOWING_PATH')

    def clamp(self,value):
        return max(-self.angular_speed,min(self.angular_speed,value))


def goal_values(request):
    if not (len(request.parameter_names)==len(request.parameter_values)==len(request.parameter_units)):
        raise ValueError('Navigation parameter array lengths differ')
    values = dict(zip(request.parameter_names,request.parameter_values))
    units = dict(zip(request.parameter_names,request.parameter_units))
    if len(values)!=len(request.parameter_names):
        raise ValueError('Duplicate navigation parameter')
    for key,unit in (('target_x','m'),('target_y','m'),('target_yaw','rad')):
        if key not in values or units.get(key)!=unit or not math.isfinite(values[key]):
            raise ValueError('Missing/invalid '+key+'; coordinates must be supplied by the parameterizer')
    return values['target_x'],values['target_y'],values['target_yaw']
