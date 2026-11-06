import math
from cask_gazebo_motion.task005_geometry import angle, rectangles_overlap
from cask_gazebo_motion.physical_path_planner import FootprintPlanner
from cask_gazebo_motion.ground_navigation import VehiclePlanner


class Task005RoutePlanner(VehiclePlanner):
    def __init__(self, robot, health):
        if robot == 'jekko':
            # Retracted feet/legs and raised stowed boom: retain each envelope separately.
            self.BODY = ((-.94, .94, -.37, .37),
                (.60, 1.31, .14, .79), (.60, 1.31, -.79, -.14),
                (-1.32, -.60, .14, .79), (-1.32, -.60, -.79, -.14),
                (-.22, 1.92, -.18, .18))
        else:
            raise ValueError('This route profile is for Jekko')
        FootprintPlanner.__init__(self, (-20., 20., -10., 12.),
                                        resolution=.4, obstacle_radius=0., safety_margin=.07, pivot=0.)
        self.health, self.calls, self._clearance_cache = health, 0, None
        self.prefer_left = True
        self.static_boxes = ()
        self.reverse = False
        self.forward_boxes = self.boxes
        self._turn_radius = max(math.hypot(x,y)
            for x0,x1,y0,y1 in self.boxes
            for x,y in ((x0,y0),(x0,y1),(x1,y0),(x1,y1)))

    def virtual_pose(self, pose):
        return (pose[0],pose[1],angle(pose[2]+(math.pi if self.reverse else 0.)))

    def plan_route(self, start, goal, obstacles):
        # Prefer the mode compatible with the requested parking orientation.
        # At a bench-side west-facing work pose, reverse approach avoids an
        # unnecessary full forward search for a turn beside the workbench.
        bearing = math.atan2(goal[1]-start[1],goal[0]-start[0])
        prefer_reverse = getattr(self,'prefer_reverse',
                                 abs(angle(goal[2]-bearing))>math.pi/2)
        modes = (True,False) if prefer_reverse else (False,True)
        for reverse in modes:
            self.reverse = reverse
            self.boxes = tuple((-x1,-x0,-y1,-y0) for x0,x1,y0,y1 in self.forward_boxes) if reverse else self.forward_boxes
            self.start_yaw,self.goal_yaw = self.virtual_pose(start)[2],self.virtual_pose(goal)[2]
            path = self.plan(start[:2],goal[:2],obstacles)
            if path is not None:
                return path
        return None

    def pose_clear(self, pose, obstacles):
        # Exact results are reused only during one plan with fixed obstacles,
        # static boxes and body orientation mode. Live driving has no cache.
        cache = self._clearance_cache
        key = ('task005_pose',tuple(pose))
        if cache is not None and key in cache:
            return cache[key]
        clear = (super().pose_clear(pose,obstacles)
            and not any(rectangles_overlap((*pose,*box),static,.07)
                        for box in self.boxes for static in self.static_boxes))
        if cache is not None:
            cache[key] = clear
        return clear

    def route_still_clear(self, pose, path, target, obstacles):
        """Recheck a retained route against the current complete peer snapshot."""
        previous = self._clearance_cache
        self._clearance_cache = {}
        try:
            xy,yaw = pose[:2],pose[2]
            for point in path:
                if math.dist(xy,point)<1e-8:
                    continue
                if not self.edge_clear(xy,point,yaw,obstacles):
                    return False
                yaw = math.atan2(point[1]-xy[1],point[0]-xy[0])
                xy = point
            return self.turn_clear(xy,yaw,target[2],obstacles)
        finally:
            self._clearance_cache = previous

    def turn_clear(self, xy, first_yaw, last_yaw, obstacles):
        if self.health:
            self.health()
        # A circle containing the entire rotating footprint proves clear only
        # if it also misses rectangular supports. Nearby turns retain all of
        # the original 0.025-radian samples and full rectangle checks.
        x,y = xy
        radius = self._turn_radius
        if (self.pivot == 0.
                and self.min_x <= x-radius and x+radius <= self.max_x
                and self.min_y <= y-radius and y+radius <= self.max_y
                and all(math.hypot(ox-x,oy-y) > radius+r+self.radius
                        for ox,oy,r in obstacles)
                and all(self._outside_support(x,y,radius,box) for box in self.static_boxes)):
            return True
        return self._checked('turn_clear',tuple(xy),first_yaw,last_yaw,obstacles)

    @staticmethod
    def _outside_support(x,y,radius,box):
        bx,by,yaw,x0,x1,y0,y1 = box
        c,s = math.cos(yaw),math.sin(yaw)
        dx,dy = x-bx,y-by
        lx,ly = c*dx+s*dy,-s*dx+c*dy
        # Square padding of .07 on the moving rectangle adds at most
        # sqrt(2)*.07 to its enclosing radius, regardless of heading.
        return math.hypot(max(x0-lx,0.,lx-x1),max(y0-ly,0.,ly-y1)) > radius+math.sqrt(2.)*.07

