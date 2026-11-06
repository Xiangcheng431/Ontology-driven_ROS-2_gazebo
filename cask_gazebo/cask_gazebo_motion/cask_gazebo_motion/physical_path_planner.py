"""Shared orientation-aware planar collision planner; subclasses supply geometry."""
import heapq
import math


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class FootprintPlanner:
    BODY = ()
    WHEELS = ()
    LOAD = ()

    def __init__(self, bounds, resolution=.5, obstacle_radius=.45,
                 safety_margin=.20, loaded=False, pivot=0.):
        self.min_x, self.max_x, self.min_y, self.max_y = bounds
        self.resolution = resolution
        self.radius = obstacle_radius + safety_margin
        self.boxes = self.BODY + (self.LOAD if loaded else ())
        self.pivot = pivot
        self.start_yaw = self.goal_yaw = 0.

    def pose_clear(self, pose, obstacles):
        x, y, yaw = pose
        if not (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y):
            return False
        c, s = math.cos(yaw), math.sin(yaw)
        for ox, oy in obstacles:
            dx, dy = ox-x, oy-y
            lx, ly = c*dx+s*dy, -s*dx+c*dy
            for xmin, xmax, ymin, ymax in self.boxes:
                ex = max(xmin-lx, 0., lx-xmax)
                ey = max(ymin-ly, 0., ly-ymax)
                if math.hypot(ex, ey) <= self.radius:
                    return False
            if any(math.hypot(lx-wx, ly-wy) <= self.radius+wr
                   for wx, wy, wr in self.WHEELS):
                return False
        return True

    def turn_clear(self, xy, first_yaw, last_yaw, obstacles):
        delta = angle(last_yaw-first_yaw)
        count = max(1, math.ceil(abs(delta)/.025))
        for i in range(count+1):
            yaw = first_yaw+delta*i/count
            # Test both an ideal base-centred turn and the measured wheel pivot.
            # The latter shifts the base, especially while initially unloaded.
            for pivot in (0., self.pivot):
                x = xy[0]+pivot*(math.cos(first_yaw)-math.cos(yaw))
                y = xy[1]+pivot*(math.sin(first_yaw)-math.sin(yaw))
                if not self.pose_clear((x, y, yaw), obstacles):
                    return False
        return True

    def segment_clear(self, first, second, obstacles):
        distance = math.dist(first, second)
        yaw = math.atan2(second[1]-first[1], second[0]-first[0]) if distance else self.start_yaw
        count = max(1, math.ceil(distance/.05))
        return all(self.pose_clear((first[0]+(second[0]-first[0])*i/count,
                                   first[1]+(second[1]-first[1])*i/count, yaw), obstacles)
                   for i in range(count+1))

    def edge_clear(self, first, second, yaw, obstacles):
        bearing = math.atan2(second[1]-first[1], second[0]-first[0])
        return (self.turn_clear(first, yaw, bearing, obstacles)
                and self.segment_clear(first, second, obstacles))

    def plan(self, start, goal, obstacles):
        obstacles = tuple(obstacles)
        if (not self.pose_clear((*start, self.start_yaw), obstacles)
                or not self.pose_clear((*goal, self.goal_yaw), obstacles)):
            return None
        bearing = math.atan2(goal[1]-start[1], goal[0]-start[0])
        if (self.edge_clear(start, goal, self.start_yaw, obstacles)
                and self.turn_clear(goal, bearing, self.goal_yaw, obstacles)):
            return [goal]
        # Eight headings are part of the search state: a narrow gap may admit
        # a straight pose while rejecting a turn at the same XY coordinate.
        directions = tuple((dx, dy) for dx, dy in
                           ((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)))
        def point(state):
            return start[0]+state[0]*self.resolution, start[1]+state[1]*self.resolution
        source = (0, 0, -1)
        edge_penalty = getattr(self, 'edge_penalty', None)
        queue, best, parents = [(0., 0., source)], {source: 0.}, {}
        expanded = 0
        while queue and expanded < 50000:
            _, cost, state = heapq.heappop(queue)
            if cost > best[state]+1e-9:
                continue
            expanded += 1
            xy = point(state)
            yaw = self.start_yaw if state[2] == -1 else state[2]*math.pi/4
            if math.dist(xy, goal) < getattr(self, 'goal_connection_distance', 1.5):
                final_yaw = math.atan2(goal[1]-xy[1], goal[0]-xy[0])
                if (self.edge_clear(xy, goal, yaw, obstacles)
                        and self.turn_clear(goal, final_yaw, self.goal_yaw, obstacles)):
                    path = [goal]
                    while state != source:
                        path.append(point(state))
                        state = parents[state]
                    path.reverse()
                    return self.smooth(start, self.start_yaw, path, obstacles)
            for heading, (dx, dy) in enumerate(directions):
                nxt = (state[0]+dx, state[1]+dy, heading)
                nxtxy = point(nxt)
                if not self.edge_clear(xy, nxtxy, yaw, obstacles):
                    continue
                newcost = cost+math.dist(xy,nxtxy)+.4*abs(angle(heading*math.pi/4-yaw))
                if edge_penalty:
                    newcost += edge_penalty(xy, nxtxy, start, goal)
                if newcost >= best.get(nxt, math.inf):
                    continue
                best[nxt], parents[nxt] = newcost, state
                heapq.heappush(queue, (newcost+math.dist(nxtxy,goal), newcost, nxt))
        return None

    def smooth(self, start, yaw, path, obstacles):
        result, anchor, index = [], start, 0
        while index < len(path):
            chosen = index
            for j in range(len(path)-1, index-1, -1):
                heading = math.atan2(path[j][1]-anchor[1], path[j][0]-anchor[0])
                after = (self.goal_yaw if j == len(path)-1 else
                         math.atan2(path[j+1][1]-path[j][1],path[j+1][0]-path[j][0]))
                if (self.edge_clear(anchor,path[j],yaw,obstacles)
                        and self.turn_clear(path[j],heading,after,obstacles)):
                    chosen = j
                    break
            yaw = math.atan2(path[chosen][1]-anchor[1],path[chosen][0]-anchor[0])
            anchor = path[chosen]
            result.append(anchor)
            index = chosen+1
        return result

    def command_clear(self, pose, velocity, omega, pivot, obstacles, horizon=1.):
        # Predict base motion around the controller reference axle.
        x,y,yaw = pose
        for _ in range(20):
            dt = horizon/20
            x += (velocity*math.cos(yaw)+pivot*omega*math.sin(yaw))*dt
            y += (velocity*math.sin(yaw)-pivot*omega*math.cos(yaw))*dt
            yaw += omega*dt
            if not self.pose_clear((x,y,yaw),obstacles):
                return False
        return True
