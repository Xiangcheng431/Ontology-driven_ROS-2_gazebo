"""World-frame grid path planner shared by mobile robot Skill Servers.

This plans a geometric path for a kinematic visualization. It does not model
steering limits, acceleration, sensing, or physical collision dynamics.
"""

import heapq
import math


class GridPlanner:
    def __init__(self, bounds=(-8.0, 8.0, -2.0, 8.0), resolution=0.25,
                 robot_radius=0.45, obstacle_radius=0.45):
        self.min_x, self.max_x, self.min_y, self.max_y = bounds
        self.resolution = float(resolution)
        self.clearance = float(robot_radius) + float(obstacle_radius)
        if (self.resolution <= 0 or self.clearance < 0
                or self.max_x <= self.min_x or self.max_y <= self.min_y):
            raise ValueError('Invalid grid bounds, resolution, or radii')
        self.nx = int(math.floor((self.max_x - self.min_x) / resolution)) + 1
        self.ny = int(math.floor((self.max_y - self.min_y) / resolution)) + 1

    def cell(self, point):
        return (round((point[0] - self.min_x) / self.resolution),
                round((point[1] - self.min_y) / self.resolution))

    def center(self, cell):
        return (self.min_x + cell[0] * self.resolution,
                self.min_y + cell[1] * self.resolution)

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.nx and 0 <= cell[1] < self.ny

    def point_clear(self, point, obstacles, margin=0.0):
        x, y = point
        if not (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y):
            return False
        return all(math.hypot(x - ox, y - oy) > self.clearance + margin
                   for ox, oy in obstacles)

    def segment_clear(self, first, second, obstacles):
        distance = math.dist(first, second)
        count = max(1, math.ceil(distance / (self.resolution / 4)))
        return all(self.point_clear((first[0] + (second[0] - first[0]) * i / count,
                                     first[1] + (second[1] - first[1]) * i / count),
                                    obstacles)
                   for i in range(count + 1))

    def plan(self, start, goal, obstacles):
        """Return waypoints ending at the exact goal, or None if blocked."""
        obstacles = tuple(obstacles)
        if not self.point_clear(start, obstacles) or not self.point_clear(goal, obstacles):
            return None
        if self.segment_clear(start, goal, obstacles):
            return [goal]

        source, target = self.cell(start), self.cell(goal)
        if not self.in_bounds(source) or not self.in_bounds(target):
            return None

        # Grid cells are inflated by half the cell diagonal, so a path between
        # adjacent free centers cannot graze a circular obstacle.
        margin = self.resolution / math.sqrt(2)
        def free(cell):
            return (self.in_bounds(cell) and
                    self.point_clear(self.center(cell), obstacles, margin))

        if not free(source) or not free(target):
            return None
        frontier = [(0.0, 0.0, source)]
        best = {source: 0.0}
        parent = {}
        offsets = ((1, 0), (-1, 0), (0, 1), (0, -1),
                   (1, 1), (1, -1), (-1, 1), (-1, -1))

        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if cost > best[current] + 1e-9:
                continue
            if current == target:
                cells = []
                while current != source:
                    cells.append(current)
                    current = parent[current]
                cells.reverse()
                waypoints = [self.center(cell) for cell in cells]
                if not waypoints or not self.segment_clear(start, waypoints[0], obstacles):
                    return None
                if not self.segment_clear(waypoints[-1], goal, obstacles):
                    return None
                waypoints.append(goal)
                return waypoints

            for dx, dy in offsets:
                neighbor = (current[0] + dx, current[1] + dy)
                if not free(neighbor):
                    continue
                if dx and dy and (not free((current[0] + dx, current[1]))
                                  or not free((current[0], current[1] + dy))):
                    continue
                new_cost = cost + math.hypot(dx, dy)
                if new_cost >= best.get(neighbor, math.inf):
                    continue
                best[neighbor] = new_cost
                parent[neighbor] = current
                heuristic = math.dist(neighbor, target)
                heapq.heappush(frontier, (new_cost + heuristic, new_cost, neighbor))
        return None
