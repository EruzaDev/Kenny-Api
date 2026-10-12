"""Live-map routing and nearer-call selection; no simulator truth queries."""
import heapq
import math

import numpy as np


class Planner:
    def __init__(self, grid, radius=.163, resolution=.1):
        self.origin = grid['origin'][:2]
        if abs(grid['origin'][2]) > 1e-9:
            raise ValueError('Planner expects the axis-aligned refined grid')
        scale = max(1, math.ceil(resolution / grid['resolution']))
        self.resolution = scale * grid['resolution']
        raw = np.asarray(grid['data']).reshape(grid['height'], grid['width'])
        h, w = math.ceil(raw.shape[0] / scale), math.ceil(raw.shape[1] / scale)
        blocked = np.ones((h * scale, w * scale), dtype=bool)
        blocked[:raw.shape[0], :raw.shape[1]] = raw != 0
        blocked = blocked.reshape(h, scale, w, scale).any(axis=(1, 3))
        self.blocked = blocked.copy()
        cells = math.ceil((radius + .05) / self.resolution)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if math.hypot(dx, dy) * self.resolution > radius + .05 + self.resolution / 2:
                    continue
                ys, ye = max(0, dy), min(h, h + dy)
                xs, xe = max(0, dx), min(w, w + dx)
                self.blocked[ys:ye, xs:xe] |= blocked[ys - dy:ye - dy, xs - dx:xe - dx]
        # Robot footprint may not extend beyond the represented map.
        self.blocked[:cells] = self.blocked[-cells:] = True
        self.blocked[:, :cells] = self.blocked[:, -cells:] = True

    def cell(self, point):
        return tuple(math.floor((point[i] - self.origin[i]) / self.resolution) for i in (0, 1))

    def point(self, cell):
        return [self.origin[i] + (cell[i] + .5) * self.resolution for i in (0, 1)]

    def free(self, cell):
        x, y = cell
        return 0 <= y < self.blocked.shape[0] and 0 <= x < self.blocked.shape[1] and not self.blocked[y, x]

    def path(self, start, goal):
        a, b = self.cell(start), self.cell(goal)
        if not self.free(a) or not self.free(b):
            return []
        queue = [(math.dist(a, b), 0., a)]
        costs, previous = {a: 0.}, {}
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            if current == b:
                route = [b]
                while route[-1] != a:
                    route.append(previous[route[-1]])
                return [list(start[:2]), *[self.point(c) for c in reversed(route)], list(goal[:2])]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = current[0] + dx, current[1] + dy
                if not self.free(nxt) or (dx and dy and (not self.free((current[0] + dx, current[1])) or not self.free((current[0], current[1] + dy)))):
                    continue
                candidate = cost + math.hypot(dx, dy)
                if candidate < costs.get(nxt, math.inf):
                    costs[nxt], previous[nxt] = candidate, current
                    heapq.heappush(queue, (candidate + math.dist(nxt, b), candidate, nxt))
        return []


def nearest(jobs, position, destination, current=None, margin=.25):
    """Straight-line remaining distance; equal distances retain the current job."""
    ranked = []
    for job in jobs:
        if job['status'] != 'queued':
            continue
        try:
            distance = math.dist(position[:2], destination(job)[:2])
        except (ValueError, KeyError):
            continue
        ranked.append((distance, job['created'], job['id'], job))
    if not ranked:
        return None
    distance, _, _, job = min(ranked)
    if current is not None and distance + margin >= math.dist(position[:2], destination(current)[:2]):
        return None
    return job
