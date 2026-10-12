"""Estimated map + authoritative SLAM observations, in a common map frame."""
from collections import deque
import copy
import math

import numpy as np

from app.environment import validate_environment

MAX_CELLS = 4_000_000


def transform(points, pose):
    points = np.asarray(points, dtype=float)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def inverse(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return [-c * pose[0] - s * pose[1], s * pose[0] - c * pose[1], -pose[2]]


def compose(a, b):
    p = transform([b[:2]], a)[0]
    return [float(p[0]), float(p[1]), a[2] + b[2]]


def bounds(grid):
    return transform([[0, 0], [grid['width'] * grid['resolution'], 0],
                      [0, grid['height'] * grid['resolution']],
                      [grid['width'] * grid['resolution'], grid['height'] * grid['resolution']]], grid['origin'])


def checked_grid(grid):
    w, h, r = grid['width'], grid['height'], grid['resolution']
    if type(w) is not int or type(h) is not int or w < 1 or h < 1 or w * h > MAX_CELLS:
        raise ValueError('Invalid SLAM grid size')
    if not math.isfinite(r) or not .005 <= r <= 1 or len(grid['origin']) != 3 or not np.isfinite(grid['origin']).all():
        raise ValueError('Invalid SLAM grid geometry')
    data = np.asarray(grid['data'])
    if data.size != w * h or not np.isfinite(data).all() or np.any(data != np.floor(data)) or np.any((data < -1) | (data > 100)):
        raise ValueError('Invalid SLAM occupancy values')
    return {**grid, 'data': data.astype(np.int16).reshape(h, w)}


def sample(grid, x, y):
    ox, oy, yaw = grid['origin']
    c, s = math.cos(yaw), math.sin(yaw)
    col = np.floor((c * (x - ox) + s * (y - oy)) / grid['resolution']).astype(int)
    row = np.floor((-s * (x - ox) + c * (y - oy)) / grid['resolution']).astype(int)
    inside = (col >= 0) & (row >= 0) & (col < grid['width']) & (row < grid['height'])
    values = np.full(x.shape, -1, dtype=np.int16)
    values[inside] = grid['data'][row[inside], col[inside]]
    return values


class AdaptiveMap:
    def __init__(self, environment):
        self.estimate = copy.deepcopy(validate_environment(environment))
        self.prior = checked_grid(self.estimate['grid'])
        self.markers = {m['id']: copy.deepcopy(m) for m in self.estimate['markers']}
        self.landmarks = {}
        self.slam = None
        self.alignment = [0., 0., 0.]
        self.revision = 0
        self.grid = None
        self.observed = None
        self.rebuild()

    def update_slam(self, grid, alignment):
        source = checked_grid(grid)
        source['origin'] = compose(alignment, source['origin'])
        self.slam, self.alignment = source, list(alignment)
        self.rebuild()

    def rebuild(self):
        corners = bounds(self.prior)
        if self.slam is not None:
            corners = np.vstack((corners, bounds(self.slam)))
        # Use the finest source resolution so live walls are not downsampled away.
        resolution = min(self.prior['resolution'], self.slam['resolution'] if self.slam else 1.)
        lower = np.floor(corners.min(axis=0) / resolution) * resolution
        upper = np.ceil(corners.max(axis=0) / resolution) * resolution
        w, h = np.rint((upper - lower) / resolution).astype(int)
        if w * h > MAX_CELLS:
            raise ValueError('Refined map exceeds four million cells')
        data = np.full((h, w), -1, dtype=np.int16)
        observed = np.zeros((h, w), dtype=bool)
        # Chunk coordinates to avoid full-size floating-point meshes on the Pi.
        for begin in range(0, h, 128):
            end = min(h, begin + 128)
            x, y = np.meshgrid(lower[0] + (np.arange(w) + .5) * resolution,
                               lower[1] + (np.arange(begin, end) + .5) * resolution)
            prior = sample(self.prior, x, y)
            for obstacle in self.estimate.get('obstacles', []):
                inside = (x >= obstacle['x']) & (x <= obstacle['x'] + obstacle['width']) & (y >= obstacle['y']) & (y <= obstacle['y'] + obstacle['depth'])
                prior[inside] = 100
            if self.slam is not None:
                measured = sample(self.slam, x, y)
                seen = measured >= 0
                prior[seen] = np.where(measured[seen] >= 65, 100, np.where(measured[seen] <= 19, 0, -1))
                observed[begin:end] = seen
            # No-go areas are semantic restrictions, not estimated walls.
            for obstacle in self.estimate.get('obstacles', []):
                if obstacle['kind'] == 'prohibited':
                    inside = (x >= obstacle['x']) & (x <= obstacle['x'] + obstacle['width']) & (y >= obstacle['y']) & (y <= obstacle['y'] + obstacle['depth'])
                    prior[inside] = 100
            data[begin:end] = prior
        self.grid = {'width': int(w), 'height': int(h), 'resolution': resolution,
                     'origin': [float(lower[0]), float(lower[1]), 0.], 'data': data.ravel().tolist()}
        self.observed = observed
        self.revision += 1

    def observe_marker(self, marker_id, pose_in_slam, timestamp):
        if marker_id not in self.markers or len(pose_in_slam) != 3 or not np.isfinite(pose_in_slam).all():
            return False
        samples = self.landmarks.setdefault(marker_id, deque(maxlen=15))
        if samples and timestamp <= samples[-1][0]:
            return False
        # Discard old samples so repeated observations can follow loop closure.
        while samples and timestamp - samples[0][0] > 30:
            samples.popleft()
        samples.append((timestamp, list(pose_in_slam)))
        if len(samples) < 3:
            return False
        poses = np.asarray([p for _, p in samples])
        center = np.median(poses[:, :2], axis=0)
        inliers = np.linalg.norm(poses[:, :2] - center, axis=1) <= .15
        if np.count_nonzero(inliers) < 3:
            return False
        angles = poses[inliers, 2]
        yaw = math.atan2(np.sin(angles).mean(), np.cos(angles).mean())
        if np.hypot(np.sin(angles).mean(), np.cos(angles).mean()) < .8:
            return False
        pose = compose(self.alignment, [*np.median(poses[inliers, :2], axis=0), yaw])
        self.markers[marker_id].update(x=pose[0], y=pose[1], yaw=pose[2])
        self.revision += 1
        return True

    def destination(self, job):
        if job['dictionary'] != self.estimate['dictionary']:
            raise ValueError('Call dictionary differs from robot map')
        marker = self.markers.get(job['marker_id'])
        if marker is None:
            raise ValueError('Call marker is missing from robot map')
        offset = .5 if marker['mount'] == 'wall' else 0.
        return [marker['x'] + math.cos(marker['yaw']) * offset,
                marker['y'] + math.sin(marker['yaw']) * offset,
                marker['yaw'] + (math.pi if marker['mount'] == 'wall' else 0.)]

    def export(self, pose=None):
        result = {**copy.deepcopy(self.estimate), 'grid': copy.deepcopy(self.grid),
                  'markers': list(copy.deepcopy(self.markers).values()),
                  # Do not re-stamp obsolete editor walls over corrected occupancy.
                  'obstacles': [copy.deepcopy(o) for o in self.estimate.get('obstacles', []) if o['kind'] == 'prohibited'],
                  'mapping': {'revision': self.revision, 'observed_cells': int(self.observed.sum()),
                              'prior_cells': int(self.observed.size - self.observed.sum()),
                              'alignment_map_from_slam': self.alignment}}
        if pose is not None:
            result['start'] = list(pose[:2])
        marker_id = result.get('destination_marker_id')
        if marker_id in self.markers:
            result['goal'] = self.destination({'marker_id': marker_id, 'dictionary': result['dictionary']})[:2]
        return result
