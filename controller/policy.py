"""Resident PPO and the checkpoint's exact live observation layout."""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np

FEATURES = [('lidar_range', 72), ('lidar_valid', 72), ('depth_range', 36), ('depth_valid', 36),
            ('floor_drop_range', 12), ('floor_valid', 12), ('downward_hazard', 3), ('downward_valid', 3),
            ('route_points_xy', 6), ('route_valid', 3), ('goal_relative_xy', 2), ('measured_velocity', 2),
            ('previous_requested_action', 2), ('previous_executed_action', 2),
            ('localization_health', 3), ('sensor_age_and_guard', 3)]


def local(points, pose):
    delta = np.asarray(points) - pose[:2]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return delta @ np.array([[c, -s], [s, c]])


def lookahead(route, pose):
    if not len(route):
        return np.zeros((3, 2)), np.zeros(3)
    route = np.asarray(route)
    if len(route) == 1:
        return local(np.repeat(route, 3, axis=0), pose), np.ones(3)
    segments = np.diff(route, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    fractions = np.divide(np.sum((pose[:2] - route[:-1]) * segments, axis=1), lengths ** 2,
                          out=np.zeros_like(lengths), where=lengths > 1e-9)
    projections = route[:-1] + np.clip(fractions, 0, 1)[:, None] * segments
    index = int(np.argmin(np.linalg.norm(projections - pose[:2], axis=1)))
    forward = np.vstack((projections[index], route[index + 1:]))
    cumulative = np.r_[0., np.cumsum(np.linalg.norm(np.diff(forward, axis=0), axis=1))]
    points = np.column_stack([np.interp([.5, 1., 2.], cumulative, forward[:, axis]) for axis in (0, 1)])
    return local(points, pose), np.ones(3)


class ResidentPolicy:
    def __init__(self, checkpoint):
        from kenny_rl.config import load_config
        from kenny_rl.env import KennyEnv
        from stable_baselines3 import PPO
        import torch
        torch.set_num_threads(1)
        checkpoint = Path(checkpoint).resolve()
        source = checkpoint.parent.parent if checkpoint.parent.name == 'checkpoints' else checkpoint.parent
        self.robot, self.config, training = load_config(source / 'config.json')
        self.contract = json.loads((source / 'contract.json').read_text())
        env = KennyEnv(self.robot, self.config)
        expected = json.loads(json.dumps(env.contract()))
        env.close()
        if self.contract != expected or self.contract['features'] != [list(v) for v in FEATURES]:
            raise ValueError('Checkpoint contract is incompatible with the live feature adapter')
        self.model = PPO.load(checkpoint, device='cpu', custom_objects={
            'lr_schedule': lambda _: 0., 'learning_rate': 0.,
            'clip_range': lambda _: float(training.get('clip_range', .2)), 'clip_range_vf': None})
        self.history = deque(maxlen=self.config.history)
        self.requested = self.executed = np.array([-1., 0.], dtype=np.float32)
        # Allocate and warm inference once. This is not a sensor observation or command.
        self.model.predict(np.zeros(self.contract['frame_size'] * self.config.history, dtype=np.float32), deterministic=True)

    def destination_changed(self):
        # Clear old goal-relative features, while keeping sensors/map/model alive.
        self.history.clear()
        self.requested = self.executed = np.array([-1., 0.], dtype=np.float32)

    def predict(self, sensors, pose, goal, route, dimensions, velocity, uncertainty, marker_age, now, intervened):
        r = self.robot
        points, route_valid = lookahead(route, np.asarray(pose))
        values = [sensors['lidar'][0] / r.lidar_range, sensors['lidar'][1],
                  sensors['depth'][0] / r.camera_range, sensors['depth'][1],
                  sensors['floor'][0] / r.camera_range, sensors['floor'][1],
                  sensors['downward'][0], sensors['downward'][1], points.ravel() / 3., route_valid,
                  local([goal[:2]], np.asarray(pose))[0] / np.asarray(dimensions),
                  np.asarray(velocity) / [r.max_speed, r.max_turn_rate], self.requested, self.executed,
                  [uncertainty / .5, min(marker_age / 30., 1.), float(uncertainty < .35)],
                  [max(0., now - sensors['lidar'][2]), 0., float(intervened)]]
        frame = np.clip(np.concatenate([np.asarray(v).ravel() for v in values]), -1, 1).astype(np.float32)
        if frame.size != self.contract['frame_size'] or not np.isfinite(frame).all():
            raise ValueError('Live observation does not match checkpoint contract')
        if not self.history:
            self.history.extend(frame.copy() for _ in range(self.config.history))
        else:
            self.history.append(frame)
        action, _ = self.model.predict(np.concatenate(self.history), deterministic=True)
        if np.asarray(action).shape != (2,) or not np.isfinite(action).all():
            raise ValueError('Policy returned an invalid action')
        self.requested = np.clip(action, -1, 1)
        return [(self.requested[0] + 1) * r.max_speed / 2, self.requested[1] * r.max_turn_rate]

    def record_executed(self, command):
        self.executed = np.array([2 * command[0] / self.robot.max_speed - 1,
                                  command[1] / self.robot.max_turn_rate], dtype=np.float32)


def guard(command, sensors, now, radius=.163, speed_limit=.1, braking=.35, uncertainty=.05):
    v, w = float(command[0]), float(command[1])
    if not np.isfinite([v, w]).all():
        return [0., 0.], 'invalid_policy_command'
    for name in ('lidar', 'depth', 'floor', 'downward'):
        if name not in sensors or now - sensors[name][2] > .5:
            return [0., 0.], f'{name}_stale'
    lidar, depth, floor, downward = (sensors[name] for name in ('lidar', 'depth', 'floor', 'downward'))
    if not np.asarray(downward[1]).all() or np.asarray(downward[0]).any():
        return [0., 0.], 'downward_invalid_or_hazard'
    if np.count_nonzero(lidar[1]) < 65 or not np.asarray(depth[1]).all() or not np.asarray(floor[1]).all():
        return [0., 0.], 'sensor_coverage_incomplete'
    v, w = min(max(v, 0.), speed_limit), max(-.6, min(.6, w))
    stopping = radius + uncertainty + .1 + v * .3 + v * v / (2 * braking)
    angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
    relevant = np.abs(angles) < math.pi / 3 if abs(w) < .1 else np.ones(72, dtype=bool)
    if not np.asarray(lidar[1])[relevant].all() or np.any(np.asarray(lidar[0])[relevant] < stopping) or np.any(np.asarray(depth[0]) < stopping):
        return [0., 0.], 'observed_obstacle'
    if np.any(np.asarray(floor[0]) < stopping + .15):
        return [0., 0.], 'observed_floor_drop'
    return [v, w], ''
