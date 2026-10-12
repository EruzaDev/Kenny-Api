"""Measured sensor projection. Missing samples remain invalid, never clear."""
import math
import numpy as np


def rotation(quaternion):
    x, y, z, w = quaternion
    q = np.asarray([x, y, z, w], dtype=float)
    length = np.linalg.norm(q)
    if not np.isfinite(q).all() or length < 1e-9:
        raise ValueError('Invalid sensor TF quaternion')
    x, y, z, w = q / length
    return np.array([[1 - 2 * (y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def lidar_features(ranges, angle_min, angle_increment, range_min, range_max, translation, quaternion, maximum):
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    points = points @ rotation(quaternion).T + translation
    angles = np.arctan2(points[:, 1], points[:, 0])
    bins = np.floor((angles + math.pi) * 72 / (2 * math.pi) + .5).astype(int) % 72
    output, seen = np.full(72, maximum), np.zeros(72, dtype=bool)
    np.minimum.at(output, bins, np.linalg.norm(points[:, :2], axis=1))
    seen[bins] = True
    return np.clip(output, 0, maximum), seen


def depth_features(points, translation, quaternion, robot):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
    radial = np.linalg.norm(points, axis=1)
    good = (radial >= robot.camera_min_range) & (radial <= robot.camera_range)
    too_close = radial < robot.camera_min_range
    base = points @ rotation(quaternion).T + translation
    azimuth = np.degrees(np.arctan2(base[:, 1], base[:, 0]))
    fov = robot.camera_hfov_deg
    bins = np.floor((azimuth + fov / 2) * 35 / fov + .5).astype(int)
    visible = (bins >= 0) & (bins < 36)
    output, valid = np.full(36, robot.camera_range), np.zeros(36, dtype=bool)
    valid[bins[visible & good]] = True
    valid[bins[visible & too_close]] = False
    obstacle = visible & good & (base[:, 2] > .015) & (base[:, 2] < robot.height + .05)
    horizontal = np.linalg.norm(base[:, :2], axis=1)
    np.minimum.at(output, bins[obstacle], horizontal[obstacle])
    # Actual measured ground/below-ground points, not missing depth pixels.
    floor = np.full(12, robot.camera_range)
    floor_valid = np.zeros(12, dtype=bool)
    floor_bins = np.floor((azimuth + fov / 2) * 11 / fov + .5).astype(int)
    ground = (floor_bins >= 0) & (floor_bins < 12) & good & (base[:, 2] <= .03)
    floor_valid[floor_bins[ground]] = True
    drops = ground & (base[:, 2] < -.05)
    np.minimum.at(floor, floor_bins[drops], horizontal[drops])
    return (np.clip(output, 0, robot.camera_range), valid), (floor, floor_valid)
