"""Depth units, pinhole geometry and optional depth-to-RGB registration."""

from dataclasses import dataclass

import cv2
import numpy as np


def depth_metres(image, encoding, min_depth=0.1, max_depth=9.5, uint16_scale=0.001):
    """32FC1 is metres; 16UC1 uses the explicitly configured scale (normally mm)."""
    if encoding == '32FC1':
        depth = np.asarray(image, dtype=np.float32).copy()
    elif encoding == '16UC1':
        depth = np.asarray(image, dtype=np.float32) * uint16_scale
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}; use 32FC1 or 16UC1')
    if depth.ndim != 2:
        raise ValueError('Depth must be a single-channel image')
    depth[~np.isfinite(depth) | (depth < min_depth) | (depth > max_depth)] = np.nan
    return depth


@dataclass
class Calibration:
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray

    def __post_init__(self):
        self.k = np.asarray(self.k, dtype=float).reshape(3, 3)
        self.d = np.asarray(self.d, dtype=float)
        if (self.width <= 0 or self.height <= 0 or not np.isfinite(self.k).all()
                or not np.isfinite(self.d).all() or self.k[0, 0] <= 0
                or self.k[1, 1] <= 0 or not np.allclose(self.k[2], [0, 0, 1])
                or abs(self.k[0, 1]) > 1e-6 or abs(self.k[1, 0]) > 1e-6):
            raise ValueError('Invalid or unsupported CameraInfo intrinsics')
        if self.d.size not in (0, 4, 5, 8, 12, 14):
            raise ValueError('Unsupported distortion coefficient count')

    def unproject(self, uv, depths):
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        if self.d.size and np.any(self.d):
            rays = cv2.undistortPoints(uv[:, None], self.k, self.d)[:, 0]
        else:
            rays = (uv - self.k[:2, 2]) / [self.k[0, 0], self.k[1, 1]]
        return np.column_stack((rays, np.ones(len(rays)))) * np.asarray(depths)[:, None]

    def project(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), self.k, self.d)
        return pixels[:, 0]


def transform_points(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def transform_matrix(transform):
    """Convert geometry_msgs/Transform into a homogeneous rigid transform."""
    q = transform.rotation
    x, y, z, w = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = np.array([x, y, z, w]) / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Invalid TF translation')
    return matrix


def register_depth(depth, depth_calibration, rgb_calibration, depth_to_rgb):
    """Project depth into RGB with a nearest-surface z-buffer; leave holes invalid.

    Both calibration frames must use ROS optical axes (right, down, forward).
    No interpolation across occlusion boundaries or calibration-free resizing.
    """
    v, u = np.nonzero(np.isfinite(depth))
    registered = np.full((rgb_calibration.height, rgb_calibration.width), np.inf,
                         dtype=np.float32)
    if not len(u):
        registered[:] = np.nan
        return registered
    points = depth_calibration.unproject(np.column_stack((u, v)), depth[v, u])
    points = transform_points(points, depth_to_rgb)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
    if len(points):
        pixels = np.rint(rgb_calibration.project(points)).astype(np.int64)
        valid = ((pixels[:, 0] >= 0) & (pixels[:, 0] < rgb_calibration.width)
                 & (pixels[:, 1] >= 0) & (pixels[:, 1] < rgb_calibration.height))
        u, v = pixels[valid].T
        np.minimum.at(registered, (v, u), points[valid, 2])
    registered[~np.isfinite(registered)] = np.nan
    return registered
