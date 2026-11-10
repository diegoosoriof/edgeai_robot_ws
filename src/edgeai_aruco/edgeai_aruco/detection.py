"""OpenCV detection and metric pose estimation, independent of ROS transport."""

from dataclasses import dataclass

import cv2
import numpy as np


def dictionary_from_name(name):
    if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
        raise ValueError(f'Unknown OpenCV ArUco dictionary: {name}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def positive_float(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and greater than zero')
    return value


def marker_points(size):
    """TL, TR, BR, BL; marker X right, Y up, Z out of its printed face."""
    half = positive_float(size, 'marker_size') / 2
    return np.array([
        [-half, half, 0], [half, half, 0],
        [half, -half, 0], [-half, -half, 0],
    ], dtype=np.float64)


@dataclass
class Calibration:
    matrix: np.ndarray
    distortion: np.ndarray
    width: int
    height: int

    @classmethod
    def from_values(cls, k, d, width, height, model):
        matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(d, dtype=np.float64)
        if (width <= 0 or height <= 0 or not np.isfinite(matrix).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1])
                or not np.allclose([matrix[0, 1], matrix[1, 0]], 0)):
            raise ValueError('CameraInfo has invalid or uncalibrated intrinsics')
        if model not in ('', 'plumb_bob', 'rational_polynomial'):
            raise ValueError(f'Unsupported distortion model: {model}')
        if (distortion.size not in (0, 4, 5, 8, 12, 14)
                or not np.isfinite(distortion).all()
                or (not model and np.any(distortion))):
            raise ValueError('CameraInfo has invalid distortion coefficients')
        return cls(matrix, distortion, width, height)


@dataclass
class Observation:
    marker_id: int
    size: float
    rvec: np.ndarray
    tvec: np.ndarray


def estimate_pose(corners, size, calibration, max_reprojection_error=3.0):
    points = marker_points(size)
    image_points = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(image_points).all():
        return None
    # Homography initialization handles planar squares, including the exactly
    # frontal case in OpenCV 4.12 (which has an IPPE rotation singularity).
    success, rvec, tvec = cv2.solvePnP(
        points, image_points, calibration.matrix, calibration.distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    if np.any((rotation @ points.T + tvec)[2] <= 0):
        return None
    projected, _ = cv2.projectPoints(
        points, rvec, tvec, calibration.matrix, calibration.distortion)
    error = np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points)**2, axis=1)))
    if not np.isfinite(error) or error > max_reprojection_error:
        return None
    return rvec, tvec


def rotation_quaternion(rvec):
    """Rodrigues vector to ROS quaternion (x, y, z, w), including rotations by pi."""
    vector = np.asarray(rvec, dtype=np.float64).reshape(3)
    angle = np.linalg.norm(vector)
    scale = 0.5 if angle < 1e-12 else np.sin(angle / 2) / angle
    return np.r_[vector * scale, np.cos(angle / 2)]


class Detector:
    def __init__(self, dictionary, marker_size, size_overrides=None, max_error=3.0):
        self.dictionary = dictionary_from_name(dictionary)
        self.marker_size = positive_float(marker_size, 'marker_size')
        self.size_overrides = size_overrides or {}
        self.max_error = positive_float(max_error, 'max_reprojection_error')
        for marker_id, size in self.size_overrides.items():
            if marker_id < 0 or marker_id >= len(self.dictionary.bytesList):
                raise ValueError(f'Marker ID {marker_id} is outside {dictionary}')
            positive_float(size, f'size of marker {marker_id}')
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, parameters)

    def detect(self, image, calibration=None):
        corners, ids, _ = self.detector.detectMarkers(image)
        observations = []
        if ids is not None and calibration is not None:
            for marker_corners, marker_id in zip(corners, ids.ravel()):
                marker_id = int(marker_id)
                size = self.size_overrides.get(marker_id, self.marker_size)
                pose = estimate_pose(marker_corners, size, calibration, self.max_error)
                if pose is not None:
                    observations.append(Observation(marker_id, size, *pose))
        return corners, ids, sorted(observations, key=lambda observation: observation.marker_id)
