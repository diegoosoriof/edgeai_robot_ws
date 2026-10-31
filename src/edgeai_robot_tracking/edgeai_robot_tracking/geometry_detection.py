"""Colour-independent native-depth components and conservative surface validation.

The metric four-neighbour graph is embedded in a doubled image grid, allowing
OpenCV to label it once in linear time. Height is validated AFTER clustering:
pre-crop tall surfaces cannot be reduced to a plausible low obstacle.
"""

from dataclasses import dataclass, asdict

import cv2
import numpy as np

from .camera import transform_points
from .detection import Candidate


@dataclass
class GeometryConfig:
    # Broad acquisition volume in observer base coordinates, independent of track.
    min_forward: float = 0.25
    max_forward: float = 5.0
    max_lateral: float = 3.0
    floor_height: float = 0.0
    floor_clearance: float = 0.012
    # Conservative box around this URDF's chassis, wheels and upper sensors.
    self_min_forward: float = -0.24
    self_max_forward: float = 0.23
    self_half_width: float = 0.19
    self_height: float = 0.23
    cluster_stride: int = 2
    connect_distance: float = 0.065
    max_sampled_points: int = 100000
    min_component_points: int = 6
    max_component_points: int = 16000
    max_components: int = 128
    boundary_margin: float = 0.02
    min_top_height: float = 0.04
    max_height: float = 0.23
    max_bottom_height: float = 0.11
    min_visible_height: float = 0.025
    min_visible_span: float = 0.025
    max_footprint_length: float = 0.55
    max_footprint_width: float = 0.40
    max_horizontal_diagonal: float = 0.65
    chassis_band_min: float = 0.025
    chassis_band_max: float = 0.12
    min_chassis_fraction: float = 0.25
    support_saturation: int = 60
    min_quality: float = 0.45

    def validate(self):
        for name, value in asdict(self).items():
            if not np.isfinite(value):
                raise ValueError(f'geometry.{name} must be finite')
        for name in ('cluster_stride', 'max_sampled_points', 'min_component_points',
                     'max_component_points', 'max_components', 'support_saturation'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 1:
                raise ValueError(f'geometry.{name} must be a positive integer')
        for name in ('max_lateral', 'self_half_width', 'self_height', 'connect_distance',
                     'min_top_height', 'max_height', 'max_bottom_height',
                     'min_visible_height', 'min_visible_span', 'max_footprint_length',
                     'max_footprint_width', 'max_horizontal_diagonal'):
            if getattr(self, name) <= 0:
                raise ValueError(f'geometry.{name} must be positive')
        if (not 0 <= self.min_forward < self.max_forward
                or self.self_min_forward >= self.self_max_forward
                or not 0 <= self.floor_clearance < self.min_top_height <= self.max_height
                or not 0 <= self.boundary_margin < min(
                    (self.max_forward - self.min_forward) / 2, self.max_lateral)
                or not 0 <= self.chassis_band_min < self.chassis_band_max <= self.max_height
                or not 0 <= self.min_chassis_fraction <= 1
                or not 0 <= self.min_quality <= 1
                or self.min_component_points > self.max_component_points
                or self.min_visible_span > self.max_footprint_length
                or self.min_visible_height > self.max_height
                or self.max_bottom_height > self.max_height
                or self.max_footprint_width > self.max_footprint_length):
            raise ValueError('Invalid geometry search, size or quality bounds')


def metric_labels(points, allowed, connect_distance):
    """Label each sampled point once, with explicit metric edges, not colour edges."""
    height, width = allowed.shape
    graph = np.zeros((2 * height - 1, 2 * width - 1), np.uint8)
    graph[::2, ::2] = allowed
    squared_limit = connect_distance ** 2
    horizontal = np.sum(np.diff(points, axis=1) ** 2, axis=2) <= squared_limit
    vertical = np.sum(np.diff(points, axis=0) ** 2, axis=2) <= squared_limit
    graph[::2, 1::2] = allowed[:, :-1] & allowed[:, 1:] & horizontal
    graph[1::2, ::2] = allowed[:-1] & allowed[1:] & vertical
    _, labels = cv2.connectedComponents(graph, connectivity=4)
    return labels[::2, ::2]


def detect_geometry(depth, calibration, camera_to_base, config):
    """Return (common candidates, None mask, rejection counts); RGB is not an input.

    Depth must already be validated metres on its NATIVE pixel grid. Components
    include all above-floor heights; no maximum-height crop precedes labelling.
    Disconnected surfaces are deliberately not grouped across unsupported gaps.
    """
    if depth.shape != (calibration.height, calibration.width):
        raise ValueError('Native depth/calibration dimensions differ')
    stride = config.cluster_stride
    rows = (depth.shape[0] + stride - 1) // stride
    cols = (depth.shape[1] + stride - 1) // stride
    rejected = {}

    def reject(reason, count=1):
        rejected[reason] = rejected.get(reason, 0) + int(count)

    if rows * cols > config.max_sampled_points:
        reject('sample_limit')
        return [], None, rejected
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    pixels = np.stack((u, v), axis=-1)
    points = calibration.unproject(pixels.reshape(-1, 2), depth[::stride, ::stride].ravel())
    points = transform_points(points, camera_to_base).reshape(rows, cols, 3)
    finite = np.isfinite(points).all(axis=2)
    z = points[:, :, 2] - config.floor_height
    above_floor = finite & (z >= config.floor_clearance)
    self_mask = (above_floor
                 & (points[:, :, 0] >= config.self_min_forward)
                 & (points[:, :, 0] <= config.self_max_forward)
                 & (np.abs(points[:, :, 1]) <= config.self_half_width)
                 & (z <= config.self_height))
    search = ((points[:, :, 0] >= config.min_forward)
              & (points[:, :, 0] <= config.max_forward)
              & (np.abs(points[:, :, 1]) <= config.max_lateral))
    allowed = above_floor & ~self_mask & search
    reject('floor_points', np.count_nonzero(finite & ~above_floor))
    reject('self_points', np.count_nonzero(self_mask))
    reject('outside_search_points', np.count_nonzero(above_floor & ~self_mask & ~search))
    labels = metric_labels(points, allowed, config.connect_distance)

    # Keep pre-crop geometry: flag metric-connected neighbours removed by the
    # horizontal search volume or self box. Cropped large obstacles are rejected.
    truncated = np.zeros_like(allowed)
    for axis in (0, 1):
        left = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        right = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        close = np.sum((points[left] - points[right]) ** 2, axis=2) <= (
            config.connect_distance ** 2)
        truncated[left] |= allowed[left] & above_floor[right] & ~allowed[right] & close
        truncated[right] |= allowed[right] & above_floor[left] & ~allowed[left] & close

    flat = np.flatnonzero(allowed)
    if not len(flat):
        return [], None, rejected
    point_labels = labels.ravel()[flat]
    counts = np.bincount(point_labels)
    eligible = np.flatnonzero(counts >= config.min_component_points)
    reject('small_components', np.count_nonzero(
        (counts > 0) & (counts < config.min_component_points)))
    if len(eligible) > config.max_components:
        reject('component_limit', len(eligible))
        return [], None, rejected
    # Sort once; no repeated whole-cloud masks/flood fills per candidate.
    supported = counts[point_labels] >= config.min_component_points
    flat, point_labels = flat[supported], point_labels[supported]
    order = np.argsort(point_labels, kind='stable')
    groups = np.split(flat[order], np.cumsum(counts[eligible])[:-1]) if len(flat) else []
    candidates = []
    flat_points, flat_pixels = points.reshape(-1, 3), pixels.reshape(-1, 2)
    for members in groups:
        if len(members) > config.max_component_points:
            reject('component_point_limit')
            continue
        cluster, uv = flat_points[members], flat_pixels[members]
        low, high = np.min(cluster, axis=0), np.max(cluster, axis=0)
        bottom, top = low[2] - config.floor_height, high[2] - config.floor_height
        margin = config.boundary_margin
        if (truncated.ravel()[members].any()
                or low[0] <= config.min_forward + margin
                or high[0] >= config.max_forward - margin
                or max(abs(low[1]), abs(high[1])) >= config.max_lateral - margin
                or np.any(uv[:, 0] == 0) or np.any(uv[:, 1] == 0)
                or np.any(uv[:, 0] >= depth.shape[1] - stride)
                or np.any(uv[:, 1] >= depth.shape[0] - stride)):
            reject('search_or_image_boundary')
            continue
        if top > config.max_height:
            reject('tall_component')
            continue
        if (top < config.min_top_height or bottom > config.max_bottom_height
                or top - bottom < config.min_visible_height):
            reject('height_dimensions')
            continue
        # Oriented horizontal rectangle avoids fixed base-x/base-y length tests.
        rectangle = cv2.minAreaRect(cluster[:, :2].astype(np.float32))
        length, width = sorted(rectangle[1], reverse=True)
        diagonal = float(np.hypot(length, width))
        if (length < config.min_visible_span or length > config.max_footprint_length
                or width > config.max_footprint_width
                or diagonal > config.max_horizontal_diagonal):
            reject('footprint_dimensions')
            continue
        heights = cluster[:, 2] - config.floor_height
        chassis_fraction = float(np.mean((heights >= config.chassis_band_min)
                                         & (heights <= config.chassis_band_max)))
        if chassis_fraction < config.min_chassis_fraction:
            reject('chassis_support')
            continue
        quality = float(0.30 * min(1., len(cluster) / config.support_saturation)
                        + 0.25 * chassis_fraction
                        + 0.25 * min(1., (top - bottom) / 0.07)
                        + 0.20 * min(1., length / 0.265))
        if quality < config.min_quality:
            reject('low_quality')
            continue
        pixel_low, pixel_high = np.min(uv, axis=0), np.max(uv, axis=0)
        candidates.append(Candidate(
            position=np.mean(cluster, axis=0), points=cluster, pixels=uv,
            bbox=(*pixel_low, *pixel_high), quality=quality, extent=high - low,
            members=set(members.tolist()), metrics={
                'point_count': len(cluster), 'bottom_m': float(bottom), 'top_m': float(top),
                'footprint_length_m': float(length), 'footprint_width_m': float(width),
                'chassis_fraction': chassis_fraction,
            }))
    return candidates, None, rejected
