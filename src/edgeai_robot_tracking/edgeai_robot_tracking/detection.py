"""Conservative green seeds and bounded, floor-filtered depth components."""

from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera import transform_points


@dataclass
class DetectionConfig:
    hsv_lower: tuple = (35, 65, 35)
    hsv_upper: tuple = (90, 255, 255)
    min_green_area: int = 3
    morphology_size: int = 0
    max_candidates: int = 12
    min_seed_depths: int = 3
    seed_depth_band: float = 0.08
    min_seed_fraction: float = 0.35
    seed_min_height: float = 0.015
    seed_max_height: float = 0.15
    floor_height: float = 0.0
    floor_clearance: float = 0.012
    max_height: float = 0.38
    region_radius: float = 0.48
    cluster_stride: int = 2
    connect_distance: float = 0.065
    max_cluster_pixels: int = 16000
    min_cluster_points: int = 6
    min_visible_width: float = 0.025
    min_visible_height: float = 0.025
    max_visible_width: float = 0.58
    max_visible_diagonal: float = 0.75  # axis-aligned bounds expand when the target turns
    duplicate_overlap: float = 0.35


@dataclass
class Candidate:
    position: np.ndarray                 # visible surface, observer base coordinates
    points: np.ndarray
    pixels: np.ndarray
    bbox: tuple
    quality: float
    extent: np.ndarray
    members: set
    green_pixels: int | None = None      # inapplicable to geometry-only candidates
    depth_inlier_fraction: float | None = None
    metrics: dict = field(default_factory=dict)


def robust_seed(uv, depths, calibration, config):
    """Find the densest depth band, then use its median 3D point, never one pixel."""
    valid = np.isfinite(depths)
    uv, depths = uv[valid], depths[valid]
    if len(depths) < config.min_seed_depths:
        return None
    order = np.argsort(depths)
    sorted_depths = depths[order]
    ends = np.searchsorted(sorted_depths, sorted_depths + config.seed_depth_band,
                           side='right')
    start = int(np.argmax(ends - np.arange(len(ends))))
    indices = order[start:ends[start]]
    fraction = len(indices) / len(depths)
    if len(indices) < config.min_seed_depths or fraction < config.min_seed_fraction:
        return None
    points = calibration.unproject(uv[indices], depths[indices])
    return np.median(points, axis=0), fraction, len(indices)


def connected_cluster(points, allowed, start, max_distance, limit):
    """4-connected pixels with a metric edge bound; exceeding the cap rejects it."""
    height, width = allowed.shape
    visited = np.zeros_like(allowed)
    stack = [start]
    visited[start] = True
    members = []
    squared_limit = max_distance ** 2
    while stack:
        row, col = stack.pop()
        members.append((row, col))
        if len(members) > limit:
            return None
        for nr, nc in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if visited[nr, nc] or not allowed[nr, nc]:
                continue
            delta = points[nr, nc] - points[row, col]
            if np.dot(delta, delta) <= squared_limit:
                visited[nr, nc] = True
                stack.append((nr, nc))
    return np.asarray(members, dtype=int)


def detect(bgr, depth, calibration, camera_to_base, config):
    """Return merged surface candidates, full-resolution mask and rejection counts."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(config.hsv_lower), np.array(config.hsv_upper))
    if config.morphology_size > 1:
        kernel = np.ones((config.morphology_size, config.morphology_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    ids = sorted((i for i in range(1, count) if stats[i, 4] >= config.min_green_area),
                 key=lambda i: stats[i, 4], reverse=True)
    rejected = {'small_color_regions': count - 1 - len(ids)}
    if len(ids) > config.max_candidates:
        # Do not hide competing targets by silently keeping only the biggest seeds.
        rejected['too_many_seeds'] = len(ids)
        return [], mask, rejected
    stride = config.cluster_stride
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    pixels = np.stack((u, v), axis=-1)
    camera_points = calibration.unproject(pixels.reshape(-1, 2), depth[::stride, ::stride].ravel())
    points = transform_points(camera_points, camera_to_base).reshape(*u.shape, 3)
    floor = config.floor_height
    above_floor = (np.isfinite(points).all(axis=2)
                   & (points[:, :, 2] >= floor + config.floor_clearance)
                   & (points[:, :, 2] <= floor + config.max_height))
    candidates = []

    def reject(reason):
        rejected[reason] = rejected.get(reason, 0) + 1

    for label in ids:
        sv, su = np.nonzero(labels == label)
        result = robust_seed(np.column_stack((su, sv)), depth[sv, su], calibration, config)
        if result is None:
            reject('invalid_seed_depth')
            continue
        camera_seed, fraction, support = result
        seed = transform_points(camera_seed, camera_to_base)
        if not floor + config.seed_min_height <= seed[2] <= floor + config.seed_max_height:
            reject('seed_height')
            continue
        distance = np.sum((points - seed) ** 2, axis=2)
        allowed = above_floor & (distance <= config.region_radius ** 2)
        if not allowed.any():
            reject('no_nearby_points')
            continue
        nearest = np.unravel_index(np.argmin(np.where(allowed, distance, np.inf)), u.shape)
        if distance[nearest] > config.connect_distance ** 2:
            reject('seed_disconnected')
            continue
        indices = connected_cluster(points, allowed, nearest, config.connect_distance,
                                    config.max_cluster_pixels)
        if indices is None or len(indices) < config.min_cluster_points:
            reject('cluster_size')
            continue
        rows, cols = indices.T
        cluster = points[rows, cols]
        extent = np.ptp(cluster, axis=0)
        if np.max(cluster[:, 2]) >= floor + config.max_height - config.connect_distance:
            reject('height_boundary')
            continue
        if (max(extent[:2]) > config.max_visible_width
                or np.linalg.norm(extent[:2]) > config.max_visible_diagonal
                or max(extent[:2]) < config.min_visible_width
                or extent[2] < config.min_visible_height):
            reject('visible_dimensions')
            continue
        # Hitting the search boundary indicates truncated geometry, e.g. a green wall.
        if np.max(np.linalg.norm(cluster - seed, axis=1)) > config.region_radius * 0.97:
            reject('region_boundary')
            continue
        cluster_pixels = pixels[rows, cols]
        low, high = np.min(cluster_pixels, axis=0), np.max(cluster_pixels, axis=0)
        quality = float(min(1., support / 12) * 0.35 + fraction * 0.35
                        + min(1., len(cluster) / 60) * 0.30)
        candidate = Candidate(
            position=np.mean(cluster, axis=0), points=cluster, pixels=cluster_pixels,
            bbox=(*low, *high), quality=quality, green_pixels=support,
            depth_inlier_fraction=fraction, extent=extent,
            members=set((rows * points.shape[1] + cols).tolist()),
        )
        duplicate = next((index for index, old in enumerate(candidates)
                          if len(old.members & candidate.members)
                          / min(len(old.members), len(candidate.members))
                          >= config.duplicate_overlap), None)
        if duplicate is None:
            candidates.append(candidate)
        elif candidate.quality > candidates[duplicate].quality:
            candidates[duplicate] = candidate
    return candidates, mask, rejected
