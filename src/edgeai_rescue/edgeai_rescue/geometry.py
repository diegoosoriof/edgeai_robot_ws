"""Planar geometry shared by goal validation, peer obstacles and map checks."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    @property
    def finite(self):
        return all(math.isfinite(value) for value in (self.x, self.y, self.yaw))


def shift_pose(values, forward):
    x, y, angle = values
    return [x+forward*math.cos(angle), y+forward*math.sin(angle), angle]


def polygon_at(pose, footprint, padding=0.0):
    vertices = np.asarray(footprint, dtype=float).copy()
    vertices += np.sign(vertices) * padding
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return vertices @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]


def polygons_overlap(first, second, clearance=0.0):
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    for polygon in (first, second):
        for edge in np.roll(polygon, -1, axis=0) - polygon:
            normal = np.array([-edge[1], edge[0]])
            normal /= np.linalg.norm(normal)
            a, b = first @ normal, second @ normal
            if a.max() + clearance < b.min() or b.max() + clearance < a.min():
                return False
    return True


def perimeter_points(vertices, spacing=0.025):
    vertices = np.asarray(vertices)
    points = []
    for start, end in zip(vertices, np.roll(vertices, -1, axis=0)):
        for fraction in np.linspace(0, 1, max(2, math.ceil(np.linalg.norm(end-start)/spacing)), endpoint=False):
            points.append(start + fraction * (end-start))
    return np.asarray(points)


class OccupancyGrid:
    def __init__(self, data, resolution, origin=(0, 0, 0), blocked_at=65, unknown=-1):
        self.data = np.asarray(data)
        if self.data.ndim != 2 or resolution <= 0:
            raise ValueError('A 2D grid and positive resolution are required')
        self.resolution = float(resolution)
        self.origin = Pose2D(*origin)
        self.blocked_at, self.unknown = blocked_at, unknown

    def grid_points(self, world_points):
        xy = np.asarray(world_points) - [self.origin.x, self.origin.y]
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        return xy @ np.array([[c, -s], [s, c]]) / self.resolution

    def pose_is_free(self, pose, footprint, padding=0.0):
        if not pose.finite:
            return False
        polygon = self.grid_points(polygon_at(pose, footprint, padding))
        lo = np.floor(polygon.min(axis=0)).astype(int)
        hi = np.floor(polygon.max(axis=0)).astype(int)
        height, width = self.data.shape
        if np.any(lo < 0) or hi[0] >= width or hi[1] >= height:
            return False
        for row in range(lo[1], hi[1]+1):
            for column in range(lo[0], hi[0]+1):
                value = self.data[row, column]
                if value == self.unknown or value >= self.blocked_at:
                    cell = np.array([[column, row], [column+1, row],
                                     [column+1, row+1], [column, row+1]])
                    if polygons_overlap(polygon, cell):
                        return False
        return True


def validate_goals(target, grid, footprint):
    goals = {robot: Pose2D(*target['goals'][robot]) for robot in target['robots']}
    center = np.array(target['center'])
    for robot, pose in goals.items():
        if not pose.finite or np.linalg.norm([pose.x-center[0], pose.y-center[1]]) > target['radius']-.02:
            raise ValueError(f'{robot} goal is outside the rescue radius/margin')
        if not grid.pose_is_free(pose, footprint, padding=.015):
            raise ValueError(f'{robot} goal footprint collides with the map')
    values = list(goals.values())
    for index, first in enumerate(values):
        for second in values[index+1:]:
            if polygons_overlap(polygon_at(first, footprint), polygon_at(second, footprint), clearance=.08):
                raise ValueError('Cooperative goal footprints do not have enough separation')
    return goals
