"""Prepare a Part 2 occupancy map from the reused static arena collision shapes.

This is a geometry-derived reference map, not a SLAM map or a Part 1 result.
"""

import argparse
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .robot_model import transform


def sdf_pose(element):
    values = list(map(float, (element.findtext('pose') or '0 0 0 0 0 0').split()))
    return transform(values[:3], values[3:])


def hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for sequence, result in ((points, lower), (reversed(points), upper)):
        for point in sequence:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 1e-12:
                result.pop()
            result.append(point)
    return np.array(lower[:-1]+upper[:-1])


def collision_polygons(world_file, models_dir):
    polygons = []
    def model_shapes(model, parent):
        base = parent @ sdf_pose(model)
        for link in model.findall('link'):
            link_frame = base @ sdf_pose(link)
            for collision in link.findall('collision'):
                box = collision.find('geometry/box')
                if box is None:
                    continue  # The ground plane is not an obstacle.
                half = np.fromstring(box.findtext('size'), sep=' ')/2
                frame = link_frame @ sdf_pose(collision)
                corners = np.array([(frame @ np.r_[half*np.array(signs), 1])[:3]
                                    for signs in product((-1, 1), repeat=3)])
                polygons.append((hull(corners[:, :2]), float(corners[:, 2].min()),
                                 float(corners[:, 2].max())))
        for include in model.findall('include'):
            included(include, base)

    def included(include, parent):
        uri = include.findtext('uri')
        if not uri.startswith('model://'):
            raise ValueError(f'Unsupported model URI {uri}')
        path = Path(models_dir)/uri.removeprefix('model://')/'model.sdf'
        model_shapes(ET.parse(path).getroot().find('model'), parent @ sdf_pose(include))

    world = ET.parse(world_file).getroot().find('world')
    for model in world.findall('model'):
        model_shapes(model, np.eye(4))
    for include in world.findall('include'):
        included(include, np.eye(4))
    return polygons


def generate(world_file, models_dir, output, resolution=.025):
    origin = np.array([-.25, -.25])
    count = round(5.5/resolution)
    rows, columns = np.indices((count, count))
    centers = np.stack([origin[0]+(columns+.5)*resolution,
                        origin[1]+(rows+.5)*resolution], axis=-1)
    occupied = np.any((centers < 0) | (centers > 5), axis=-1)
    for polygon, _, _ in collision_polygons(world_file, models_dir):
        inside = np.ones((count, count), dtype=bool)
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
            edge = end-start
            cross = edge[0]*(centers[:, :, 1]-start[1])-edge[1]*(centers[:, :, 0]-start[0])
            # Cell-center occupancy preserves the measured wall locations for
            # AMCL. Expanding every box by half a cell shrinks the entire arena.
            inside &= cross >= -1e-10
        occupied |= inside
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    image = np.where(occupied[::-1], 0, 254).astype('uint8')
    output.with_suffix('.pgm').write_bytes(f'P5\n{count} {count}\n255\n'.encode()+image.tobytes())
    metadata = {'image': output.with_suffix('.pgm').name, 'mode': 'trinary',
                'resolution': resolution, 'origin': [float(origin[0]), float(origin[1]), 0.],
                'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}
    output.with_suffix('.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    return np.where(occupied, 100, 0).astype('int8'), metadata


def main():
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('edgeai_rescue'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=share/'worlds/rescue_arena.sdf')
    parser.add_argument('--models-dir', type=Path, default=share/'models')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.world, args.models_dir, args.output)


if __name__ == '__main__':
    main()
