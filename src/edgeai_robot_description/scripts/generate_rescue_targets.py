#!/usr/bin/env python3
"""Generate physical rescue stands using edgeai_aruco's marker geometry."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from edgeai_aruco.marker_model import add_visual, generate_model


def generate_targets(config_path, models_path):
    config = yaml.safe_load(Path(config_path).read_text())
    ids = [config['targets'][name]['id'] for name in ('S1', 'C1')]
    if len(set(ids)) != 2:
        raise ValueError('S1 and C1 must have distinct ArUco IDs')
    for name in ('S1', 'C1'):
        directory = Path(models_path) / f'rescue_target_{name}'
        generate_model(directory, config['dictionary'], config['targets'][name]['id'],
                       config['marker_size'])
        tree = ET.parse(directory / 'model.sdf')
        model = tree.getroot().find('model')
        marker = model.find('link')
        # Native panel +Z becomes world -Y; image right stays world +X.
        ET.SubElement(marker, 'pose').text = '0 0 0.20 1.570796326795 0 0'
        board = float(marker.findtext('visual/geometry/box/size').split()[0])
        if board >= 0.4:
            raise ValueError('Marker and quiet zone must fit above the floor at a 0.20 m centre')
        stand = ET.SubElement(model, 'link', name='stand')
        # Backplate is behind the printed face; stem stops at its lower edge.
        stem_height = 0.20 - board / 2
        boxes = (
            ('backplate', (0, 0.006, 0.20), (board, 0.010, board)),
            ('stem', (0, 0.012, stem_height / 2), (0.025, 0.025, stem_height)),
            ('base', (0, 0, 0.01), (0.14, 0.14, 0.02)),
        )
        for part, (x, y, z), (sx, sy, sz) in boxes:
            add_visual(stand, part, x, y, z, sx, sy, sz, '0.35 0.35 0.35 1')
            collision = ET.SubElement(stand, 'collision', name=f'{part}_collision')
            ET.SubElement(collision, 'pose').text = f'{x:.12g} {y:.12g} {z:.12g} 0 0 0'
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = f'{sx:.12g} {sy:.12g} {sz:.12g}'
        ET.indent(tree, space='  ')
        tree.write(directory / 'model.sdf', encoding='utf-8', xml_declaration=True)
        metadata = ET.parse(directory / 'model.config')
        metadata.getroot().find('description').text = (
            f"Rescue stand {name}: {config['dictionary']} ID {config['targets'][name]['id']}; "
            f"{config['marker_size']:g} m black square, physical backplate, stem and base")
        ET.indent(metadata, space='  ')
        metadata.write(directory / 'model.config', encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--models-dir', type=Path, required=True)
    args = parser.parse_args()
    generate_targets(args.config, args.models_dir)


if __name__ == '__main__':
    main()
