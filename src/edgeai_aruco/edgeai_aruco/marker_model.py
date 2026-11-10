"""Generate camera-visible SDF geometry directly from OpenCV's marker bits."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from .detection import dictionary_from_name, positive_float


def add_visual(link, name, x, y, z, width, height, thickness, color):
    visual = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(visual, 'pose').text = f'{x:.12g} {y:.12g} {z:.12g} 0 0 0'
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    ET.SubElement(box, 'size').text = f'{width:.12g} {height:.12g} {thickness:.12g}'
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    material = ET.SubElement(visual, 'material')
    for component in ('ambient', 'diffuse', 'emissive'):
        ET.SubElement(material, component).text = color
    ET.SubElement(material, 'specular').text = '0 0 0 1'


def generate_model(directory, dictionary='DICT_4X4_50', marker_id=1, size=0.20):
    """XY panel; +X right, +Y up, +Z out of printed face; no collisions."""
    directory = Path(directory)
    marker_dictionary = dictionary_from_name(dictionary)
    size = positive_float(size, 'marker_size')
    if marker_id < 0 or marker_id >= len(marker_dictionary.bytesList):
        raise ValueError(f'Marker ID {marker_id} is outside {dictionary}')
    cells = marker_dictionary.markerSize + 2  # Includes the one-cell black border.
    bits = cv2.aruco.generateImageMarker(marker_dictionary, marker_id, cells)
    cell = size / cells
    sdf = ET.Element('sdf', version='1.9')
    sdf.append(ET.Comment(
        f' Generated with edgeai_aruco.generate_marker: {dictionary}, ID {marker_id}, '
        f'black square {size:g} m; one-cell white quiet zone. '))
    model = ET.SubElement(sdf, 'model', name=directory.name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='marker')
    # Backing front lies at z=0; black cells protrude 0.2 mm to avoid z-fighting.
    add_visual(link, 'white_backing', 0, 0, -0.0005,
               size + 2 * cell, size + 2 * cell, 0.001, '1 1 1 1')
    for row in range(cells):
        for column in range(cells):
            if bits[row, column] == 0:
                add_visual(link, f'black_{row}_{column}',
                           -size / 2 + (column + 0.5) * cell,
                           size / 2 - (row + 0.5) * cell, 0.0001,
                           cell, cell, 0.0002, '0 0 0 1')
    directory.mkdir(parents=True, exist_ok=True)
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(directory / 'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.Element('model')
    ET.SubElement(config, 'name').text = directory.name
    ET.SubElement(config, 'version').text = '1.0'
    ET.SubElement(config, 'sdf', version='1.9').text = 'model.sdf'
    ET.SubElement(config, 'description').text = (
        f'{dictionary} ID {marker_id}; {size:g} m black square, visual geometry only')
    ET.indent(config, space='  ')
    ET.ElementTree(config).write(directory / 'model.config', encoding='utf-8', xml_declaration=True)
    preview = np.pad(bits, 1, constant_values=255)
    preview = cv2.resize(preview, None, fx=60, fy=60, interpolation=cv2.INTER_NEAREST)
    if not cv2.imwrite(str(directory / 'marker.png'), preview):
        raise RuntimeError('Could not write marker preview')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='Gazebo model directory')
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--id', type=int, default=1, dest='marker_id')
    parser.add_argument('--size', type=float, default=0.20, help='Black square side in metres')
    arguments = parser.parse_args(args)
    generate_model(arguments.output, arguments.dictionary, arguments.marker_id, arguments.size)


if __name__ == '__main__':
    main()
