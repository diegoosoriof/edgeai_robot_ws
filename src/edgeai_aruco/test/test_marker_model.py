from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest

from edgeai_aruco.detection import Detector, dictionary_from_name
from edgeai_aruco.marker_model import generate_model


def test_generated_geometry_matches_marker(tmp_path):
    directory = tmp_path / 'test_marker'
    generate_model(directory, 'DICT_4X4_50', 7, 0.3)
    model = ET.parse(directory / 'model.sdf')
    assert model.findall('.//collision') == []
    visuals = model.findall('.//visual')
    assert visuals[0].findtext('geometry/box/size').split()[:2] == ['0.4', '0.4']
    # Reconstruct the camera-facing pattern from the actual SDF cell positions.
    bits = np.full((6, 6), 255, dtype=np.uint8)
    for visual in visuals[1:]:
        x, y, z, *_ = map(float, visual.findtext('pose').split())
        assert z > 0  # Separated from the white backing.
        column = round((x + 0.15) / 0.05 - 0.5)
        row = round((0.15 - y) / 0.05 - 0.5)
        bits[row, column] = 0
    expected = cv2.aruco.generateImageMarker(dictionary_from_name('DICT_4X4_50'), 7, 6)
    np.testing.assert_array_equal(bits, expected)
    image = cv2.imread(str(directory / 'marker.png'))
    assert Detector('DICT_4X4_50', 0.3).detect(image)[1].ravel().tolist() == [7]


def test_checked_in_model_is_reproducible(tmp_path):
    root = Path(__file__).resolve().parents[2]
    checked_in = root / 'edgeai_robot_description/models/edgeai_aruco_marker'
    generated = tmp_path / checked_in.name
    generate_model(generated)
    for filename in ('model.sdf', 'model.config', 'marker.png'):
        assert (generated / filename).read_bytes() == (checked_in / filename).read_bytes()


def test_invalid_marker_id(tmp_path):
    with pytest.raises(ValueError):
        generate_model(tmp_path / 'invalid', marker_id=50)
