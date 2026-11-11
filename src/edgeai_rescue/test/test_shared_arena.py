"""Check shared midterm assets, prepared map and safe rescue poses together."""

from pathlib import Path

import numpy as np
import yaml

from edgeai_rescue.configuration import load_mission_config
from edgeai_rescue.geometry import OccupancyGrid, Pose2D, polygon_at, polygons_overlap, validate_goals
from edgeai_rescue.reference_map import generate
from edgeai_rescue.robot_model import describe_robot


PACKAGE = Path(__file__).resolve().parents[1]
ARENA = PACKAGE.parent / 'edgeai_robot_description'


def test_default_mission_uses_shared_robot_and_target_specifications():
    config = load_mission_config(PACKAGE / 'config/mission.yaml', ARENA)
    assert config['robots'] == yaml.safe_load((ARENA / 'config/rescue_robots.yaml').read_text())
    targets = yaml.safe_load((ARENA / 'config/rescue_targets.yaml').read_text())['targets']
    for name, target in targets.items():
        assert all(config['targets'][name][key] == value for key, value in target.items())
    assert config['targets']['S1']['robots'] == ['robot2']
    assert set(config['targets']['C1']['robots']) == {'robot1', 'robot2'}


def test_custom_mission_can_override_defaults(tmp_path):
    custom = yaml.safe_load((PACKAGE / 'config/mission.yaml').read_text())
    custom['robots'] = {'robot1': {'x': .5, 'y': .5, 'yaw': 0}}
    custom['targets']['S1']['hold_seconds'] = 12.
    path = tmp_path / 'mission.yaml'
    path.write_text(yaml.safe_dump(custom))
    config = load_mission_config(path, ARENA)
    assert config['robots'] == custom['robots']
    assert config['targets']['S1']['hold_seconds'] == 12.
    assert config['targets']['C1']['hold_seconds'] == 10.


def test_prepared_map_matches_shared_world_and_rescue_slots_are_safe(tmp_path):
    data, metadata = generate(ARENA / 'worlds/rescue_arena.sdf', ARENA / 'models',
                              tmp_path / 'rescue_reference')
    assert (tmp_path / 'rescue_reference.pgm').read_bytes() == (PACKAGE / 'maps/rescue_reference.pgm').read_bytes()
    assert metadata == yaml.safe_load((PACKAGE / 'maps/rescue_reference.yaml').read_text())
    grid = OccupancyGrid(data, metadata['resolution'], metadata['origin'])
    _, footprint = describe_robot(ARENA / 'urdf/edgeai_robot_real.urdf.xacro', 'robot1')
    config = load_mission_config(PACKAGE / 'config/mission.yaml', ARENA)
    starts = [Pose2D(**pose) for pose in config['robots'].values()]
    for pose in starts:
        assert grid.pose_is_free(pose, footprint, padding=.015)
        points = polygon_at(pose, footprint)
        assert np.all(points >= [0, 0]) and np.all(points <= [1.4, 1.0])
    assert not polygons_overlap(*(polygon_at(pose, footprint) for pose in starts), clearance=.08)
    for target in config['targets'].values():
        validate_goals(target, grid, footprint)
