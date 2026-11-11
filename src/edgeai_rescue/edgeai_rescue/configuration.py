"""Combine shared arena specifications with Part 2 assignments and tuning."""

from pathlib import Path

import yaml


def load_mission_config(path, arena_share=None):
    if arena_share is None:
        from ament_index_python.packages import get_package_share_directory
        arena_share = get_package_share_directory('edgeai_robot_description')
    arena_config = Path(arena_share) / 'config'
    config = yaml.safe_load(Path(path).read_text())
    # Explicit overrides in a custom mission file remain supported.
    config.setdefault('robots', yaml.safe_load((arena_config / 'rescue_robots.yaml').read_text()))
    targets = yaml.safe_load((arena_config / 'rescue_targets.yaml').read_text())['targets']
    config['targets'] = {
        name: {**targets[name], **settings} for name, settings in config['targets'].items()
    }
    return config
