"""Build namespaced Nav2 parameters and validate the map-loading interface."""

from copy import deepcopy
from pathlib import Path

import yaml

from .geometry import shift_pose


def replace_robot(value, robot):
    if isinstance(value, dict):
        return {key: replace_robot(item, robot) for key, item in value.items()}
    if isinstance(value, list):
        return [replace_robot(item, robot) for item in value]
    return value.replace('{robot}', robot) if isinstance(value, str) else value


def costmap_parameters(robot, footprint, global_map):
    return {
        'use_sim_time': True, 'global_frame': 'map' if global_map else f'{robot}/odom',
        'robot_base_frame': f'{robot}/drive_frame',
        'footprint': str(footprint), 'footprint_padding': .025,
        'resolution': .025, 'update_frequency': 3. if global_map else 8.,
        'publish_frequency': 2., 'transform_tolerance': .3,
        'rolling_window': not global_map, 'track_unknown_space': False,
        **({} if global_map else {'width': 3, 'height': 3}),
        'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
        'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/map',
                         'map_subscribe_transient_local': True},
        'obstacle_layer': {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
            'observation_sources': 'scan peer',
            'scan': {'topic': f'/{robot}/scan', 'data_type': 'LaserScan', 'clearing': True,
                     'marking': True, 'inf_is_valid': True, 'raytrace_max_range': 12.,
                     'obstacle_max_range': 12., 'obstacle_min_range': .0,
                     'max_obstacle_height': 1.0},
            'peer': {'topic': f'/{robot}/peer_obstacles', 'data_type': 'PointCloud2',
                     'sensor_frame': f'{robot}/base_footprint', 'clearing': True, 'marking': True,
                     'observation_persistence': 0., 'expected_update_rate': .5,
                     'min_obstacle_height': .0, 'max_obstacle_height': 1.,
                     'obstacle_max_range': 12., 'raytrace_max_range': 12.},
        },
        'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                            'cost_scaling_factor': 3., 'inflation_radius': .60},
        'always_send_full_costmap': True,
    }


def robot_parameters(template, robot, start, footprint, control_offset):
    parameters = replace_robot(deepcopy(template), robot)
    amcl = parameters['amcl']['ros__parameters']
    initial = shift_pose([float(start['x']), float(start['y']), float(start['yaw'])], control_offset)
    amcl.update({'initial_pose': {'x': initial[0], 'y': initial[1], 'z': 0., 'yaw': initial[2]}})
    for name, global_map in (('local_costmap', False), ('global_costmap', True)):
        control_footprint = [[x-control_offset, y] for x, y in footprint]
        parameters[name] = {name: {'ros__parameters': costmap_parameters(robot, control_footprint, global_map)}}
    stop = [[x-control_offset+(.03 if x > 0 else -.03), y+(.03 if y > 0 else -.03)] for x, y in footprint]
    parameters['collision_monitor']['ros__parameters']['StopFootprint']['points'] = str(stop)
    return {robot: parameters}


def resolve_map(yaml_path, image_override, output):
    yaml_path = Path(yaml_path).expanduser().resolve()
    if not yaml_path.is_file():
        raise ValueError(f'Map YAML not found: {yaml_path}; supply map_yaml:=...')
    metadata = yaml.safe_load(yaml_path.read_text())
    image = Path(image_override).expanduser().resolve() if image_override else Path(metadata['image'])
    if not image.is_absolute():
        image = yaml_path.parent/image
    if not image.is_file():
        raise ValueError(f'Map image not found: {image}; supply map_image:=...')
    if float(metadata['resolution']) <= 0 or len(metadata['origin']) != 3:
        raise ValueError('Map requires positive resolution and an [x,y,yaw] origin')
    metadata['image'] = str(image)
    Path(output).write_text(yaml.safe_dump(metadata, sort_keys=False))
    return str(output)
