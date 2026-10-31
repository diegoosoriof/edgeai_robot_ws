import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
import yaml

from edgeai_robot_tracking.methods import validate_detection_method


def load_launch(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'detection_launch', package / 'launch/detection_tracking.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(package))
    captured = {}

    def capture_node(**kwargs):
        captured.update(kwargs)
        return kwargs

    monkeypatch.setattr(module, 'Node', capture_node)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    return module, context, captured


@pytest.mark.parametrize('method', ['green_seeds', 'geometry'])
def test_launch_dispatch_defaults_and_configuration_precedence(monkeypatch, tmp_path, method):
    module, context, captured = load_launch(monkeypatch, tmp_path)
    assert context.launch_configurations['detection_method'] == 'green_seeds'
    context.launch_configurations.update(
        detection_method=method, observer_ns='robot7', start_simulation='true')
    custom = tmp_path / 'custom.yaml'
    custom.write_text('/**:\n  ros__parameters:\n    output_frame: custom_base\n'
                      '    geometry:\n      max_forward: 4.0\n')
    context.launch_configurations['config_file'] = str(custom)
    module.experiment(context)
    parameters = evaluate_parameters(context, normalize_parameters(captured['parameters']))
    merged = {}
    for item in parameters:
        if isinstance(item, Path):
            merged.update(yaml.safe_load(item.read_text())['/**']['ros__parameters'])
        else:
            merged.update(item)
    assert captured['namespace'] == 'robot7'
    assert merged['detection_method'] == method
    assert merged['use_sim_time'] is True
    assert merged['depth_optical_frame'] == 'robot7/camera_optical_frame'
    assert merged['output_frame'] == 'custom_base'
    assert merged['geometry']['max_forward'] == 4.0
    context.launch_configurations['output_frame'] = 'explicit_output'
    module.experiment(context)
    assert evaluate_parameters(context, normalize_parameters(captured['parameters']))[-1][
        'output_frame'] == (
        'explicit_output')


def test_unsupported_method_is_rejected_by_launch_and_node_contract(monkeypatch, tmp_path):
    module, context, _ = load_launch(monkeypatch, tmp_path)
    context.launch_configurations['detection_method'] = 'nearest_obstacle'
    with pytest.raises(ValueError, match='Unsupported detection_method'):
        module.experiment(context)
    with pytest.raises(ValueError, match='green_seeds or geometry'):
        validate_detection_method('nearest_obstacle')
