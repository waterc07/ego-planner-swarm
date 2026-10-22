import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.utilities import evaluate_parameters


def parameters(**overrides):
    path = Path(__file__).parents[1]/"launch"/"boom_birds_offline.launch.py"
    spec = importlib.util.spec_from_file_location("ego_bounds_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({k: str(v) for k, v in overrides.items()})
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument): action.execute(context)
    actions = next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
    # 只展开配置，不执行 Node 或启动进程。
    return {k: v for p in evaluate_parameters(context, actions[0]._Node__parameters) for k, v in p.items()}


def test_default_negative_bounds_and_configured_vio_origin():
    original = parameters()
    shifted = parameters(map_min_z=-.5, virtual_ceil_z=2.9)
    assert original["grid_map/ground_height"] == -.1
    assert shifted["grid_map/ground_height"] == -.5
    assert shifted["grid_map/virtual_ceil_height"] == 2.9
    assert shifted["grid_map/ground_height"] < -.03445 < -.02031 < 0
    assert shifted["grid_map/map_size_z"] == original["grid_map/map_size_z"] == 6.


@pytest.mark.parametrize("overrides", [
    {"map_min_z": "nan"}, {"map_min_z": "inf"},
    {"virtual_ceil_z": "nan"}, {"map_size_z": "inf"},
    {"map_size_z": 0}, {"map_size_z": -1},
    {"virtual_ceil_z": -.1}, {"virtual_ceil_z": 6},
])
def test_invalid_vertical_bounds_refuse_before_node_execution(overrides):
    with pytest.raises(ValueError, match="map_min_z"):
        parameters(**overrides)
