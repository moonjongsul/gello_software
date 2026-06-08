import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_ros_parameters(config_path):
    """Read the picking_cell YAML and return the ros__parameters mapping.

    The config uses the ``/**`` wildcard node name, so we unwrap that and the
    ``ros__parameters`` key to get at the plain parameter dictionary.
    """
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f) or {}

    for node_value in data.values():
        if isinstance(node_value, dict) and 'ros__parameters' in node_value:
            return node_value['ros__parameters']
    return {}


def _stringify_args(launch_args):
    """launch_arguments values must be strings."""
    return {key: str(value) for key, value in (launch_args or {}).items()}


def _setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)

    # box picking node
    box_picking_node = Node(
        package='picking_cell',
        executable='box_picking',
        output='screen',
        parameters=[config_path],
    )

    return [box_picking_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('picking_cell')
    default_config = os.path.join(pkg_share, 'config', 'config.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the picking_cell parameter YAML file.',
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=_setup),
    ])
