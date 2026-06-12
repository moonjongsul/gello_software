import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _domain_group(actions, domain_id):
    """Wrap ``actions`` so they run under the given ROS_DOMAIN_ID.

    ``SetEnvironmentVariable`` inside a ``GroupAction`` is scoped to that group,
    so every node launched within inherits this ROS_DOMAIN_ID without affecting
    the rest of the launch.
    """
    if domain_id is None:
        return GroupAction(actions)
    return GroupAction([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain_id)),
        *actions,
    ])


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
    params = _load_ros_parameters(config_path)

    # box picking node runs under the vision ROS_DOMAIN_ID from the config.
    vision_domain = params.get('vision', {}).get('ros_domain_id')

    box_picking_node = Node(
        package='picking_cell',
        executable='box_picking',
        output='screen',
        parameters=[config_path],
    )

    return [_domain_group([box_picking_node], vision_domain)]


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
