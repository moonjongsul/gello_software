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
    params = _load_ros_parameters(config_path)

    camera_args = _stringify_args(params.get('camera', {}).get('launch_args', {}))
    robot_args = _stringify_args(params.get('robot', {}).get('launch_args', {}))

    # Use ur_type (e.g. "ur10e") as the tf_prefix so joint/link/TF names are
    # namespaced (ur10e_shoulder_link, ...). UR expects a trailing underscore.
    ur_type = robot_args.get('ur_type')
    if ur_type and 'tf_prefix' not in robot_args:
        robot_args['tf_prefix'] = f'{ur_type}_'

    # ROS namespace for the whole UR stack (e.g. "ur10e"). Topics/nodes end up
    # under /ur10e/... . Pull it from launch_args, falling back to ur_type.
    # The namespaced launch (ur_control_ns.launch.py) consumes this as its own
    # `namespace` argument, so the spawner targets /<ns>/controller_manager.
    if 'namespace' not in robot_args:
        robot_args['namespace'] = ur_type or ''

    realsense_share = get_package_share_directory('realsense2_camera')
    pkg_share = get_package_share_directory('picking_cell')

    # RealSense camera: realsense2_camera/rs_launch.py
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments=camera_args.items(),
    )

    # UR robot driver via our namespace-aware wrapper of ur_control.launch.py.
    # It puts every UR node (incl. /joint_states) under the namespace and keeps
    # the controller spawner pointed at the namespaced controller_manager.
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ur_control_ns.launch.py')
        ),
        launch_arguments=robot_args.items(),
    )

    # box picking node
    box_picking_node = Node(
        package='picking_cell',
        executable='box_picking',
        output='screen',
        parameters=[config_path],
    )
    # return [ur_control_launch]
    return [realsense_launch, ur_control_launch]#, box_picking_node]


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
