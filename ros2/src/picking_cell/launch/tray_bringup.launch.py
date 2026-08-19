import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _domain_group(actions, domain_id):
    """지정한 ROS_DOMAIN_ID 환경에서 actions를 실행."""
    if domain_id is None:
        return GroupAction(actions)
    return GroupAction([SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain_id)), *actions])

def _load_ros_parameters(config_path):
    """YAML 파일에서 ros__parameters 내용만 읽는다."""
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f) or {}

    for node_value in data.values():
        if isinstance(node_value, dict) and 'ros__parameters' in node_value:
            return node_value['ros__parameters']
    return {}

def _stringify_args(launch_args):
    """launch argument 값은 문자열로 전달."""
    return {key: str(value) for key, value in (launch_args or {}).items()}

def _setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    params = _load_ros_parameters(config_path)

    camera_args = _stringify_args(params.get('camera', {}).get('launch_args', {}))
    robot_args = _stringify_args(params.get('robot', {}).get('launch_args', {}))

    camera_domain = params.get('camera', {}).get('ros_domain_id')
    robot_domain = params.get('robot', {}).get('ros_domain_id')
    vision_domain = params.get('vision', {}).get('ros_domain_id')

    ur_type = robot_args.get('ur_type')
    if ur_type and 'tf_prefix' not in robot_args:
        robot_args['tf_prefix'] = f'{ur_type}_'

    if 'namespace' not in robot_args:
        robot_args['namespace'] = ur_type or ''

    realsense_share = get_package_share_directory('realsense2_camera')
    pkg_share = get_package_share_directory('picking_cell')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments=camera_args.items()
    )

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'ur_control_ns.launch.py')),
        launch_arguments=robot_args.items()
    )

    tray_picking_node = Node(
        package='picking_cell',
        executable='tray_picking',
        name='tray_picking',
        output='screen',
        parameters=[config_path]
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'default_call_service_timeout': 60.0}]
    )

    return [
        _domain_group([realsense_launch], camera_domain),
        _domain_group([ur_control_launch], robot_domain),
        _domain_group([tray_picking_node, rosbridge_node], vision_domain)
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('picking_cell')

    # TrayPicking 전용 설정 파일
    default_config = os.path.join(pkg_share, 'config', 'config_tray.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the tray picking parameter YAML file.'
    )

    return LaunchDescription([config_arg, OpaqueFunction(function=_setup)])