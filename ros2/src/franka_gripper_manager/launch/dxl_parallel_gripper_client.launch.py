import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("franka_gripper_manager")

    default_config = [
        pkg_share,
        "/config/dxl_parallel_gripper_client.yaml",
    ]

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Full path to the dxl_parallel_gripper_client config YAML file",
    )

    node = Node(
        package="franka_gripper_manager",
        executable="dxl_parallel_gripper_client",
        name="dxl_parallel_gripper_client",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([
        config_arg,
        node,
    ])
