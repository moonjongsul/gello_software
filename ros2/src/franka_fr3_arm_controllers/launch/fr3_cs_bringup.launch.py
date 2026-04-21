from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    franka_cs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('franka_fr3_arm_controllers'),
            '/launch/franka_fr3_cartesian_controller.launch.py'
        ])
    )

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('franka_gripper_manager'),
            '/launch/dxl_parallel_gripper_client.launch.py'
        ])
    )

    return LaunchDescription([
        franka_cs,
        TimerAction(period=0.0, actions=[gripper]),
    ])
