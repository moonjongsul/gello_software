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

    # orbbec_1 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('orbbec_camera'),
    #         '/launch/gemini2.launch.py'
    #     ]),
    #     launch_arguments={
    #         "camera_name": "front_view",
    #         # "serial_number": "AY35C3200EM", # 카메라 시리얼 번호
    #         "usb_port": "2-2",
    #         "device_num": "2",
    #         "color_width": "1280",
    #         "color_height": "720",
    #         "color_fps": "30",

    #         "enable_color_auto_exposure": "true",
    #         "color_ae_max_exposure": "700",
    #         "color_brightness": "100",
            
    #         "enable_depth": "false",
    #         "enable_ir": "false",
    #         "enable_pointcloud": "false",
    #         "sync_mode": "primary",
    #         "trigger_out_enabled": "true",
    #         "attach_component_container_enable": "false",
    #         "attach_component_container_name": "orbbec_container_primary",
    #     }.items()
    # )

    # orbbec_2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('orbbec_camera'),
    #         '/launch/gemini2.launch.py'
    #     ]),
    #     launch_arguments={
    #         "camera_name": "side_view",
    #         # "serial_number": "AY3794301V0", # 카메라 시리얼 번호
    #         "usb_port": "2-4",
    #         "device_num": "2",
    #         "color_width": "1280",
    #         "color_height": "720",
    #         "color_fps": "30",

    #         "enable_color_auto_exposure": "true",
    #         "color_ae_max_exposure": "700",
    #         "color_brightness": "100",

    #         "enable_depth": "false",
    #         "enable_ir": "false",
    #         "enable_pointcloud": "false",
    #         "sync_mode": "secondary_synced",
    #         "trigger_out_enabled": "false",
    #         "attach_component_container_enable": "false",
    #         "attach_component_container_name": "orbbec_container_primary",
    #     }.items()
    # )

    # realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('realsense2_camera'),
    #         '/launch/rs_multi_camera_launch_sync.py'
    #     ]),
    #     launch_arguments={
    #         'camera_name1':      'front',
    #         'camera_name2':      'rear',
    #         'camera_namespace1': 'wrist',
    #         'camera_namespace2': 'wrist',
    #         'serial_no1':        "'_315122272391'",
    #         'serial_no2':        "'_335122271613'",
    #         'enable_depth1':     'false',
    #         'enable_depth2':     'false',
    #         'rotation_filter.enable2':   'true',
    #         'rotation_filter.rotation2': '180.0',
    #     }.items()
    # )

    return LaunchDescription([
        franka_cs,
        TimerAction(period=0.0, actions=[gripper]),
        # TimerAction(period=2.0, actions=[orbbec_1]),   # primary: 먼저 시작
        # TimerAction(period=4.0, actions=[orbbec_2]),   # secondary: primary 준비 후 시작
        # TimerAction(period=6.0, actions=[realsense]),
    ])
