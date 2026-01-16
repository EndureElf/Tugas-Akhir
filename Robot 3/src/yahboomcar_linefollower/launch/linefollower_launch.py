from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yahboomcar_bringup'),
                'launch/yahboomcar_bringup_launch.py'))
    )

    launch_description = LaunchDescription([
        bringup_launch_include,
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"camera_calibration_file_path": "/opt/tros/humble/lib/mipi_cam/config/GC4663_calibration.yaml"},
                {"image_width": 960},
                {"image_height": 544},
                {"video_device": "GC4663"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        Node(
            package='yahboomcar_linefollower',
            executable='linefollower',
        )
        ]) 
    return launch_description
