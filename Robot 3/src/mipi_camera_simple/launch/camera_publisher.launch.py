#!/usr/bin/env python3
"""
Launch file untuk menjalankan MIPI camera publisher
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description untuk camera publisher"""

    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera width resolution'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Camera height resolution'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Camera frame rate'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug output'
    )

    # Create camera publisher node
    camera_publisher_node = Node(
        package='mipi_camera_simple',
        executable='camera_publisher',
        name='mipi_camera_publisher',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'debug': LaunchConfiguration('debug'),
        }]
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        debug_arg,
        camera_publisher_node
    ])
