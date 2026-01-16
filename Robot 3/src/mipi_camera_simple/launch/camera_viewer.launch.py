#!/usr/bin/env python3
"""
Launch file untuk menjalankan camera viewer
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description untuk camera viewer"""

    # Declare launch arguments
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/mipi_camera/image_raw/compressed',
        description='Camera topic to subscribe'
    )

    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Show camera window'
    )

    # Create camera viewer node
    camera_viewer_node = Node(
        package='mipi_camera_simple',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'show_window': LaunchConfiguration('show_window'),
        }]
    )

    return LaunchDescription([
        topic_arg,
        show_window_arg,
        camera_viewer_node
    ])
