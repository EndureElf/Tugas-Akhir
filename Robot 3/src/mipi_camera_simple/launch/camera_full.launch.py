#!/usr/bin/env python3
"""
Launch file untuk menjalankan publisher dan viewer sekaligus
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description untuk publisher + viewer"""

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
            'debug': True,
        }]
    )

    # Create camera viewer node
    camera_viewer_node = Node(
        package='mipi_camera_simple',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'topic': '/mipi_camera/image_raw/compressed',
            'show_window': True,
        }]
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        camera_publisher_node,
        camera_viewer_node
    ])
