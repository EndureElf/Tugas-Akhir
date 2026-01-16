from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    follower = Node(
        package='yahboomcar_multi',
        executable='robot2_follower_node',
        name='robot2_follower',
        namespace='robot2',
        output='screen',
        parameters=[
            {"follow_distance": 0.6},
            {"linear_gain": 0.6},
            {"angular_gain": 1.0}
        ]
    )

    return LaunchDescription([follower])
