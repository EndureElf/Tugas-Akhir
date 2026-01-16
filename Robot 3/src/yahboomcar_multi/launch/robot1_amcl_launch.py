import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_file = os.path.join(
        get_package_share_directory('yahboomcar_multi'),
        'param',
        'robot1_amcl_params.yaml'
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',                     # ← WAJIB (jangan robot1_amcl)
        namespace='robot1',              # ← WAJIB untuk multi robot
        parameters=[param_file],
        remappings=[
            ('scan', '/robot1/scan'),
            ('initialpose', '/robot1/initialpose'),
            ('tf', 'tf'),
            ('tf_static', 'tf_static'),
        ],
        output='screen'
    )

    life_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='robot1',             # ← masuk namespace robot1
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['amcl']}
        ],
        output='screen'
    )

    return LaunchDescription([
        amcl_node,
        life_node
    ])
