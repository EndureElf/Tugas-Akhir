import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('yahboomcar_multi'),
        'param',
        'robot1_nav_params.yaml'
    )
    # =============== NAV2 SERVERS ===============

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        namespace='robot1',
        name='controller_server',
        output='screen',
        parameters=[params],
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        namespace='robot1',
        name='planner_server',
        output='screen',
        parameters=[params],
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        namespace='robot1',
        name='behavior_server',
        output='screen',
        parameters=[params],
    )

    # =============== BT NAVIGATOR DENGAN SYNC DELAY ===============
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        namespace='robot1',
        name='bt_navigator',
        output='screen',
        parameters=[params],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        namespace='robot1',
        name='waypoint_follower',
        output='screen',
        parameters=[params],
    )

    # =============== LIFECYCLE MANAGER ===============

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='robot1',
        name='nav_lifecycle_manager',
        output='screen',
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"bond_timeout": 20.0},
            {"node_names": [
                "controller_server",
                "planner_server", 
                "behavior_server",
                "bt_navigator",
                "waypoint_follower"
            ]}
        ]
    )

    return LaunchDescription([
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_node
    ])
