import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'yahboomcar_autonomous'
    yaml_name = 'mpc_settings.yaml'

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    yaml_path = os.path.join(pkg_share,'config',yaml_name)

    democar_mpc_node = Node(
        package='yahboomcar_autonomous',
        executable='mpc_node',
        name='mpc_controller',
        parameters=[yaml_path],
        output='screen',
    )

    ld.add_action(democar_mpc_node)

    return ld
