import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(package_path, 'maps', 'yahboom_map_lss_carto.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'mppi_nav_params.yaml'))

    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename', 
        default=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path,
                              description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to param file to load'),
        
        DeclareLaunchArgument('default_bt_xml_filename', default_value=default_bt_xml_filename,
                              description='Full path to behavior tree xml file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'default_bt_xml_filename': default_bt_xml_filename}.items(),
        ),
    ])
