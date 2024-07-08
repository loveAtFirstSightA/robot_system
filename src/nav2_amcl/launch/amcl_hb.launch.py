import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/home/fcbox/.fcbox/map/1709870483105/MAP_release1.4.3_0308_1709870471.yaml',
        description='Full path to map yaml file to load'
    )
    map_yaml_path = LaunchConfiguration('map_yaml_path')

    nav2_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_path}],
        output='screen')
    
    nav2_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[os.path.join(get_package_share_directory('nav2_amcl'),
            'config', 'hb.yaml')],
        output='screen')
    
    nav2_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['amcl', 'map_server']}],
        output='screen')
    
    ld = LaunchDescription()
    
    ld.add_action(map_yaml_path_arg)
    ld.add_action(nav2_map_server_cmd)
    ld.add_action(nav2_amcl_cmd)
    ld.add_action(nav2_lifecycle_manager_cmd)
    
    return ld
