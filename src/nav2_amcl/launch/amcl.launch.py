#!/usr/bin python3
# -*- coding: UTF-8 -*-

# startup localization of amcl

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('nav2_amcl'),
                'config', 'params.yaml'
            ),
            description='Full path to the AMCL parameter file to use'),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            # parameters=[os.path.join(get_package_share_directory('nav2_amcl'),
            #     'config', 'params.yaml')],
            parameters=[LaunchConfiguration('params_file')],
            output='screen'),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{'autostart': False},
                        {'node_names': ['amcl']}],
            output='screen')
    ])
