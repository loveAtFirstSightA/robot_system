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
        Node(
            package='path_tracking',
            executable='path_tracking',
            name='path_tracking',
            # remappings=[('/* old_topic_name */', '/* new_topic_name */')],
            # parameters=[{'/* param_name */': /* param_value */}],
            parameters=[os.path.join(get_package_share_directory('path_tracking'),
                'config', 'params.yaml')],
            output='screen'),
    ])
