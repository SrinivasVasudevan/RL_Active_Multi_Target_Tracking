"""Launch one target node. Pass robot_id as argument (default 0).

Usage:
  ros2 launch limo_target target.launch.py robot_id:=0
  ros2 launch limo_target target.launch.py robot_id:=1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('limo_target')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('tau', default_value='2.0'),

        Node(
            package='limo_target',
            executable='target_node',
            name=['limo_target_', LaunchConfiguration('robot_id')],
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'target_params.yaml'),
                {
                    'robot_id': LaunchConfiguration('robot_id'),
                    'tau':      LaunchConfiguration('tau'),
                },
            ],
        ),
    ])
