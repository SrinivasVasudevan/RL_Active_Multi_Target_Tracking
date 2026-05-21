"""Launch one agent node. Pass robot_id as argument (default 0).

Usage:
  ros2 launch limo_agent agent.launch.py robot_id:=0
  ros2 launch limo_agent agent.launch.py robot_id:=1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('limo_agent')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('max_num_targets', default_value='7'),
        DeclareLaunchArgument('tau', default_value='1.0'),

        Node(
            package='limo_agent',
            executable='agent_node',
            name=['limo_agent_', LaunchConfiguration('robot_id')],
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'agent_params.yaml'),
                {
                    'robot_id':        LaunchConfiguration('robot_id'),
                    'max_num_targets': LaunchConfiguration('max_num_targets'),
                    'tau':             LaunchConfiguration('tau'),
                },
            ],
        ),
    ])
