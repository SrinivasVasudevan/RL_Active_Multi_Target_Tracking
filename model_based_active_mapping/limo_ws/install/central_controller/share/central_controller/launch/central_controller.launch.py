import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ROS2_MBAM = os.path.join(
    os.path.expanduser('~'),
    'HRA/RL_Active_Multi_Target_Tracking/model_based_active_mapping',
    'ros2_ws/install/mbam_gazebo_tracking/share/mbam_gazebo_tracking',
)


def generate_launch_description():
    pkg_share = get_package_share_directory('central_controller')

    default_params = os.path.join(_ROS2_MBAM, 'config', 'params_compare.yaml')
    default_ckpt = os.path.join(_ROS2_MBAM, 'checkpoints', 'best_model_seed42.pth')

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        DeclareLaunchArgument('num_targets', default_value='3'),
        DeclareLaunchArgument('max_num_targets', default_value='7'),
        DeclareLaunchArgument('tau', default_value='1.0'),
        DeclareLaunchArgument('max_linear_vel', default_value='0.15'),
        DeclareLaunchArgument('max_angular_vel', default_value='0.40'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('arena_half_size', default_value='5.0'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('model_path', default_value=default_ckpt),
        DeclareLaunchArgument('single_robot_mode', default_value='false'),

        Node(
            package='central_controller',
            executable='central_controller',
            name='central_controller',
            output='screen',
            parameters=[{
                'num_robots':        LaunchConfiguration('num_robots'),
                'num_targets':       LaunchConfiguration('num_targets'),
                'max_num_targets':   LaunchConfiguration('max_num_targets'),
                'tau':               LaunchConfiguration('tau'),
                'max_linear_vel':    LaunchConfiguration('max_linear_vel'),
                'max_angular_vel':   LaunchConfiguration('max_angular_vel'),
                'seed':              LaunchConfiguration('seed'),
                'arena_half_size':   LaunchConfiguration('arena_half_size'),
                'params_file':       LaunchConfiguration('params_file'),
                'model_path':        LaunchConfiguration('model_path'),
                'single_robot_mode': LaunchConfiguration('single_robot_mode'),
            }],
        ),
    ])
