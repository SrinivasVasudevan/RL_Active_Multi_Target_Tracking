from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ids',
            default_value='limo_1,limo_2',
            description='Comma-separated robot IDs expected by the controller.',
        ),
        DeclareLaunchArgument(
            'policy_mode',
            default_value='target_or_search',
            description='Controller policy mode: target_or_search or stop.',
        ),
        DeclareLaunchArgument(
            'target_linear_mps',
            default_value='0.16',
            description='Desired forward speed before LIMO-side safety limiting.',
        ),
        DeclareLaunchArgument(
            'search_angular_radps',
            default_value='0.25',
            description='Desired angular speed while searching for targets.',
        ),
        Node(
            package='central_controller',
            executable='central_controller_node',
            name='central_controller',
            output='screen',
            parameters=[
                {'robot_ids': LaunchConfiguration('robot_ids')},
                {'policy_mode': LaunchConfiguration('policy_mode')},
                {'target_linear_mps': LaunchConfiguration('target_linear_mps')},
                {'search_angular_radps': LaunchConfiguration('search_angular_radps')},
            ],
        ),
    ])
