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
        Node(
            package='central_controller',
            executable='central_controller_node',
            name='central_controller',
            output='screen',
            parameters=[
                {'robot_ids': LaunchConfiguration('robot_ids')},
            ],
        ),
    ])
