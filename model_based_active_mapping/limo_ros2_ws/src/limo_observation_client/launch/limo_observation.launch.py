from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='limo_1',
            description='Unique robot ID, for example limo_1 or limo_2.',
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Seconds between local observation publications.',
        ),
        Node(
            package='limo_observation_client',
            executable='limo_observation_node',
            name='limo_observation_node',
            output='screen',
            parameters=[
                {'robot_id': LaunchConfiguration('robot_id')},
                {'publish_period_sec': LaunchConfiguration('publish_period_sec')},
            ],
        ),
    ])
