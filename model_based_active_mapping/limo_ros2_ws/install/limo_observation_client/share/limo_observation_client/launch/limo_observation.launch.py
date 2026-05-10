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
            default_value='0.2',
            description='Seconds between local observation publications.',
        ),
        DeclareLaunchArgument(
            'output_cmd_vel_topic',
            default_value='/cmd_vel',
            description='Final LIMO velocity topic after local safety limiting.',
        ),
        DeclareLaunchArgument(
            'test_speed_scale',
            default_value='0.35',
            description='Global speed scale applied on the LIMO during tests.',
        ),
        DeclareLaunchArgument(
            'max_linear_mps',
            default_value='0.18',
            description='Maximum forward/reverse speed allowed by the LIMO safety node.',
        ),
        DeclareLaunchArgument(
            'max_angular_radps',
            default_value='0.55',
            description='Maximum yaw speed allowed by the LIMO safety node.',
        ),
        DeclareLaunchArgument(
            'front_stop_distance_m',
            default_value='0.55',
            description='Stop forward motion if front scan sector is closer than this.',
        ),
        Node(
            package='limo_observation_client',
            executable='limo_observation_node',
            namespace=LaunchConfiguration('robot_id'),
            name='observation_node',
            output='screen',
            parameters=[
                {'robot_id': LaunchConfiguration('robot_id')},
                {'publish_period_sec': LaunchConfiguration('publish_period_sec')},
                {'output_cmd_vel_topic': LaunchConfiguration('output_cmd_vel_topic')},
                {'test_speed_scale': LaunchConfiguration('test_speed_scale')},
                {'max_linear_mps': LaunchConfiguration('max_linear_mps')},
                {'max_angular_radps': LaunchConfiguration('max_angular_radps')},
                {'front_stop_distance_m': LaunchConfiguration('front_stop_distance_m')},
            ],
        ),
    ])
