from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument(
                "desired_vel_topic",
                default_value="mbam/target_0/desired_world_velocity",
            ),
            Node(
                package="mbam_limo_target",
                executable="target_velocity_follower",
                name="mbam_target_follower",
                output="screen",
                parameters=[
                    {
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "desired_vel_topic": LaunchConfiguration("desired_vel_topic"),
                        "cmd_vel_out": "cmd_vel_raw",
                        "max_linear_x": 0.12,
                        "max_angular_z": 0.4,
                    }
                ],
            ),
            Node(
                package="mbam_limo_target",
                executable="cmd_vel_safety",
                name="mbam_target_safety",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_in": "cmd_vel_raw",
                        "cmd_vel_out": "cmd_vel",
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "max_linear_x": 0.12,
                        "max_angular_z": 0.4,
                        "min_range_m": 0.45,
                    }
                ],
            ),
        ]
    )
