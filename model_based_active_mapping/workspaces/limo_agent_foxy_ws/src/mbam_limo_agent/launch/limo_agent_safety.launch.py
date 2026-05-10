from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_in", default_value="cmd_vel_raw"),
            DeclareLaunchArgument("cmd_vel_out", default_value="cmd_vel"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("max_linear_x", default_value="0.2"),
            DeclareLaunchArgument("max_angular_z", default_value="0.5"),
            DeclareLaunchArgument("min_range_m", default_value="0.5"),
            Node(
                package="mbam_limo_agent",
                executable="cmd_vel_safety",
                name="mbam_cmd_vel_safety",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_in": LaunchConfiguration("cmd_vel_in"),
                        "cmd_vel_out": LaunchConfiguration("cmd_vel_out"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "max_linear_x": LaunchConfiguration("max_linear_x"),
                        "max_angular_z": LaunchConfiguration("max_angular_z"),
                        "min_range_m": LaunchConfiguration("min_range_m"),
                    }
                ],
            ),
        ]
    )
