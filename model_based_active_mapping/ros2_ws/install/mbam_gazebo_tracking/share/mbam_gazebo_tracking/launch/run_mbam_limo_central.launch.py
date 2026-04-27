import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mbam_gazebo_tracking")
    params_file = os.path.join(pkg_share, "config", "params_compare.yaml")
    checkpoint = os.path.join(pkg_share, "checkpoints", "best_model_seed42.pth")

    coordinator = Node(
        package="mbam_gazebo_tracking",
        executable="limo_central_coordinator",
        output="screen",
        parameters=[
            {
                "params_file": LaunchConfiguration("mbam_params_file"),
                "model_path": LaunchConfiguration("model_path"),
                "robot_names": LaunchConfiguration("robot_names"),
                "report_topic": LaunchConfiguration("report_topic"),
                "cmd_topic_template": LaunchConfiguration("cmd_topic_template"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "marker_frame": LaunchConfiguration("marker_frame"),
                "max_num_landmarks": LaunchConfiguration("max_num_landmarks"),
                "control_period_sec": LaunchConfiguration("control_period_sec"),
                "max_report_age_sec": LaunchConfiguration("max_report_age_sec"),
                "track_association_distance_m": LaunchConfiguration("track_association_distance_m"),
                "track_timeout_sec": LaunchConfiguration("track_timeout_sec"),
                "search_linear_velocity": LaunchConfiguration("search_linear_velocity"),
                "search_angular_velocity": LaunchConfiguration("search_angular_velocity"),
                "enable_collision_pause": LaunchConfiguration("enable_collision_pause"),
                "collision_lookahead_sec": LaunchConfiguration("collision_lookahead_sec"),
                "collision_robot_radius": LaunchConfiguration("collision_robot_radius"),
                "collision_target_radius": LaunchConfiguration("collision_target_radius"),
                "collision_safety_margin": LaunchConfiguration("collision_safety_margin"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mbam_params_file", default_value=params_file),
            DeclareLaunchArgument("model_path", default_value=checkpoint),
            DeclareLaunchArgument("robot_names", default_value="limo0,limo1"),
            DeclareLaunchArgument("report_topic", default_value="/mbam/robot_reports"),
            DeclareLaunchArgument("cmd_topic_template", default_value="/{robot_name}/mbam_cmd_vel"),
            DeclareLaunchArgument("marker_topic", default_value="/mbam/real_world_markers"),
            DeclareLaunchArgument("marker_frame", default_value="map"),
            DeclareLaunchArgument("max_num_landmarks", default_value="7"),
            DeclareLaunchArgument("control_period_sec", default_value="-1.0"),
            DeclareLaunchArgument("max_report_age_sec", default_value="1.0"),
            DeclareLaunchArgument("track_association_distance_m", default_value="1.5"),
            DeclareLaunchArgument("track_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("search_linear_velocity", default_value="0.0"),
            DeclareLaunchArgument("search_angular_velocity", default_value="0.4"),
            DeclareLaunchArgument("enable_collision_pause", default_value="true"),
            DeclareLaunchArgument("collision_lookahead_sec", default_value="1.0"),
            DeclareLaunchArgument("collision_robot_radius", default_value="0.28"),
            DeclareLaunchArgument("collision_target_radius", default_value="0.35"),
            DeclareLaunchArgument("collision_safety_margin", default_value="0.10"),
            coordinator,
        ]
    )
