import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("mbam_central_controller")
    default_params = os.path.join(pkg, "config", "params_compare.yaml")
    default_ckpt = os.path.join(pkg, "checkpoints", "best_model_seed42.pth")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("model_path", default_value=default_ckpt),
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument("fixed_num_landmarks", default_value="5"),
            Node(
                package="mbam_central_controller",
                executable="central_runner",
                name="mbam_central_runner",
                output="screen",
                parameters=[
                    {
                        "params_file": LaunchConfiguration("params_file"),
                        "model_path": LaunchConfiguration("model_path"),
                        "num_robots": LaunchConfiguration("num_robots"),
                        "fixed_num_landmarks": LaunchConfiguration("fixed_num_landmarks"),
                    }
                ],
            ),
        ]
    )
