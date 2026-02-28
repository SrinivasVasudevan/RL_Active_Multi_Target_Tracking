import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_entities(context, *args, **kwargs):
    pkg_share = get_package_share_directory("mbam_gazebo_tracking")
    agent_model = os.path.join(pkg_share, "models", "agent_bot", "model.sdf")
    target_model = os.path.join(pkg_share, "models", "target_bot", "model.sdf")

    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    max_num_landmarks = int(LaunchConfiguration("max_num_landmarks").perform(context))

    actions = []

    for i in range(num_robots):
        actions.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-entity",
                    f"agent_{i}",
                    "-file",
                    agent_model,
                    "-x",
                    str(i * 0.8),
                    "-y",
                    "0.0",
                    "-z",
                    "0.15",
                    "-robot_namespace",
                    f"agent_{i}",
                ],
            )
        )

    for i in range(max_num_landmarks):
        actions.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-entity",
                    f"target_{i}",
                    "-file",
                    target_model,
                    "-x",
                    str(2.0 + i * 0.5),
                    "-y",
                    "2.0",
                    "-z",
                    "0.15",
                ],
            )
        )

    runner = Node(
        package="mbam_gazebo_tracking",
        executable="episode_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "params_file": LaunchConfiguration("mbam_params_file"),
                "model_path": LaunchConfiguration("model_path"),
                "seed": LaunchConfiguration("seed"),
                "network_type": LaunchConfiguration("network_type"),
                "num_robots": LaunchConfiguration("num_robots"),
                "max_num_landmarks": LaunchConfiguration("max_num_landmarks"),
                "num_clusters": LaunchConfiguration("num_clusters"),
                "num_test_trials": LaunchConfiguration("num_test_trials"),
                "output_dir": LaunchConfiguration("output_dir"),
                "debug_sensor_fusion": LaunchConfiguration("debug_sensor_fusion"),
            }
        ],
    )

    actions.append(TimerAction(period=5.0, actions=[runner]))
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("mbam_gazebo_tracking")
    gazebo_share = get_package_share_directory("gazebo_ros")

    world = os.path.join(pkg_share, "worlds", "mbam_tracking.world")
    rviz_config = os.path.join(pkg_share, "rviz", "mbam_tracking.rviz")
    params_file = os.path.join(pkg_share, "config", "params_compare.yaml")
    checkpoint = os.path.join(pkg_share, "checkpoints", "best_model_seed42.pth")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "verbose": "true",
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    static_world_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "map",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("network_type", default_value="1"),
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument("max_num_landmarks", default_value="7"),
            DeclareLaunchArgument("num_clusters", default_value="2"),
            DeclareLaunchArgument("num_test_trials", default_value="-1"),
            DeclareLaunchArgument("debug_sensor_fusion", default_value="false"),
            DeclareLaunchArgument("mbam_params_file", default_value=params_file),
            DeclareLaunchArgument("model_path", default_value=checkpoint),
            DeclareLaunchArgument(
                "output_dir",
                default_value=os.path.join(os.getcwd(), "mbam_ros2_test_results"),
            ),
            gazebo,
            static_world_map_tf,
            OpaqueFunction(function=_spawn_entities),
            rviz,
        ]
    )
