import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _normalize_database_model_name(model_name: str) -> str:
    model_name = model_name.strip()
    if model_name.startswith("model://"):
        model_name = model_name[len("model://") :]
    return model_name.strip("/ ")


def _is_local_gazebo_model_available(model_name: str) -> bool:
    if not model_name:
        return False

    search_roots = [
        os.path.expanduser("~/.gazebo/models"),
        "/usr/share/gazebo-11/models",
    ]
    extra_paths = [p for p in os.environ.get("GAZEBO_MODEL_PATH", "").split(":") if p]
    search_roots.extend(extra_paths)

    for root in search_roots:
        root = root.strip()
        if not root:
            continue

        # Common layout: <root>/<model_name>/model.sdf
        candidate = os.path.join(root, model_name)
        if os.path.isfile(os.path.join(candidate, "model.sdf")):
            return True

        # Some GAZEBO_MODEL_PATH entries may already point at the model folder.
        if os.path.basename(os.path.normpath(root)) == model_name and os.path.isfile(
            os.path.join(root, "model.sdf")
        ):
            return True

    return False


def _spawn_entities(context, *args, **kwargs):
    pkg_share = get_package_share_directory("mbam_gazebo_tracking")
    agent_model = os.path.join(pkg_share, "models", "agent_bot", "model.sdf")
    target_model = os.path.join(pkg_share, "models", "target_bot", "model.sdf")
    agent_model_database = _normalize_database_model_name(
        LaunchConfiguration("agent_model_database").perform(context)
    )
    target_model_database = _normalize_database_model_name(
        LaunchConfiguration("target_model_database").perform(context)
    )
    strict_database_models = (
        LaunchConfiguration("strict_database_models").perform(context).strip().lower() == "true"
    )

    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    max_num_landmarks = int(LaunchConfiguration("max_num_landmarks").perform(context))

    actions = []
    spawn_nodes = []

    def _resolve_database_or_fallback(role: str, model_database: str) -> str:
        if not model_database:
            return ""

        if _is_local_gazebo_model_available(model_database):
            return model_database

        if strict_database_models:
            print(
                f"[mbam_gazebo_tracking] Requested {role} database model "
                f"'{model_database}' was not found locally; strict mode keeps database spawning enabled."
            )
            return model_database

        print(
            f"[mbam_gazebo_tracking] Requested {role} database model '{model_database}' "
            "is not installed locally. Falling back to bundled local SDF model."
        )
        return ""

    effective_agent_database = _resolve_database_or_fallback("agent", agent_model_database)
    effective_target_database = _resolve_database_or_fallback("target", target_model_database)

    def _spawn_args(
        entity: str,
        model_file: str,
        model_database: str,
        x: str,
        y: str,
        z: str,
        robot_namespace: str = "",
    ):
        args = ["-entity", entity]
        if model_database:
            args += ["-database", model_database]
        else:
            args += ["-file", model_file]
        args += ["-x", x, "-y", y, "-z", z]
        args += ["-timeout", "120.0"]
        if robot_namespace:
            args += ["-robot_namespace", robot_namespace]
        return args

    for i in range(num_robots):
        node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=_spawn_args(
                entity=f"agent_{i}",
                model_file=agent_model,
                model_database=effective_agent_database,
                x=str(i * 0.8),
                y="0.0",
                z="0.15",
                robot_namespace=f"agent_{i}",
            ),
        )
        spawn_nodes.append(node)

    for i in range(max_num_landmarks):
        node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=_spawn_args(
                entity=f"target_{i}",
                model_file=target_model,
                model_database=effective_target_database,
                x=str(2.0 + i * 0.5),
                y="2.0",
                z="0.15",
            ),
        )
        spawn_nodes.append(node)

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
                "publish_lidar_markers": LaunchConfiguration("publish_lidar_markers"),
                "lidar_marker_stride": LaunchConfiguration("lidar_marker_stride"),
                "restrict_lidar_visualization": LaunchConfiguration("restrict_lidar_visualization"),
                "lidar_visualization_margin_rad": LaunchConfiguration("lidar_visualization_margin_rad"),
                "enable_collision_pause": LaunchConfiguration("enable_collision_pause"),
                "collision_lookahead_sec": LaunchConfiguration("collision_lookahead_sec"),
                "collision_robot_radius": LaunchConfiguration("collision_robot_radius"),
                "collision_target_radius": LaunchConfiguration("collision_target_radius"),
                "collision_safety_margin": LaunchConfiguration("collision_safety_margin"),
            }
        ],
    )

    if not spawn_nodes:
        actions.append(TimerAction(period=2.0, actions=[runner]))
        return actions

    # Spawn entities sequentially. This avoids /spawn_entity service contention
    # and queue-timeout failures when many models are inserted at once.
    actions.append(spawn_nodes[0])
    for prev_node, next_node in zip(spawn_nodes, spawn_nodes[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=prev_node,
                    on_exit=[next_node],
                )
            )
        )

    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_nodes[-1],
                on_exit=[TimerAction(period=2.0, actions=[runner])],
            )
        )
    )
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
            DeclareLaunchArgument("publish_lidar_markers", default_value="false"),
            DeclareLaunchArgument("lidar_marker_stride", default_value="6"),
            DeclareLaunchArgument("restrict_lidar_visualization", default_value="true"),
            DeclareLaunchArgument("lidar_visualization_margin_rad", default_value="0.05"),
            DeclareLaunchArgument("enable_collision_pause", default_value="true"),
            DeclareLaunchArgument("collision_lookahead_sec", default_value="1.0"),
            DeclareLaunchArgument("collision_robot_radius", default_value="0.28"),
            DeclareLaunchArgument("collision_target_radius", default_value="0.30"),
            DeclareLaunchArgument("collision_safety_margin", default_value="0.10"),
            DeclareLaunchArgument("agent_model_database", default_value=""),
            DeclareLaunchArgument("target_model_database", default_value=""),
            DeclareLaunchArgument("strict_database_models", default_value="false"),
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
