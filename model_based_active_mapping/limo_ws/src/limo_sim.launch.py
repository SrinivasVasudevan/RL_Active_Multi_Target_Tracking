"""
Combined simulation launch: Gazebo + central_controller + N agent nodes + M target nodes.

All nodes are throttled to slow, visualizable speeds (see config params).

Usage:
  ros2 launch limo_ws/src/limo_sim.launch.py
  ros2 launch limo_ws/src/limo_sim.launch.py num_robots:=2 num_targets:=3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_and_launch(context, *args, **kwargs):
    mbam_share = get_package_share_directory('mbam_gazebo_tracking')
    agent_sdf = os.path.join(mbam_share, 'models', 'agent_bot', 'model.sdf')
    target_sdf = os.path.join(mbam_share, 'models', 'target_bot', 'model.sdf')

    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    num_targets = int(LaunchConfiguration('num_targets').perform(context))
    max_num_targets = int(LaunchConfiguration('max_num_targets').perform(context))
    tau = LaunchConfiguration('tau').perform(context)
    max_lin = LaunchConfiguration('max_linear_vel').perform(context)
    max_ang = LaunchConfiguration('max_angular_vel').perform(context)

    mbam_params = os.path.join(mbam_share, 'config', 'params_compare.yaml')
    mbam_ckpt = os.path.join(mbam_share, 'checkpoints', 'best_model_seed42.pth')

    limo_agent_share = get_package_share_directory('limo_agent')
    limo_target_share = get_package_share_directory('limo_target')

    actions = []

    # Spawn agent robots
    for i in range(num_robots):
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', f'agent_{i}',
                '-file', agent_sdf,
                '-x', str(i * 1.0),
                '-y', '0.0',
                '-z', '0.15',
                '-robot_namespace', f'agent_{i}',
            ],
        ))

    # Spawn target robots at diverse positions
    for i in range(max_num_targets):
        tx = 2.0 + (i % 4) * 1.2
        ty = 2.0 + (i // 4) * 1.2
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', f'target_{i}',
                '-file', target_sdf,
                '-x', str(tx),
                '-y', str(ty),
                '-z', '0.15',
                '-robot_namespace', f'target_{i}',
            ],
        ))

    # Central controller (delayed to allow Gazebo to settle)
    cc_node = Node(
        package='central_controller',
        executable='central_controller',
        name='central_controller',
        output='screen',
        parameters=[{
            'num_robots':      num_robots,
            'num_targets':     num_targets,
            'max_num_targets': max_num_targets,
            'tau':             float(tau),
            'max_linear_vel':  float(max_lin),
            'max_angular_vel': float(max_ang),
            'seed':            42,
            'arena_half_size': 5.0,
            'params_file':     mbam_params,
            'model_path':      mbam_ckpt,
        }],
    )
    actions.append(TimerAction(period=6.0, actions=[cc_node]))

    # Agent nodes (one per robot)
    for i in range(num_robots):
        agent_node = Node(
            package='limo_agent',
            executable='agent_node',
            name=f'limo_agent_{i}',
            output='screen',
            parameters=[
                os.path.join(limo_agent_share, 'config', 'agent_params.yaml'),
                {
                    'robot_id':        i,
                    'max_num_targets': max_num_targets,
                    'tau':             float(tau),
                },
            ],
        )
        actions.append(TimerAction(period=6.0, actions=[agent_node]))

    # Target nodes (one per target)
    for i in range(max_num_targets):
        target_node = Node(
            package='limo_target',
            executable='target_node',
            name=f'limo_target_{i}',
            output='screen',
            parameters=[
                os.path.join(limo_target_share, 'config', 'target_params.yaml'),
                {
                    'robot_id': i,
                    'seed':     42,
                },
            ],
        )
        actions.append(TimerAction(period=6.0, actions=[target_node]))

    return actions


def generate_launch_description():
    mbam_share = get_package_share_directory('mbam_gazebo_tracking')
    gazebo_share = get_package_share_directory('gazebo_ros')
    world = os.path.join(mbam_share, 'worlds', 'mbam_tracking.world')
    rviz_cfg = os.path.join(mbam_share, 'rviz', 'mbam_tracking.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'map'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui',            default_value='true'),
        DeclareLaunchArgument('use_rviz',       default_value='true'),
        DeclareLaunchArgument('num_robots',     default_value='2'),
        DeclareLaunchArgument('num_targets',    default_value='3'),
        DeclareLaunchArgument('max_num_targets', default_value='7'),
        DeclareLaunchArgument('tau',            default_value='1.0'),
        DeclareLaunchArgument('max_linear_vel', default_value='0.15'),
        DeclareLaunchArgument('max_angular_vel', default_value='0.40'),

        gazebo,
        static_tf,
        OpaqueFunction(function=_spawn_and_launch),
        rviz,
    ])
