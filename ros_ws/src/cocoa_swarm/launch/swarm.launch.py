"""
swarm.launch.py
===============
Launches the full swarm stack for 2-drone (cf231 + cf232) coordination:
  • action_server_node  for cf231  (remapped to /cf231/drone_command)
  • action_server_node  for cf232  (remapped to /cf232/drone_command)
  • swarm_manager_node             (serves /swarm_command)

Usage:
    ros2 launch cocoa_swarm swarm.launch.py
    ros2 launch cocoa_swarm swarm.launch.py drone_ids:='["cf231","cf232"]'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── cf231 action server ───────────────────────────────────────────────
        # The action_server_node constructs its action name as /{drone_id}/drone_command
        # directly in code, so no remapping or namespace is needed here.
        Node(
            package='cocoa_llm_planner',
            executable='action_server_node',
            name='action_server_cf231',
            parameters=[{'drone_id': 'cf231'}],
            output='screen',
        ),

        # ── cf232 action server ───────────────────────────────────────────────
        Node(
            package='cocoa_llm_planner',
            executable='action_server_node',
            name='action_server_cf232',
            parameters=[{'drone_id': 'cf232'}],
            output='screen',
        ),

        # ── swarm manager ─────────────────────────────────────────────────────
        Node(
            package='cocoa_swarm',
            executable='swarm_manager_node',
            name='swarm_manager_node',
            parameters=[{
                'drone_ids': ['cf231', 'cf232'],
                'base_z': 1.0,
                'z_separation': 0.5,
                'action_timeout': 60.0,
                'feedback_rate': 5.0,
                'safety_margin': 0.8,
            }],
            output='screen',
        ),

        # ── path planner (A* obstacle avoidance) ─────────────────────────────
        Node(
            package='cocoa_path_planner',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
        ),
    ])
