"""Headless variant of Nav2's planning_benchmark_bringup.py: map_server + planner_server only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('params')
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        DeclareLaunchArgument('map'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_link']),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}, {'topic_name': 'map'}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
             parameters=[{'autostart': True}, {'node_names': ['map_server', 'planner_server']}]),
    ])
