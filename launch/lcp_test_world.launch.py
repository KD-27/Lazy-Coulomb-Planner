#!/usr/bin/env python3
# Copyright 2026 Kaveesha Dhananjaya
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

r"""
lcp_test_world.launch.py.

Launches a TurtleBot3 Gazebo simulation in one of the three LCP test worlds.

Usage:
  ros2 launch lazy_coulomb_planner lcp_test_world.launch.py world:=single_box
  ros2 launch lazy_coulomb_planner lcp_test_world.launch.py world:=wall_obstacle
  ros2 launch lazy_coulomb_planner lcp_test_world.launch.py world:=narrow_gap

Then in separate terminals:
  ros2 launch nav2_bringup localization_launch.py \\
    use_sim_time:=true \\
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml

  ros2 launch nav2_bringup navigation_launch.py \\
    use_sim_time:=true \\
    params_file:=$HOME/ros2_ws/src/lazy_coulomb_planner/config/nav2_params.yaml

  ros2 launch nav2_bringup rviz_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lazy_coulomb_planner')

    # ── Declare arguments ─────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='single_box',
        description='World to load: single_box | wall_obstacle | narrow_gap',
    )

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='1.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='5.0')

    # ── World file path ───────────────────────────────────────────────────────
    world_file = PathJoinSubstitution([
        pkg_share, 'worlds',
        [LaunchConfiguration('world'), '.world'],
    ])

    # ── Gazebo server ─────────────────────────────────────────────────────────
    gzserver = Node(
        package='gazebo_ros',
        executable='gzserver',
        output='screen',
        arguments=[
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
    )

    # ── Gazebo client (GUI) ───────────────────────────────────────────────────
    gzclient = Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen',
    )

    # ── TurtleBot3 URDF ───────────────────────────────────────────────────────
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_burger', 'model.sdf',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('turtlebot3_description'),
                    'urdf', 'turtlebot3_burger.urdf'),
            ).read(),
        }],
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'burger',
            '-file', urdf_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01',
        ],
    )

    return LaunchDescription([
        world_arg,
        x_pose_arg,
        y_pose_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
