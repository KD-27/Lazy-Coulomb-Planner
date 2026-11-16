"""
Headless Gazebo with a TurtleBot3 and the Nav2 stack, without nav2_bringup.

Starts the simulator, the robot, localisation on a known map and the navigation
servers. Everything is launched as plain nodes so the planner under test can be
swapped through the parameters file alone.
"""
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    sim_time = {'use_sim_time': True}

    with open(os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf'), 'r') as f:
        robot_description = f.read()

    # LCP_SIM_WORLD selects another world (an SDF or xacro file, for example one
    # written by benchmark/generate_map.py --sdf); the default is Nav2's sandbox.
    world_source = os.environ.get('LCP_SIM_WORLD') or os.path.join(
        sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    # The world is a xacro file; Gazebo needs it expanded into a real SDF file.
    # Expand it here, before anything is launched: started as a separate process,
    # xacro can lose the race against Gazebo, which then exits for lack of a world.
    handle, world_sdf = tempfile.mkstemp(prefix='lcp_sim_', suffix='.sdf')
    os.close(handle)
    subprocess.run(
        ['xacro', '-o', world_sdf, 'headless:=True', world_source], check=True)
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_sdf], output='screen')
    remove_world = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda _: os.path.exists(world_sdf) and os.remove(world_sdf))]))

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items())

    def nav2(package, executable, name, remappings=None):
        return Node(package=package, executable=executable, name=name, output='screen',
                    parameters=[params], remappings=remappings or [])

    # Velocity chain: controller / behaviours -> cmd_vel_nav -> velocity smoother
    # -> cmd_vel_smoothed -> collision monitor -> cmd_vel.
    to_smoother = [('cmd_vel', 'cmd_vel_nav')]
    # LCP_SIM_LOCALIZATION=odometry: no AMCL; the map frame is fixed to where the
    # robot was spawned, so its pose comes from the simulator's wheel odometry.
    # Used in generated worlds, where AMCL can lock onto the wrong one of many
    # identical obstacles and the test would then measure localisation failures.
    odometry_only = os.environ.get('LCP_SIM_LOCALIZATION') == 'odometry'
    localization = ['map_server'] if odometry_only else ['map_server', 'amcl']
    if odometry_only:
        localizer = Node(
            package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
            output='screen', parameters=[sim_time],
            arguments=['--x', LaunchConfiguration('x_pose'), '--y', LaunchConfiguration('y_pose'),
                       '--frame-id', 'map', '--child-frame-id', 'odom'])
    else:
        localizer = nav2('nav2_amcl', 'amcl', 'amcl')
    navigation = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                  'velocity_smoother', 'collision_monitor', 'bt_navigator']

    # LCP_SIM_MOVERS: prefix of a world built by add_moving_obstacles.py; adds the
    # bridge for the obstacles' velocity commands and the node that drives them.
    movers = []
    movers_prefix = os.environ.get('LCP_SIM_MOVERS')
    if movers_prefix:
        movers = [
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='movers_bridge',
                 output='screen', parameters=[{'config_file': movers_prefix + '.bridge.yaml',
                                               'use_sim_time': True}]),
            ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(__file__), '..',
                                                        'move_obstacles.py'),
                                movers_prefix + '.lanes.yaml'], output='screen'),
        ]

    return LaunchDescription(movers + [
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        # The world refers to models that ship with nav2_minimal_tb3_sim, and the
        # robot's meshes use package:// URIs, resolved from the share directory.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(sim_dir)),
        remove_world, gazebo, robot,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[sim_time, {'robot_description': robot_description}]),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params, {'yaml_filename': map_yaml}]),
        localizer,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[sim_time, {'autostart': True, 'node_names': localization}]),

        nav2('nav2_controller', 'controller_server', 'controller_server', to_smoother),
        nav2('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav2('nav2_planner', 'planner_server', 'planner_server'),
        nav2('nav2_behaviors', 'behavior_server', 'behavior_server', to_smoother),
        nav2('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', to_smoother),
        nav2('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        nav2('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[sim_time, {'autostart': True, 'node_names': navigation}]),
    ])
