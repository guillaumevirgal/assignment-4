"""PRM planning on the saved SLAM map (Part 2).

Starts the map server with the saved map, the PRM planner and RViz.
Optionally also the simulation and the CW1 path follower.

    ros2 launch b31ys_kalman prm_planner.launch.py sim:=true
    ros2 launch b31ys_kalman prm_planner.launch.py map:=/output/my_map.yaml
    ros2 launch b31ys_kalman prm_planner.launch.py goal_x:=3.0 goal_y:=1.5

Then click "2D Goal Pose" in RViz to plan to that point.

map -> odom: until localisation is added (Part 3), static_map_odom:=true
publishes an identity map -> odom transform. This is valid in simulation
because slam_toolbox puts the map origin where the robot spawned, which is
also where odometry starts. Set static_map_odom:=false once AMCL or
slam_toolbox localisation provides map -> odom.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('b31ys_kalman')
    cfg = LaunchConfiguration

    args = [
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [pkg_share, 'maps', 'my_map.yaml']),
            description='Saved map (.yaml) from build_map.sh.'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Also start the Gazebo simulation.'),
        DeclareLaunchArgument('gz_headless_mode', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='Also run the path follower (drives the robot).'),
        DeclareLaunchArgument('static_map_odom', default_value='true',
                              description='Identity map -> odom TF (no localisation yet).'),
        DeclareLaunchArgument('num_samples', default_value='800'),
        DeclareLaunchArgument('k_neighbours', default_value='12'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument('robot_radius', default_value='0.18'),
        DeclareLaunchArgument('safety_margin', default_value='0.10'),
        DeclareLaunchArgument('goal_x', default_value='nan',
                              description='Optional goal planned at start-up.'),
        DeclareLaunchArgument('goal_y', default_value='nan'),
        DeclareLaunchArgument('results_csv', default_value='',
                              description='Append planning statistics to this CSV.'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rosbot_gazebo'), 'launch', 'simulation.launch.py'])),
        launch_arguments={
            'robot_model': 'rosbot',
            'rviz': 'false',
            'gz_headless_mode': cfg('gz_headless_mode'),
        }.items(),
        condition=IfCondition(cfg('sim')),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': cfg('map'), 'use_sim_time': True}],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'],
                     'use_sim_time': True}],
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=IfCondition(cfg('static_map_odom')),
    )

    planner = Node(
        package='b31ys_kalman',
        executable='prm_planner',
        name='prm_planner',
        output='screen',
        parameters=[{
            'num_samples': ParameterValue(cfg('num_samples'), value_type=int),
            'k_neighbours': ParameterValue(cfg('k_neighbours'), value_type=int),
            'random_seed': ParameterValue(cfg('random_seed'), value_type=int),
            'robot_radius': ParameterValue(cfg('robot_radius'), value_type=float),
            'safety_margin': ParameterValue(cfg('safety_margin'), value_type=float),
            'goal_x': ParameterValue(cfg('goal_x'), value_type=float),
            'goal_y': ParameterValue(cfg('goal_y'), value_type=float),
            'results_csv': cfg('results_csv'),
            'use_sim_time': True,
        }],
    )

    follower = Node(
        package='b31ys_kalman',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(cfg('follow')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_prm',
        output='log',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'prm.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(cfg('rviz')),
    )

    return LaunchDescription(args + [
        simulation, map_server, lifecycle, map_to_odom, planner, follower, rviz,
    ])
