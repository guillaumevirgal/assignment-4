"""Localisation + PRM planning + path following + RViz (Part 3).

    ros2 launch b31ys_kalman navigation.launch.py sim:=true

Then click "2D Goal Pose" in RViz: the PRM plans from the robot's
localised pose and the path follower drives there.

Pipeline:
    map_server (saved map) --/map--> AMCL, PRM planner
    AMCL: lidar + odometry -> map -> odom TF
    PRM planner: /goal_pose -> /planned_path (start = robot pose)
    path follower: /planned_path + map -> base_link TF -> /cmd_vel
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
            default_value=PathJoinSubstitution([pkg_share, 'maps', 'my_map.yaml']),
            description='Saved map (.yaml).'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Also start the Gazebo simulation.'),
        DeclareLaunchArgument('gz_headless_mode', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('num_samples', default_value='800'),
        DeclareLaunchArgument('k_neighbours', default_value='12'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument('robot_radius', default_value='0.18'),
        DeclareLaunchArgument('safety_margin', default_value='0.10'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.25'),
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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'localization.launch.py'])),
        launch_arguments={'map': cfg('map')}.items(),
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
            'results_csv': cfg('results_csv'),
            'use_sim_time': True,
        }],
    )

    follower = Node(
        package='b31ys_kalman',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[{
            'max_linear_speed': ParameterValue(cfg('max_linear_speed'), value_type=float),
            'use_sim_time': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='log',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'navigation.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(cfg('rviz')),
    )

    return LaunchDescription(args + [simulation, localization, planner, follower, rviz])
