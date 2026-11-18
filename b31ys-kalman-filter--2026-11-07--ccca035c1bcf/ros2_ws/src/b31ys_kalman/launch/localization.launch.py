"""Localisation in the saved SLAM map with AMCL (Part 3).

map_server loads the saved map, AMCL estimates the robot pose from the
lidar and odometry and publishes map -> odom.

    ros2 launch b31ys_kalman localization.launch.py
    ros2 launch b31ys_kalman localization.launch.py map:=/output/my_map.yaml

The robot is assumed to start at the map origin (where the map was
started). If not, use "2D Pose Estimate" in RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('b31ys_kalman')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_share, 'maps', 'my_map.yaml']),
            description='Saved map (.yaml).'),
        DeclareLaunchArgument(
            'amcl_params',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'amcl_params.yaml']),
            description='AMCL parameter file.'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map'),
                         'use_sim_time': True}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('amcl_params'), {'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True,
                         'node_names': ['map_server', 'amcl'],
                         'use_sim_time': True}],
        ),
    ])
