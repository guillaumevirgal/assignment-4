"""Full system, single launch file (Part 4).

Starts the simulation, localisation (AMCL on the saved map), the PRM
planner, the path follower, RViz, and the goal sequencer that visits the
patient rooms in config/rooms.yaml one after the other.

    ros2 launch b31ys_kalman hospital.launch.py
    ros2 launch b31ys_kalman hospital.launch.py tour:=false     # click goals yourself
    ros2 launch b31ys_kalman hospital.launch.py sim:=false      # real robot / sim already running

System:
    map_server --/map--> AMCL, PRM planner
    AMCL (lidar + odometry) --TF map->odom--> planner, follower
    goal_sequencer --/goal_pose--> PRM planner --/planned_path--> path_follower --/cmd_vel--> robot
    path_follower --/path_follower/status--> goal_sequencer (next room)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('b31ys_kalman')
    cfg = LaunchConfiguration

    args = [
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_share, 'maps', 'my_map.yaml']),
            description='Saved map (.yaml).'),
        DeclareLaunchArgument(
            'rooms',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'rooms.yaml']),
            description='Patient rooms and visit order.'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='Start the Gazebo simulation.'),
        DeclareLaunchArgument('gz_headless_mode', default_value='False',
                              description='False: show the Gazebo window.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('tour', default_value='true',
                              description='Visit the rooms automatically.'),
        DeclareLaunchArgument('results_csv', default_value='',
                              description='Append planning statistics to this CSV.'),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'map': cfg('map'),
            'sim': cfg('sim'),
            'gz_headless_mode': cfg('gz_headless_mode'),
            'rviz': cfg('rviz'),
            'results_csv': cfg('results_csv'),
        }.items(),
    )

    sequencer = Node(
        package='b31ys_kalman',
        executable='goal_sequencer',
        name='goal_sequencer',
        output='screen',
        parameters=[cfg('rooms'), {'use_sim_time': True}],
        condition=IfCondition(cfg('tour')),
    )

    return LaunchDescription(args + [navigation, sequencer])
