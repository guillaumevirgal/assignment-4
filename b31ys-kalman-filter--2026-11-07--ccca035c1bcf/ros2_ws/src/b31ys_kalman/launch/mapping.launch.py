"""Map husarion_world with slam_toolbox.

Starts (optionally) the ROSbot simulation, slam_toolbox in online async
mode with config/slam_params.yaml, and RViz showing the map being built.

    ros2 launch b31ys_kalman mapping.launch.py
    ros2 launch b31ys_kalman mapping.launch.py sim:=false   # sim already running
    ros2 launch b31ys_kalman mapping.launch.py rviz:=false gz_headless_mode:=False

Drive with teleop and save with the Nav2 map saver: scripts/build_map.sh
does all of it in one go.
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

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    gz_headless_mode = LaunchConfiguration('gz_headless_mode')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Husarion's simulation launch is given rviz:=false. In ROS 2 Humble,
    # arguments of an included launch file leak into this file, so our
    # RViz is listed BEFORE the simulation (its rviz condition is
    # evaluated before the leak).
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbot_gazebo'),
                'launch', 'simulation.launch.py'
            ])
        ),
        launch_arguments={
            'robot_model': 'rosbot',
            'rviz': 'false',
            'gz_headless_mode': gz_headless_mode,
        }.items(),
        condition=IfCondition(sim),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [pkg_share, 'config', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Start the Gazebo simulation as well.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz with the map, laser scan and robot.'),
        DeclareLaunchArgument(
            'gz_headless_mode', default_value='True',
            description='True: no Gazebo window. False: show Gazebo.'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'slam_params.yaml']),
            description='slam_toolbox parameter file.'),
        rviz_node,
        simulation,
        slam,
    ])
