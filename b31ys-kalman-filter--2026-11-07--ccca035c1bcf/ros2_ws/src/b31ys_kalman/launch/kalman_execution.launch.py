from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'student_name',
            default_value='student'
        ),

        DeclareLaunchArgument(
            'level',
            default_value='beng',
            choices=['beng', 'msc']
        ),

        Node(
            package='b31ys_kalman',
            executable='kalman_filter_template',
            name='kalman_filter',
            output='screen',
            parameters=[{
                'student_name':
                    LaunchConfiguration('student_name'),
                'level':
                    LaunchConfiguration('level'),
                'use_sim_time': True,
            }]
        ),

        Node(
            package='b31ys_kalman',
            executable='student_trajectory',
            name='student_trajectory',
            output='screen',
            parameters=[{
                'student_name':
                    LaunchConfiguration('student_name'),
                'use_sim_time': True,
            }]
        ),
    ])
