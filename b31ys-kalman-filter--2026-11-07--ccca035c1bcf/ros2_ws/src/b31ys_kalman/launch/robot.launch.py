from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='b31ys_kalman',
            executable='fake_sensors',
            name='fake_sensors',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'reference_topic': '/odometry/wheels',
            }]
        )
    ])
