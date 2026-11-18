#!/usr/bin/env bash
set -e

cd ~/ros2_ws/src/b31ys_kalman

echo "Creating student-only B31YS assignment..."

# Remove previous prototype and instructor material.
rm -f b31ys_kalman/instructor_ekf.py
rm -f b31ys_kalman/assignment_simulator.py
rm -f b31ys_kalman/simulator.py
rm -f b31ys_kalman/bsc_estimator.py
rm -f b31ys_kalman/msc_estimator.py
rm -f b31ys_kalman/trajectory_plotter.py

rm -rf b31ys_kalman/students
rm -rf instructor_private
rm -rf students/bsc

rm -f launch/instructor_bsc.launch.py
rm -f launch/instructor_msc.launch.py
rm -f launch/assignment_sensors.launch.py
rm -f launch/bsc.launch.py
rm -f launch/msc.launch.py

# Remove instructor materials outside the package.
rm -rf ~/b31ys_instructor_private

mkdir -p b31ys_kalman
mkdir -p students/beng
mkdir -p students/msc
mkdir -p launch
mkdir -p resource

touch b31ys_kalman/__init__.py
touch resource/b31ys_kalman

# ---------------------------------------------
# ROS 2 PACKAGE CONFIGURATION
# ---------------------------------------------

cat > setup.py <<'PY'
from setuptools import setup
from glob import glob

package_name = 'b31ys_kalman'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml', 'README.md']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/students/beng',
            ['students/beng/README.md']
        ),
        (
            'share/' + package_name + '/students/msc',
            ['students/msc/README.md']
        ),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='B31YS Teaching Team',
    maintainer_email='instructor@example.com',
    description='B31YS Kalman Filter Assignment ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_sensors = b31ys_kalman.fake_sensors:main',
            'student_trajectory = b31ys_kalman.student_trajectory:main',
            'kalman_filter_template = '
            'b31ys_kalman.kalman_filter_template:main',
        ],
    },
)
PY

cat > setup.cfg <<'CFG'
[develop]
script_dir=$base/lib/b31ys_kalman
[install]
install_scripts=$base/lib/b31ys_kalman
CFG

cat > package.xml <<'XML'
<?xml version="1.0"?>
<package format="3">
  <name>b31ys_kalman</name>
  <version>2.0.0</version>
  <description>B31YS Kalman Filter Assignment</description>

  <maintainer email="instructor@example.com">
    B31YS Teaching Team
  </maintainer>

  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>numpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
XML

# ---------------------------------------------
# FAKE GPS AND NOISY ODOMETRY
# ---------------------------------------------

cat > b31ys_kalman/fake_sensors.py <<'PY'
import math
import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def quaternion_from_yaw(theta):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(theta / 2.0)
    q.w = math.cos(theta / 2.0)
    return q


class FakeSensors(Node):

    def __init__(self):
        super().__init__('b31ys_fake_sensors')

        self.declare_parameter(
            'reference_topic', '/odometry/wheels'
        )

        self.declare_parameter('gps_sigma', 0.15)
        self.declare_parameter('odom_sigma', 0.35)
        self.declare_parameter('odom_yaw_sigma', 0.10)
        self.declare_parameter('drift_rate', 0.025)
        self.declare_parameter('seed', 42)

        self.gps_sigma = float(
            self.get_parameter('gps_sigma').value
        )
        self.odom_sigma = float(
            self.get_parameter('odom_sigma').value
        )
        self.odom_yaw_sigma = float(
            self.get_parameter('odom_yaw_sigma').value
        )
        self.drift_rate = float(
            self.get_parameter('drift_rate').value
        )

        self.rng = np.random.default_rng(
            int(self.get_parameter('seed').value)
        )

        self.reference = None
        self.drift = np.zeros(3)

        self.gps_pub = self.create_publisher(
            PoseStamped, '/gps', 10
        )

        self.noisy_pub = self.create_publisher(
            Odometry, '/odom1', 10
        )

        self.reference_pub = self.create_publisher(
            Odometry, '/odom', 10
        )

        reference_topic = self.get_parameter(
            'reference_topic'
        ).value

        self.create_subscription(
            Odometry,
            reference_topic,
            self.reference_callback,
            10
        )

        self.create_timer(1.0, self.publish_gps)
        self.create_timer(0.05, self.publish_noisy)

        self.last_noisy_time = None

        self.get_logger().info(
            'Fake GPS and noisy odometry started'
        )

    def reference_callback(self, msg):
        self.reference = msg
        self.reference_pub.publish(msg)

    def reference_state(self):
        p = self.reference.pose.pose.position
        q = self.reference.pose.pose.orientation

        return np.array([
            p.x,
            p.y,
            yaw_from_quaternion(q)
        ])

    def publish_gps(self):
        if self.reference is None:
            return

        x = self.reference_state()

        z = x[:2] + self.rng.normal(
            0.0, self.gps_sigma, 2
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.position.x = float(z[0])
        msg.pose.position.y = float(z[1])
        msg.pose.orientation.w = 1.0

        self.gps_pub.publish(msg)

    def publish_noisy(self):
        if self.reference is None:
            return

        now = self.get_clock().now()

        if self.last_noisy_time is None:
            dt = 0.05
        else:
            dt = max(
                0.0,
                min(
                    (now - self.last_noisy_time).nanoseconds
                    * 1e-9,
                    0.2
                )
            )

        self.last_noisy_time = now

        x = self.reference_state()

        self.drift += self.rng.normal(
            0.0,
            self.drift_rate * math.sqrt(dt),
            3
        )

        noise = self.rng.normal(
            0.0,
            [
                self.odom_sigma,
                self.odom_sigma,
                self.odom_yaw_sigma
            ]
        )

        z = x + self.drift + noise

        z[2] = (
            (z[2] + math.pi) % (2.0 * math.pi)
        ) - math.pi

        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_noisy'

        msg.pose.pose.position.x = float(z[0])
        msg.pose.pose.position.y = float(z[1])

        msg.pose.pose.orientation = quaternion_from_yaw(
            z[2]
        )

        msg.pose.covariance[0] = self.odom_sigma**2
        msg.pose.covariance[7] = self.odom_sigma**2
        msg.pose.covariance[35] = self.odom_yaw_sigma**2

        self.noisy_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeSensors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

# ---------------------------------------------
# STUDENT-SPECIFIC TRAJECTORY
# ---------------------------------------------

cat > b31ys_kalman/student_trajectory.py <<'PY'
import hashlib
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StudentTrajectory(Node):

    def __init__(self):
        super().__init__('student_trajectory')

        self.declare_parameter('student_name', 'student')

        name = self.get_parameter(
            'student_name'
        ).value

        digest = hashlib.sha256(
            name.encode('utf-8')
        ).digest()

        seed = int.from_bytes(digest[:4], 'little')

        self.phase = seed % 4
        self.speed_scale = 0.80 + (seed % 21) / 100.0

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.start_time = None

        self.create_timer(0.1, self.update_trajectory)

        self.get_logger().info(
            f'Student trajectory: {name}'
        )

    def update_trajectory(self):
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        t = max(
            0.0,
            (now - self.start_time).nanoseconds * 1e-9
        )

        t = (t + self.phase * 7.0) % 60.0

        msg = Twist()

        if t < 12.0:
            msg.linear.x = 0.15
            msg.angular.z = 0.0

        elif t < 22.0:
            msg.linear.x = 0.12
            msg.angular.z = 0.35

        elif t < 37.0:
            msg.linear.x = 0.18
            msg.angular.z = -0.20

        elif t < 49.0:
            msg.linear.x = 0.12
            msg.angular.z = -0.40

        else:
            msg.linear.x = 0.16
            msg.angular.z = 0.15

        msg.linear.x *= self.speed_scale

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = StudentTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

# ---------------------------------------------
# STUDENT KALMAN FILTER TEMPLATE
# NO SOLUTION INCLUDED
# ---------------------------------------------

cat > b31ys_kalman/kalman_filter_template.py <<'PY'
import math
import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


def wrap_angle(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


class KalmanFilter(Node):

    def __init__(self):
        super().__init__('kalman_filter')

        self.declare_parameter('student_name', 'student')
        self.declare_parameter('level', 'beng')

        self.level = self.get_parameter('level').value

        # Robot state: [x, y, theta]
        self.x = np.zeros((3, 1))

        # State covariance
        self.P = np.eye(3)

        # TODO:
        # Tune process noise covariance Q.
        self.Q = np.diag([0.02, 0.02, 0.01])

        # TODO:
        # Tune GPS measurement covariance.
        self.R_gps = np.diag([0.25, 0.25])

        # TODO MSc:
        # Tune noisy odometry measurement covariance.
        self.R_odom = np.diag([1.0, 1.0, 0.4])

        self.v = 0.0
        self.omega = 0.0

        self.gps_measurement = None
        self.odom_measurement = None

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/gps',
            self.gps_callback,
            10
        )

        if self.level == 'msc':
            self.create_subscription(
                Odometry,
                '/odom1',
                self.odom_callback,
                10
            )

        self.publisher = self.create_publisher(
            Odometry,
            '/kalman/odom',
            10
        )

        self.previous_time = None

        self.create_timer(0.1, self.update_kalman)

        self.get_logger().info(
            f'Kalman template started: {self.level}'
        )

    def cmd_callback(self, msg):
        self.v = float(msg.linear.x)
        self.omega = float(msg.angular.z)

    def gps_callback(self, msg):
        self.gps_measurement = np.array([
            [msg.pose.position.x],
            [msg.pose.position.y]
        ])

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.odom_measurement = np.array([
            [p.x],
            [p.y],
            [yaw_from_quaternion(q)]
        ])

    def update_kalman(self):
        """
        Main Kalman filter loop.

        BEng:
            Prediction from cmd_vel.
            Correction using GPS.

        MSc:
            Prediction from cmd_vel.
            Correction using GPS and odom1.
        """

        now = self.get_clock().now()

        if self.previous_time is None:
            self.previous_time = now
            return

        dt = (
            now - self.previous_time
        ).nanoseconds * 1e-9

        self.previous_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # ====================================
        # 1. PREDICTION
        # ====================================

        # TODO:
        #
        # Use differential-drive kinematics.
        #
        # x_next = x + v*cos(theta)*dt
        # y_next = y + v*sin(theta)*dt
        # theta_next = theta + omega*dt
        #
        # Calculate predicted state x_pred.
        #
        # Calculate motion Jacobian F.
        #
        # Calculate predicted covariance P_pred.

        x_pred = None
        P_pred = None

        if x_pred is None or P_pred is None:
            raise NotImplementedError(
                'Complete the Kalman prediction step'
            )

        self.x = x_pred
        self.P = P_pred

        # ====================================
        # 2. GPS CORRECTION
        # ====================================

        if self.gps_measurement is not None:

            z_gps = self.gps_measurement.copy()

            # Consume each GPS measurement once.
            self.gps_measurement = None

            # TODO:
            #
            # Define GPS measurement matrix H.
            #
            # Calculate innovation.
            # Calculate innovation covariance.
            # Calculate Kalman gain.
            #
            # Correct self.x and self.P.

            raise NotImplementedError(
                'Complete the GPS correction'
            )

        # ====================================
        # 3. MSc: ODOMETRY CORRECTION
        # ====================================

        if (
            self.level == 'msc'
            and self.odom_measurement is not None
        ):

            z_odom = self.odom_measurement.copy()
            self.odom_measurement = None

            # TODO MSc:
            #
            # Correct using noisy odometry:
            # x, y, theta.
            #
            # Remember to normalise angular
            # innovations.

            raise NotImplementedError(
                'Complete the MSc odometry correction'
            )

        self.publish_estimate()

    def publish_estimate(self):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_kalman'

        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])

        theta = float(self.x[2, 0])

        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = KalmanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

# ---------------------------------------------
# ROBOT SENSOR LAUNCH
# ---------------------------------------------

cat > launch/robot.launch.py <<'PY'
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
PY

# ---------------------------------------------
# STUDENT ASSIGNMENT LAUNCH
# ---------------------------------------------

cat > launch/kalman_execution.launch.py <<'PY'
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
PY

echo "Student-only ROS 2 package created."
