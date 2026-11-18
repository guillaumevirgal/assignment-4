import math
import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2)
    )


class KalmanFilter(Node):

    def __init__(self):
        super().__init__('kalman_filter')

        self.declare_parameter('student_name', 'student')
        self.declare_parameter('level', 'bsc')
        self.declare_parameter('gps_type', 'nav_msgs/msg/Odometry')

        self.level = self.get_parameter('level').value

        # State: [x, y, theta]
        self.x = np.zeros(3)
        self.P = np.eye(3)

        # Process covariance
        self.Q = np.diag([0.02, 0.02, 0.01])

        # GPS measurement covariance
        self.R_gps = np.diag([0.25, 0.25])

        # Noisy odometry measurement covariance
        self.R_odom = np.diag([1.0, 1.0, 0.4])

        self.v = 0.0
        self.omega = 0.0

        self.gps_measurement = None
        self.odom_measurement = None

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        gps_type = self.get_parameter('gps_type').value

        if gps_type == 'nav_msgs/msg/Odometry':
            gps_msg_type = Odometry
        elif gps_type == 'geometry_msgs/msg/PoseStamped':
            gps_msg_type = PoseStamped
        else:
            raise ValueError('Unsupported GPS message type')

        self.create_subscription(
            gps_msg_type, '/gps', self.gps_callback, 10
        )

        if self.level == 'msc':
            self.create_subscription(
                Odometry, '/odom1', self.odom_callback, 10
            )

        self.estimate_pub = self.create_publisher(
            Odometry, '/kalman/odom', 10
        )

        self.previous_time = self.get_clock().now()
        self.create_timer(0.1, self.update_kalman)

    def cmd_callback(self, msg):
        self.v = float(msg.linear.x)
        self.omega = float(msg.angular.z)

    def gps_callback(self, msg):
        if isinstance(msg, Odometry):
            p = msg.pose.pose.position
        else:
            p = msg.pose.position

        self.gps_measurement = np.array([p.x, p.y])

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        theta = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        self.odom_measurement = np.array([
            p.x, p.y, theta
        ])

    def update_kalman(self):

        current_time = self.get_clock().now()
        dt = (
            current_time - self.previous_time
        ).nanoseconds * 1e-9

        self.previous_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        # =====================================
        # TASK 1: PREDICTION
        # =====================================

        # Use:
        # self.x
        # self.v
        # self.omega
        # dt
        #
        # Differential drive:
        #
        # x_next = x + v*cos(theta)*dt
        # y_next = y + v*sin(theta)*dt
        # theta_next = theta + omega*dt
        #
        # TODO: Calculate predicted state

        x_pred = np.zeros(3)

        # TODO: Calculate motion Jacobian F

        F = np.eye(3)

        # TODO: Predict covariance

        P_pred = np.zeros((3, 3))

        self.x = x_pred
        self.P = P_pred

        # =====================================
        # TASK 2: GPS CORRECTION
        # =====================================

        if self.gps_measurement is not None:

            z_gps = self.gps_measurement.copy()

            # Consume each measurement once
            self.gps_measurement = None

            # TODO:
            # Define measurement matrix H
            # Calculate innovation
            # Calculate innovation covariance S
            # Calculate Kalman gain K
            # Correct state
            # Correct covariance

            pass

        # =====================================
        # TASK 3: MSc ONLY
        # =====================================

        if self.level == 'msc':

            if self.odom_measurement is not None:

                z_odom = self.odom_measurement.copy()
                self.odom_measurement = None

                # TODO:
                # Correct using x, y, theta
                # from noisy odometry
                #
                # Remember angle wrapping.

                pass

        # =====================================
        # PUBLISH ESTIMATE
        # =====================================

        self.publish_estimate()

    def publish_estimate(self):

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_kalman'

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])

        theta = float(self.x[2])

        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(theta / 2.0),
            w=math.cos(theta / 2.0)
        )

        self.estimate_pub.publish(msg)


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
