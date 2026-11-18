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
        # self.Q = np.diag([0.02, 0.02, 0.01]) # Original template value (kept for reference)
        
        self.Q = np.diag([0.002, 0.002, 0.001])

        # TODO:
        # Tune GPS measurement covariance.
        # self.R_gps = np.diag([0.25, 0.25]) # Original template value (kept for reference)
        
        self.R_gps = np.diag([0.03, 0.03])

        # TODO MSc:
        # Tune noisy odometry measurement covariance.
        # self.R_odom = np.diag([1.0, 1.0, 0.4]) # Original template value (kept for reference)
        
        self.R_odom = np.diag([10.0, 10.0, 4.0])


        self.gps_updates = 0  # GPS corrections counter
        self.odom_updates = 0 # odometry corrections counter

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



        theta = float(self.x[2, 0]) # current state's theta
        v = self.v # commanded linear speed from student_trajectory
        omega = self.omega # commanded turn rate

        x_pred = np.array([ # prediction of the next state given the commanded actions and current state
            [self.x[0, 0] + v * math.cos(theta) * dt], # x prediction
            [self.x[1, 0] + v * math.sin(theta) * dt], # y prediction
            [wrap_angle(theta + omega * dt)]           # theta prediction
        ])

        

        F = np.array([ # EKF linearisation: motion jacobian F
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0]
        ])

        
        P_pred = F @ self.P @ F.T + self.Q # covariance prediction


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

            
            H_gps = np.array([ # GPS measurement matrix H
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0]
            ])

            # Calculate innovation.
            innovation = z_gps - H_gps @ self.x

            self.ekf_correct(innovation, H_gps, self.R_gps)  # Calculate innovation covariance, Kalman gain and correct self.x and self.P.
            self.gps_updates += 1

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


            H_odom = np.eye(3) # Create odomotery identity matrix

            innovation = z_odom - H_odom @ self.x # measured odom - estimated odom

            
            

            innovation[2, 0] = wrap_angle(innovation[2, 0]) # Normalise angular innovation between -pi and pi

            self.ekf_correct(innovation, H_odom, self.R_odom)
            self.odom_updates += 1


        self.get_logger().info( # ROS2 logs
            f'x={self.x[0, 0]:.2f} y={self.x[1, 0]:.2f} ' # estimated position
            f'theta={self.x[2, 0]:.2f} | '                # estimated angle
            f'GPS updates={self.gps_updates} '   # number of gps updates
            f'odom updates={self.odom_updates}', # number of odom updates
            throttle_duration_sec=5.0 # printed at most every 5 s.
        )

        self.publish_estimate()

    def ekf_correct(self, innovation, H, R):
        """
        MSc: generic EKF measurement update, shared by the
        GPS and odometry corrections.

        innovation : (m, 1) z - h(x), angles already wrapped
        H          : (m, 3) measurement Jacobian
        R          : (m, m) measurement noise covariance
        """

        
        S = H @ self.P @ H.T + R # Innovation covariance

        
        K = self.P @ H.T @ np.linalg.inv(S) # Kalman gain 

        
        self.x = self.x + K @ innovation # State correction.
        self.x[2, 0] = wrap_angle(self.x[2, 0])

        
        


        I_KH = np.eye(3) - K @ H  # (I - K H)
        self.P = I_KH @ self.P # Covariance correction
        # self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T # Joseph form mentioned in the robot_localization paper


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

        
        
        cov = np.zeros((6, 6)) # Create 6x6 matrix [x, y, z, roll, pitch, yaw]
        idx = [0, 1, 5]  # 0 = x; 1 = y; 5 = yaw/theta
        for i in range(3):
            for j in range(3):
                cov[idx[i], idx[j]] = self.P[i, j]
        msg.pose.covariance = [float(c) for c in cov.flatten()] # Formatted for the ROS msg field
       

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
