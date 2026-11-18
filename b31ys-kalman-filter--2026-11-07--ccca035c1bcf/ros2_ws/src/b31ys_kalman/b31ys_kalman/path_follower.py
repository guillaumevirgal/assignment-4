#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

from tf2_ros import Buffer, TransformListener, TransformException


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # -----------------------------------------------------
        # Parameters
        # -----------------------------------------------------

        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 1.0)

        self.waypoint_tolerance = (
            self.get_parameter('waypoint_tolerance')
            .get_parameter_value()
            .double_value
        )

        self.max_linear_speed = (
            self.get_parameter('max_linear_speed')
            .get_parameter_value()
            .double_value
        )

        self.max_angular_speed = (
            self.get_parameter('max_angular_speed')
            .get_parameter_value()
            .double_value
        )

        # -----------------------------------------------------
        # Planned path
        # -----------------------------------------------------

        path_qos = QoSProfile(depth=1)

        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            path_qos
        )

        # -----------------------------------------------------
        # Robot velocity
        # -----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # -----------------------------------------------------
        # TF
        # -----------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.path = []
        self.target_index = 0

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Path follower ready.'
        )

    def path_callback(self, msg):

        self.path = msg.poses
        self.target_index = 0

        self.get_logger().info(
            f'Received path with {len(self.path)} points.'
        )

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def yaw_from_quaternion(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def stop(self):

        self.cmd_pub.publish(
            Twist()
        )

    def control_loop(self):

        if not self.path:
            self.stop()
            return

        if self.target_index >= len(self.path):

            self.stop()

            self.get_logger().info(
                'Goal reached.'
            )

            self.path = []

            return

        # -----------------------------------------------------
        # Current robot pose
        # -----------------------------------------------------

        try:

            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

        except TransformException:
            self.stop()
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        robot_yaw = self.yaw_from_quaternion(
            tf.transform.rotation
        )

        # -----------------------------------------------------
        # Target waypoint
        # -----------------------------------------------------

        target = (
            self.path[
                self.target_index
            ].pose.position
        )

        dx = target.x - robot_x
        dy = target.y - robot_y

        distance = math.hypot(
            dx,
            dy
        )

        # Waypoint reached
        if distance < self.waypoint_tolerance:

            self.target_index += 1

            return

        desired_yaw = math.atan2(
            dy,
            dx
        )

        yaw_error = self.normalize_angle(
            desired_yaw - robot_yaw
        )

        cmd = Twist()

        # -----------------------------------------------------
        # Rotate towards waypoint
        # -----------------------------------------------------

        cmd.angular.z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                1.5 * yaw_error
            )
        )

        # -----------------------------------------------------
        # Move when approximately aligned
        # -----------------------------------------------------

        if abs(yaw_error) < 0.35:

            cmd.linear.x = min(
                self.max_linear_speed,
                0.5 * distance
            )

        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(
            cmd
        )


def main(args=None):

    rclpy.init(args=args)

    node = PathFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()