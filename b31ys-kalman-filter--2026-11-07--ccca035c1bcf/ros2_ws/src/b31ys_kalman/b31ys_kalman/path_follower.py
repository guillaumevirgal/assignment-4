#!/usr/bin/env python3
"""Path follower (CW1), used in CW2 Part 3 to follow the PRM path.

Proportional controller: turn towards the next waypoint, drive when
roughly aligned. The robot pose is map -> base_link, provided by AMCL
(map -> odom) and the robot's odometry (odom -> base_link).

CW2 additions (marked "CW2:"):
* lidar safety stop: no forward motion if something is closer than
  safety_distance in front; slow down below slow_distance.
* /path_follower/status (std_msgs/String): following, reached, blocked,
  idle. Used to send the next goal in Part 4.
* does not publish on /cmd_vel while idle, so teleop still works.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

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
        # CW2: status and lidar safety stop
        # -----------------------------------------------------

        self.declare_parameter('safety_distance', 0.30)
        self.declare_parameter('slow_distance', 0.60)
        self.declare_parameter('front_half_angle_deg', 35.0)
        self.safety_distance = self.get_parameter('safety_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, '/path_follower/status', status_qos)
        self.status = ''
        self.set_status('idle')

        self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_callback,
            qos_profile_sensor_data)
        self.front_distance = math.inf  # closest obstacle ahead (m)
        self.laser_yaw = None           # laser frame yaw in base_link
        self.idle_stop_sent = True

        # -----------------------------------------------------
        # TF
        # -----------------------------------------------------

        self.tf_buffer = Buffer()

        # CW2: own thread, so lookups never wait on the control loop
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True
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

        # CW2: an empty path (planner found nothing) means stop
        if self.path:
            self.idle_stop_sent = False
            self.set_status('following')
        else:
            self.stop()
            self.set_status('idle')

    # CW2
    def set_status(self, status):
        if status != self.status:
            self.status = status
            self.status_pub.publish(String(data=status))

    # CW2: closest obstacle in front of the robot, from the lidar
    def scan_callback(self, msg):
        if self.laser_yaw is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id, rclpy.time.Time())
                self.laser_yaw = self.yaw_from_quaternion(tf.transform.rotation)
            except TransformException:
                return
        closest = math.inf
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = self.normalize_angle(
                msg.angle_min + i * msg.angle_increment + self.laser_yaw)
            if abs(angle) <= self.front_half_angle:
                closest = min(closest, r)
        self.front_distance = closest

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
            # CW2: stop once, then stay quiet so teleop can drive
            if not self.idle_stop_sent:
                self.stop()
                self.idle_stop_sent = True
            return

        if self.target_index >= len(self.path):

            self.stop()

            self.get_logger().info(
                'Goal reached.'
            )

            self.path = []
            self.set_status('reached')

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

        # -----------------------------------------------------
        # CW2: lidar safety stop (turning on the spot is allowed)
        # -----------------------------------------------------

        if cmd.linear.x > 0.0:
            if self.front_distance < self.safety_distance:
                cmd.linear.x = 0.0
                if self.status != 'blocked':
                    self.get_logger().warning(
                        f'Obstacle {self.front_distance:.2f} m ahead: stopped.')
                self.set_status('blocked')
            else:
                if self.front_distance < self.slow_distance:
                    cmd.linear.x *= (
                        (self.front_distance - self.safety_distance)
                        / (self.slow_distance - self.safety_distance))
                self.set_status('following')

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

        if rclpy.ok():  # CW2: context may already be shut down
            node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()