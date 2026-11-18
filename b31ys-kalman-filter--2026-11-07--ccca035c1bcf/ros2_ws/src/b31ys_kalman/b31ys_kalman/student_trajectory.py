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
