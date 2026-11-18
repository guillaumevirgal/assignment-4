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
