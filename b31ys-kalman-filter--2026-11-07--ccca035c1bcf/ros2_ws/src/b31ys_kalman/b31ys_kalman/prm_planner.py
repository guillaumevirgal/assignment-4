#!/usr/bin/env python3

import hashlib
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid, Path

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from b31ys_path_planning.prm_core import PRMCore


class PRMPlanner(Node):

    def __init__(self):

        super().__init__('prm_planner')

        # -----------------------------------------------------
        # Parameters
        # -----------------------------------------------------

        self.declare_parameter(
            'student_name',
            'Guillaume_virgal'
        )

        self.declare_parameter(
            'num_samples',
            400
        )

        self.declare_parameter(
            'k_neighbours',
            12
        )

        self.declare_parameter(
            'random_seed',
            42
        )

        self.declare_parameter(
            'robot_radius',
            0.18
        )

        self.student_name = (
            self.get_parameter('student_name')
            .get_parameter_value()
            .string_value
        )

        self.num_samples = (
            self.get_parameter('num_samples')
            .get_parameter_value()
            .integer_value
        )

        self.k_neighbours = (
            self.get_parameter('k_neighbours')
            .get_parameter_value()
            .integer_value
        )

        self.random_seed = (
            self.get_parameter('random_seed')
            .get_parameter_value()
            .integer_value
        )

        self.robot_radius = (
            self.get_parameter('robot_radius')
            .get_parameter_value()
            .double_value
        )

        # Robot starts at the map origin used by the assignment.
        self.start = (
            0.0,
            0.0
        )

        # -----------------------------------------------------
        # QoS
        # -----------------------------------------------------

        map_qos = QoSProfile(depth=1)

        map_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        map_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        path_qos = QoSProfile(depth=1)

        path_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        path_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------

        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            path_qos
        )

        self.start_pub = self.create_publisher(
            PoseStamped,
            '/prm_start',
            path_qos
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/prm_goal',
            path_qos
        )

        self.roadmap_pub = self.create_publisher(
            MarkerArray,
            '/prm_roadmap',
            path_qos
        )

        # -----------------------------------------------------
        # Map
        # -----------------------------------------------------

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.planning_started = False

        self.get_logger().info(
            'PRM planner ready.'
        )

        self.get_logger().info(
            f'Student name: {self.student_name}'
        )

        self.get_logger().info(
            'Waiting for map...'
        )

    # =========================================================
    # Coordinate conversion used only for supplied goal
    # generation.
    # =========================================================

    def grid_to_world(
        self,
        gx,
        gy,
        resolution,
        origin
    ):

        wx = (
            origin[0]
            + (gx + 0.5) * resolution
        )

        wy = (
            origin[1]
            + (gy + 0.5) * resolution
        )

        return (
            wx,
            wy
        )

    # =========================================================
    # Deterministic student-specific goal
    # =========================================================

    def generate_goal(
        self,
        occupancy,
        resolution,
        origin
    ):

        height, width = occupancy.shape

        candidates = []

        for gy in range(height):

            for gx in range(width):

                if occupancy[gy, gx] != 0:
                    continue

                wx, wy = self.grid_to_world(
                    gx,
                    gy,
                    resolution,
                    origin
                )

                distance_from_start = math.hypot(
                    wx - self.start[0],
                    wy - self.start[1]
                )

                # Avoid trivial goals next to the robot.
                if distance_from_start < 2.0:
                    continue

                candidates.append(
                    (
                        wx,
                        wy
                    )
                )

        if not candidates:

            raise RuntimeError(
                'No suitable free-space goal candidates found.'
            )

        # Sort to guarantee deterministic ordering.
        candidates.sort()

        digest = hashlib.sha256(
            self.student_name.encode('utf-8')
        ).hexdigest()

        index = (
            int(
                digest[:16],
                16
            )
            % len(candidates)
        )

        return candidates[index]

    # =========================================================

    def publish_pose(
        self,
        publisher,
        position
    ):

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        msg.pose.position.x = float(
            position[0]
        )

        msg.pose.position.y = float(
            position[1]
        )

        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        publisher.publish(msg)

    # =========================================================

    def publish_path(
        self,
        path_points
    ):

        msg = Path()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        for x, y in path_points:

            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

        self.get_logger().info(
            f'Published path with '
            f'{len(msg.poses)} points.'
        )

    # =========================================================

    

    def make_marker(
        self,
        marker_id,
        namespace,
        marker_type,
        colour,
        width,
        points,
        height
    ):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = width
        marker.scale.y = width
        marker.scale.z = width

        (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a
        ) = colour

        marker.points = [
            Point(x=float(x), y=float(y), z=height)
            for x, y in points
        ]

        return marker

    def publish_roadmap(
        self,
        prm,
        path_points
    ):

        nodes = prm.nodes  # nodes[0] = start, nodes[1] = goal

        # Every accepted edge is a pair of consecutive points.
        edge_points = []

        for i, j in prm.edges:
            edge_points.append(nodes[i])
            edge_points.append(nodes[j])

        layers = [
            # (id, namespace, type, RGBA, width, points, height)
            (0, 'edges', Marker.LINE_LIST,
             (0.2, 0.4, 1.0, 0.4), 0.01, edge_points, 0.02),
            (1, 'samples', Marker.SPHERE_LIST,
             (0.0, 0.8, 0.8, 0.9), 0.05, nodes[2:], 0.03),
            (2, 'start', Marker.SPHERE_LIST,
             (0.0, 1.0, 0.0, 1.0), 0.25, [nodes[0]], 0.06),
            (3, 'goal', Marker.SPHERE_LIST,
             (1.0, 0.0, 0.0, 1.0), 0.25, [nodes[1]], 0.06),
            (4, 'path', Marker.LINE_STRIP,
             (1.0, 0.8, 0.0, 1.0), 0.06, path_points, 0.08),
        ]

        markers = MarkerArray()

        for layer in layers:

            # Skip empty layers (for example no path found).
            if layer[5]:
                markers.markers.append(self.make_marker(*layer))

        self.roadmap_pub.publish(markers)

        self.get_logger().info(
            f'Published roadmap markers on /prm_roadmap.'
        )


    def map_callback(
        self,
        msg
    ):

        # Only plan once.
        if self.planning_started:
            return

        self.planning_started = True

        self.get_logger().info(
            'Map received.'
        )

        width = msg.info.width
        height = msg.info.height

        resolution = msg.info.resolution

        origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y
        )

        self.get_logger().info(
            f'Map size: {width} x {height} cells, '
            f'resolution {resolution:.2f} m'
        )

        occupancy = np.array(
            msg.data,
            dtype=np.int16
        ).reshape(
            (
                height,
                width
            )
        )


        # -----------------------------------------------------
        # Generate assessed goal
        # -----------------------------------------------------

        try:

            goal = self.generate_goal(
                occupancy,
                resolution,
                origin
            )

        except RuntimeError as exc:

            self.get_logger().error(
                str(exc)
            )

            return

        self.get_logger().info(
            f'Start: '
            f'({self.start[0]:.2f}, '
            f'{self.start[1]:.2f})'
        )

        self.get_logger().info(
            f'Goal: '
            f'({goal[0]:.2f}, '
            f'{goal[1]:.2f})'
        )

        self.get_logger().info(
            f'PRM samples: {self.num_samples}'
        )

        self.get_logger().info(
            f'k-neighbours: {self.k_neighbours}'
        )

        self.get_logger().info(
            f'Random seed: {self.random_seed}'
        )

        self.get_logger().info(
            f'Robot radius: '
            f'{self.robot_radius:.2f} m'
        )

        self.publish_pose(
            self.start_pub,
            self.start
        )

        self.publish_pose(
            self.goal_pub,
            goal
        )

        # -----------------------------------------------------
        # Student PRM implementation
        # -----------------------------------------------------

        prm = PRMCore(
            occupancy=occupancy,
            resolution=resolution,
            origin=origin,
            robot_radius=self.robot_radius
        )

        self.get_logger().info(
            'Running PRM...'
        )

        try:

            path = prm.plan(
                start=self.start,
                goal=goal,
                num_samples=self.num_samples,
                k_neighbours=self.k_neighbours,
                seed=self.random_seed
            )

        except NotImplementedError as exc:

            self.get_logger().warning(
                str(exc)
            )

            self.get_logger().warning(
                'PRM has not yet been implemented.'
            )

            return

        except Exception as exc:

            self.get_logger().error(
                f'PRM failed with exception: {exc}'
            )

            return
            
        self.get_logger().info(
            f'Roadmap: {len(prm.nodes) - 2} samples, ' #print number of sample
            f'{len(prm.edges)} edges' # print number of edges
        )

        self.publish_roadmap(prm, path)
        
        if not path:

            self.get_logger().warning(
                'No path found.'
            )

            return

        self.get_logger().info(
            '===================================='
        )

        self.get_logger().info(
            f'PATH FOUND: {len(path)} points'
        )

        self.get_logger().info(
            '===================================='
        )

        self.publish_path(path)


def main(args=None):

    rclpy.init(args=args)

    node = PRMPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
