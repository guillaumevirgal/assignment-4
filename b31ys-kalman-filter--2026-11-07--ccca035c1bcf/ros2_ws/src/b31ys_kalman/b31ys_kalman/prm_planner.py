#!/usr/bin/env python3
"""PRM planner node, adapted from CW1 to plan on the SLAM map.

Subscribes
    /map        nav_msgs/OccupancyGrid  saved SLAM map (from map_server)
    /goal_pose  geometry_msgs/PoseStamped  goal, e.g. RViz "2D Goal Pose"
Publishes
    /planned_path   nav_msgs/Path          path for the path follower
    /prm_start, /prm_goal  PoseStamped     start/goal actually used
    /prm_roadmap    MarkerArray            roadmap, samples, path (RViz)
    /prm_free_space OccupancyGrid          where the robot centre may go
                                           (shows the safety distance)

CW2 changes compared with CW1:
* start = robot's current pose in the map (TF map -> base_link), instead
  of a fixed (0, 0); falls back to (start_x, start_y) when TF is missing.
* goals come from /goal_pose (one plan per goal, so several rooms can be
  visited one after the other) or from goal_x / goal_y at start-up,
  instead of a goal generated from the student name.
* the roadmap is built once per map and reused for every goal; if a goal
  cannot be reached, the roadmap is rebuilt with more samples.
* planning statistics are logged, and appended to a CSV if results_csv
  is set (for the report: success rate, path length, time, nodes).
"""
import csv
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

from b31ys_kalman.prm_core import PRMCore


class PRMPlanner(Node):

    def __init__(self):
        super().__init__('prm_planner')

        # -----------------------------------------------------
        # Parameters
        # -----------------------------------------------------
        self.declare_parameter('num_samples', 800)
        self.declare_parameter('k_neighbours', 12)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('shortcut_path', True)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('results_csv', '')

        p = self.get_parameter
        self.num_samples = p('num_samples').value
        self.k_neighbours = p('k_neighbours').value
        self.random_seed = p('random_seed').value
        self.robot_radius = p('robot_radius').value
        self.safety_margin = p('safety_margin').value
        self.shortcut_path = p('shortcut_path').value
        self.max_retries = p('max_retries').value
        self.map_frame = p('map_frame').value
        self.robot_frame = p('robot_frame').value
        self.fallback_start = (p('start_x').value, p('start_y').value)
        self.results_csv = p('results_csv').value

        goal_x, goal_y = p('goal_x').value, p('goal_y').value
        self.pending_goal = (
            None if math.isnan(goal_x) or math.isnan(goal_y)
            else (goal_x, goal_y)
        )

        # -----------------------------------------------------
        # QoS
        # -----------------------------------------------------
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------
        self.path_pub = self.create_publisher(Path, '/planned_path', latched)
        self.start_pub = self.create_publisher(PoseStamped, '/prm_start', latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/prm_goal', latched)
        self.roadmap_pub = self.create_publisher(MarkerArray, '/prm_roadmap', latched)
        self.free_space_pub = self.create_publisher(
            OccupancyGrid, '/prm_free_space', latched)

        # -----------------------------------------------------
        # Subscriptions and TF
        # -----------------------------------------------------
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.tf_buffer = Buffer()
        # Own thread, so lookups inside callbacks don't block TF updates
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.prm = None
        self.map_msg = None
        self.query_count = 0

        self.get_logger().info(
            'PRM planner ready. Waiting for /map, then goals on /goal_pose '
            '(RViz "2D Goal Pose").'
        )

    # =========================================================
    # Map: build the configuration space and the roadmap once
    # =========================================================
    def map_callback(self, msg):
        self.map_msg = msg
        width, height = msg.info.width, msg.info.height
        resolution = msg.info.resolution
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        occupancy = np.array(msg.data, dtype=np.int16).reshape((height, width))

        self.get_logger().info(
            f'Map received: {width} x {height} cells, '
            f'resolution {resolution:.2f} m'
        )

        self.prm = PRMCore(
            occupancy=occupancy,
            resolution=resolution,
            origin=origin,
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
        )
        self.publish_free_space()
        self.build_roadmap(self.num_samples, self.random_seed)

        if self.pending_goal is not None:
            goal, self.pending_goal = self.pending_goal, None
            # Give TF a moment so the start is the real robot pose
            self.initial_goal_timer = self.create_timer(
                2.0, lambda: self.run_initial_goal(goal))

    def run_initial_goal(self, goal):
        self.initial_goal_timer.cancel()
        self.plan_to(goal)

    def build_roadmap(self, num_samples, seed):
        edges = self.prm.build_roadmap(num_samples, self.k_neighbours, seed)
        s = self.prm.stats
        self.get_logger().info(
            f'Roadmap built: {s["roadmap_nodes"]} nodes, {edges} edges, '
            f'{s["build_time"]:.2f} s '
            f'(robot radius {self.robot_radius:.2f} m + '
            f'safety margin {self.safety_margin:.2f} m, k={self.k_neighbours})'
        )

    # =========================================================
    # Goals
    # =========================================================
    def goal_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Goal frame is "{msg.header.frame_id}", expected '
                f'"{self.map_frame}". Using it as map coordinates.'
            )
        self.plan_to((msg.pose.position.x, msg.pose.position.y))

    def robot_position(self):
        """Robot position in the map frame, or the fallback start."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time(),
                timeout=Duration(seconds=1.0))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as exc:
            self.get_logger().warning(
                f'No {self.map_frame} -> {self.robot_frame} transform ({exc}). '
                f'Using start ({self.fallback_start[0]:.2f}, '
                f'{self.fallback_start[1]:.2f}).'
            )
            return self.fallback_start

    def plan_to(self, goal):
        if self.prm is None:
            self.get_logger().warning('No map yet, goal ignored.')
            return
        self.query_count += 1
        start = self.robot_position()
        self.get_logger().info(
            f'[goal {self.query_count}] start ({start[0]:.2f}, {start[1]:.2f}) '
            f'-> goal ({goal[0]:.2f}, {goal[1]:.2f})'
        )

        t0 = time.perf_counter()
        build_time = 0.0
        path = self.prm.query(start, goal, self.k_neighbours, self.shortcut_path)

        # Not connected: denser roadmap, new seed (only if start/goal are valid)
        retries = 0
        while (not path and retries < self.max_retries
               and self.prm.start is not None and self.prm.goal is not None):
            retries += 1
            samples = self.num_samples * (2 ** retries)
            self.get_logger().info(
                f'No path, rebuilding the roadmap with {samples} samples...')
            self.build_roadmap(samples, self.random_seed + retries)
            build_time += self.prm.stats['build_time']
            path = self.prm.query(start, goal, self.k_neighbours, self.shortcut_path)

        planning_time = time.perf_counter() - t0
        s = self.prm.stats

        if self.prm.start is not None:
            self.publish_pose(self.start_pub, self.prm.start)
        if self.prm.goal is not None:
            self.publish_pose(self.goal_pub, self.prm.goal)
        self.publish_roadmap(path)

        if s.get('start_snapped'):
            self.get_logger().info('Start was too close to an obstacle: moved to the nearest safe cell.')
        if s.get('goal_snapped'):
            self.get_logger().info('Goal was too close to an obstacle: moved to the nearest safe cell.')

        if not path:
            self.get_logger().warning(f'No path found: {s.get("failure", "")}.')
            self.publish_path([])  # clears any previous path
        else:
            self.get_logger().info(
                f'PATH FOUND: {s["path_length"]:.2f} m, {s["waypoints"]} waypoints '
                f'(roadmap path {s["raw_length"]:.2f} m, {s["raw_waypoints"]} waypoints), '
                f'planning time {planning_time:.3f} s, '
                f'roadmap {s["roadmap_nodes"]} nodes / {s["roadmap_edges"]} edges'
            )
            self.publish_path(path)

        self.save_result(start, goal, path, planning_time, build_time, retries)

    def save_result(self, start, goal, path, planning_time, build_time, retries):
        if not self.results_csv:
            return
        s = self.prm.stats
        new_file = not os.path.exists(self.results_csv)
        try:
            with open(self.results_csv, 'a', newline='') as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow([
                        'query', 'start_x', 'start_y', 'goal_x', 'goal_y',
                        'success', 'path_length_m', 'roadmap_path_length_m',
                        'waypoints', 'planning_time_s', 'rebuild_time_s',
                        'retries', 'roadmap_nodes', 'roadmap_edges',
                        'samples_setting', 'k', 'robot_radius', 'safety_margin',
                        'failure'])
                w.writerow([
                    self.query_count, f'{start[0]:.3f}', f'{start[1]:.3f}',
                    f'{goal[0]:.3f}', f'{goal[1]:.3f}', int(bool(path)),
                    f'{s.get("path_length", 0):.3f}', f'{s.get("raw_length", 0):.3f}',
                    s.get('waypoints', 0), f'{planning_time:.4f}', f'{build_time:.4f}',
                    retries, s['roadmap_nodes'], s['roadmap_edges'],
                    self.num_samples, self.k_neighbours, self.robot_radius,
                    self.safety_margin, s.get('failure', '')])
        except OSError as exc:
            self.get_logger().warning(f'Could not write {self.results_csv}: {exc}')

    # =========================================================
    # Publishing
    # =========================================================
    def publish_pose(self, publisher, position):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.orientation.w = 1.0
        publisher.publish(msg)

    def publish_path(self, path_points):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_free_space(self):
        """Grid of where the robot centre may go: 0 = allowed,
        100 = too close to an obstacle or unknown."""
        msg = OccupancyGrid()
        msg.header = self.map_msg.header
        msg.header.frame_id = self.map_frame
        msg.info = self.map_msg.info
        msg.data = np.where(self.prm.free, 0, 100).astype(np.int8).ravel().tolist()
        self.free_space_pub.publish(msg)

    def make_marker(self, marker_id, namespace, marker_type, colour, width,
                    points, height):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.scale.y = width
        marker.scale.z = width
        (marker.color.r, marker.color.g,
         marker.color.b, marker.color.a) = colour
        marker.points = [Point(x=float(x), y=float(y), z=height) for x, y in points]
        return marker

    def publish_roadmap(self, path_points):
        prm = self.prm
        nodes = prm.nodes

        edge_points = []  # Every accepted edge is a pair of consecutive points
        for i, j in prm.edges:
            edge_points.append(nodes[i])
            edge_points.append(nodes[j])

        layers = [
            # (id, namespace, type, RGBA, width, points, height)
            (0, 'edges', Marker.LINE_LIST,
             (0.2, 0.4, 1.0, 0.4), 0.01, edge_points, 0.02),
            (1, 'samples', Marker.SPHERE_LIST,
             (0.0, 0.8, 0.8, 0.9), 0.05, prm.roadmap_nodes, 0.03),
            (2, 'start', Marker.SPHERE_LIST,
             (0.0, 1.0, 0.0, 1.0), 0.25, [prm.start] if prm.start else [], 0.06),
            (3, 'goal', Marker.SPHERE_LIST,
             (1.0, 0.0, 0.0, 1.0), 0.25, [prm.goal] if prm.goal else [], 0.06),
            (4, 'roadmap_path', Marker.LINE_STRIP,
             (1.0, 0.5, 0.0, 0.6), 0.03, prm.raw_path, 0.07),
            (5, 'path', Marker.LINE_STRIP,
             (1.0, 0.8, 0.0, 1.0), 0.06, path_points, 0.08),
        ]

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL  # remove the previous query's markers
        markers.markers.append(clear)
        for layer in layers:
            if len(layer[5]) >= (2 if layer[2] in (Marker.LINE_LIST, Marker.LINE_STRIP) else 1):
                markers.markers.append(self.make_marker(*layer))
        self.roadmap_pub.publish(markers)


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
