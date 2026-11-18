#!/usr/bin/env python3
"""Goal sequencer: visit the patient rooms one after the other (Part 4).

Sends each room as a goal on /goal_pose (the PRM planner plans from the
robot's localised pose), waits until the path follower reports
"reached" on /path_follower/status, waits wait_at_room seconds, then sends
the next room. Rooms are read from config/rooms.yaml.

Also publishes the rooms as labelled markers on /rooms for RViz.

Status per goal and a summary (reached / failed, time) are logged.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException


class GoalSequencer(Node):

    def __init__(self):
        super().__init__('goal_sequencer')

        self.declare_parameter('sequence', ['room_1', 'room_2', 'room_3'])
        self.declare_parameter('place_names', ['room_1', 'room_2', 'room_3'])
        self.declare_parameter('place_x', [0.0, 0.0, 0.0])
        self.declare_parameter('place_y', [0.0, 0.0, 0.0])
        self.declare_parameter('wait_at_room', 3.0)
        self.declare_parameter('goal_timeout', 300.0)
        self.declare_parameter('start_delay', 5.0)

        p = self.get_parameter
        names = list(p('place_names').value)
        xs, ys = list(p('place_x').value), list(p('place_y').value)
        if not (len(names) == len(xs) == len(ys)):
            raise ValueError('place_names, place_x and place_y must have the same length')
        self.places = {n: (x, y) for n, x, y in zip(names, xs, ys)}
        self.sequence = [n for n in p('sequence').value if n in self.places]
        for n in p('sequence').value:
            if n not in self.places:
                self.get_logger().error(f'Unknown place "{n}" in sequence, skipped.')
        self.wait_at_room = p('wait_at_room').value
        self.goal_timeout = p('goal_timeout').value
        self.start_delay = p('start_delay').value

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rooms', latched)
        self.create_subscription(String, '/path_follower/status', self.status_cb, latched)
        self.create_subscription(OccupancyGrid, '/prm_free_space', self.planner_ready_cb, latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.planner_ready = False
        self.follower_status = ''
        self.index = -1          # current goal in self.sequence
        self.state = 'waiting'   # waiting, starting, driving, at_room, done
        self.state_since = self.now()
        self.seen_following = False
        self.results = []

        self.create_timer(0.5, self.step)
        self.create_timer(2.0, self.publish_markers)
        self.get_logger().info(
            f'Visit order: {" -> ".join(self.sequence)}. Waiting for the planner and localisation...')

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def planner_ready_cb(self, _msg):
        self.planner_ready = True

    def status_cb(self, msg):
        self.follower_status = msg.data
        if msg.data in ('following', 'blocked'):
            self.seen_following = True

    def localised(self):
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return True
        except TransformException:
            return False

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    def set_state(self, state):
        self.state = state
        self.state_since = self.now()

    # ------------------------------------------------------------------
    def step(self):
        elapsed = self.now() - self.state_since

        if self.state == 'waiting':
            if self.planner_ready and self.localised():
                self.get_logger().info(
                    f'System ready. First goal in {self.start_delay:.0f} s.')
                self.set_state('starting')

        elif self.state == 'starting':
            if elapsed >= self.start_delay:
                self.next_goal()

        elif self.state == 'driving':
            name = self.sequence[self.index]
            if self.seen_following and self.follower_status == 'reached':
                self.finish_goal(name, True, elapsed)
            elif elapsed > 3.0 and not self.seen_following and self.follower_status == 'idle':
                # Planner published an empty path: no route to this room
                self.finish_goal(name, False, elapsed, 'no path')
            elif elapsed > self.goal_timeout:
                self.finish_goal(name, False, elapsed, 'timeout')

        elif self.state == 'at_room':
            if elapsed >= self.wait_at_room:
                self.next_goal()

    def next_goal(self):
        self.index += 1
        if self.index >= len(self.sequence):
            self.set_state('done')
            self.summary()
            return
        name = self.sequence[self.index]
        x, y = self.places[name]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.seen_following = False
        self.follower_status = ''
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'[{self.index + 1}/{len(self.sequence)}] Going to {name} ({x:.1f}, {y:.1f})')
        self.set_state('driving')

    def finish_goal(self, name, reached, elapsed, reason=''):
        pos = self.robot_xy()
        error = math.dist(pos, self.places[name]) if pos else float('nan')
        self.results.append((name, reached, elapsed, error, reason))
        if reached:
            self.get_logger().info(
                f'Reached {name} in {elapsed:.0f} s (sim time), '
                f'{error:.2f} m from the target. Delivering medicine...')
            self.set_state('at_room')
        else:
            self.get_logger().warning(f'Could not reach {name}: {reason}. Next goal.')
            self.next_goal()

    def summary(self):
        ok = sum(1 for r in self.results if r[1])
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Tour finished: {ok}/{len(self.results)} places reached')
        for name, reached, t, err, reason in self.results:
            self.get_logger().info(
                f'  {name:<14} {"reached" if reached else "FAILED (" + reason + ")":<18} '
                f'{t:6.0f} s   final error {err:.2f} m')
        self.get_logger().info('=' * 50)

    # ------------------------------------------------------------------
    def publish_markers(self):
        markers = MarkerArray()
        current = (self.sequence[self.index]
                   if 0 <= self.index < len(self.sequence) else None)
        for i, (name, (x, y)) in enumerate(self.places.items()):
            colour = (1.0, 0.6, 0.0) if name == current else (0.2, 0.8, 0.2)
            for kind, mid in ((Marker.CYLINDER, 2 * i), (Marker.TEXT_VIEW_FACING, 2 * i + 1)):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'rooms'
                m.id = mid
                m.type = kind
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.orientation.w = 1.0
                m.color.r, m.color.g, m.color.b = colour
                m.color.a = 0.5 if kind == Marker.CYLINDER else 1.0
                if kind == Marker.CYLINDER:
                    m.scale.x = m.scale.y = 0.5
                    m.scale.z = 0.05
                else:
                    m.pose.position.z = 0.6
                    m.scale.z = 0.45
                    m.text = name.replace('_', ' ')
                markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GoalSequencer()
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
