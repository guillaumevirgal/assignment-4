#!/usr/bin/env python3
"""Record /gps, /kalman/odom and /odom, then save the submission plot as a PDF.

Usage (while the simulation, sensors and your Kalman filter are running):

    python3 ~/ros2_ws/src/b31ys_kalman/scripts/plot_trajectories.py --duration 60

Stops after --duration seconds of SIMULATION time (0 = until Ctrl+C), then writes
  kalman_plot.pdf  page 1: X vs Y   page 2: x(t) and y(t)
  kalman_data.csv  the raw samples, in case you want to re-plot later
"""

import argparse
import csv

import matplotlib
matplotlib.use('Agg')  # no display needed
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, duration):
        super().__init__(
            'trajectory_recorder',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.duration = duration
        self.t0 = None
        self.done = False
        self.data = {'gps': [], 'kalman': [], 'reference': []}

        self.create_subscription(PoseStamped, '/gps', self.gps_cb, 10)
        self.create_subscription(Odometry, '/kalman/odom', self.kalman_cb, 10)
        self.create_subscription(Odometry, '/odom', self.ref_cb, 10)
        self.get_logger().info('Recording... (Ctrl+C to stop early)')

    def store(self, key, stamp, x, y):
        t = stamp_to_sec(stamp)
        if t == 0.0:  # some publishers leave the stamp empty: use node clock
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        t -= self.t0
        self.data[key].append((t, x, y))
        if self.duration > 0 and t >= self.duration:
            self.done = True

    def gps_cb(self, msg):
        self.store('gps', msg.header.stamp,
                   msg.pose.position.x, msg.pose.position.y)

    def kalman_cb(self, msg):
        p = msg.pose.pose.position
        self.store('kalman', msg.header.stamp, p.x, p.y)

    def ref_cb(self, msg):
        p = msg.pose.pose.position
        self.store('reference', msg.header.stamp, p.x, p.y)


STYLE = {
    'reference': dict(label='Reference odometry (/odom)', color='black',
                      linestyle='--', linewidth=1.5),
    'kalman': dict(label='EKF estimate (/kalman/odom)', color='tab:blue',
                   linewidth=2),
    'gps': dict(label='GPS measurements (/gps)', color='tab:red',
                marker='o', markersize=4, linestyle='none'),
}


def save_outputs(data, pdf_path, csv_path):
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['source', 't', 'x', 'y'])
        for key, rows in data.items():
            for t, x, y in rows:
                w.writerow([key, f'{t:.4f}', f'{x:.4f}', f'{y:.4f}'])

    with PdfPages(pdf_path) as pdf:
        # Page 1: X vs Y
        fig, ax = plt.subplots(figsize=(8, 7))
        for key in ('reference', 'kalman', 'gps'):
            rows = data[key]
            if rows:
                ax.plot([r[1] for r in rows], [r[2] for r in rows],
                        **STYLE[key])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Robot trajectory: GPS vs EKF vs reference')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # Page 2: position vs time
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        for i, name in ((1, 'X'), (2, 'Y')):
            ax = axes[i - 1]
            for key in ('reference', 'kalman', 'gps'):
                rows = data[key]
                if rows:
                    ax.plot([r[0] for r in rows], [r[i] for r in rows],
                            **STYLE[key])
            ax.set_ylabel(f'{name} position (m)')
            ax.grid(True, alpha=0.3)
        axes[0].set_title('Position vs time')
        axes[0].legend()
        axes[1].set_xlabel('Time (s)')
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0,
                        help='simulation seconds to record (0 = until Ctrl+C)')
    parser.add_argument('--pdf', default='kalman_plot.pdf')
    parser.add_argument('--csv', default='kalman_data.csv')
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args.duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    counts = {k: len(v) for k, v in node.data.items()}
    save_outputs(node.data, args.pdf, args.csv)
    print(f'Samples: {counts}')
    print(f'Saved {args.pdf} and {args.csv}')
    if counts['kalman'] == 0:
        print('WARNING: nothing received on /kalman/odom - is your filter running?')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
