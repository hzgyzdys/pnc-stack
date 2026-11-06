#!/usr/bin/env python3
# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""Closed-loop smoke test in Gazebo for any scenario.

Start the simulation first, then run this script in another terminal:

  ros2 launch pnc_bringup sim_tracking.launch.py scenario:=s4_u_turn headless:=true
  python3 tools/sim_smoke_test.py

It reads the reference path from /plan and passes when the car drives its
whole length, stops within 0.3 m of its end, and the cross-track error after
the settling window stays below --max-cte. A collision with a wall or cone
stops the car early, so it fails the distance check.
Exit code 0 = pass, 1 = fail.
"""

import argparse
import math
import sys

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64


class SmokeTest(Node):

    def __init__(self):
        super().__init__('sim_smoke_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cte = []          # (sim time [s], cte [m])
        self.distance = 0.0    # travelled distance [m]
        self.last_xy = None
        self.odom = None
        self.plan = None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'plan', m), latched)
        self.create_subscription(Float64, '/tracking_controller/cross_track_error',
                                 lambda m: self.cte.append((self.now_s(), m.data)), 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.distance += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)
        self.odom = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=60.0, help='sim seconds')
    parser.add_argument('--max-cte', type=float, default=0.05, help='[m]')
    parser.add_argument('--settle', type=float, default=5.0,
                        help='sim seconds ignored before CTE is scored')
    args = parser.parse_args()

    rclpy.init()
    node = SmokeTest()
    stopped_since = None
    path_length = float('inf')
    goal = (float('nan'), float('nan'))
    while rclpy.ok() and node.now_s() < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.plan is not None and node.plan.poses:
            pts = [(p.pose.position.x, p.pose.position.y) for p in node.plan.poses]
            path_length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
            goal = pts[-1]
        lap_done = node.distance > 0.9 * path_length
        stopped = node.odom is not None and abs(node.odom.twist.twist.linear.x) < 0.02
        if lap_done and stopped:
            stopped_since = stopped_since or node.now_s()
            if node.now_s() - stopped_since > 1.0:
                break
        else:
            stopped_since = None

    settled = [abs(v) for t, v in node.cte if t > args.settle]
    max_cte = max(settled) if settled else float('nan')
    rms_cte = math.sqrt(sum(v * v for v in settled) / len(settled)) if settled else float('nan')
    end = node.last_xy or (float('nan'), float('nan'))
    end_dist = math.dist(end, goal)
    print(f'sim time {node.now_s():.1f} s, distance {node.distance:.2f} / {path_length:.2f} m, '
          f'end ({end[0]:.2f}, {end[1]:.2f}), '
          f'CTE rms {rms_cte:.4f} m, max {max_cte:.4f} m ({len(settled)} samples)')

    failures = []
    if node.plan is None:
        failures.append('no reference path on /plan')
    elif node.distance < 0.9 * path_length:
        failures.append(f'did not reach the end of the path ({node.distance:.2f} m)')
    if stopped_since is None:
        failures.append('did not stop at the goal')
    if not settled:
        failures.append('no cross-track error samples')
    elif max_cte > args.max_cte:
        failures.append(f'max CTE {max_cte:.4f} m > {args.max_cte} m')
    if end_dist > 0.3:
        failures.append(f'stopped {end_dist:.2f} m from the end of the path')

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print('FAIL: ' + '; '.join(failures))
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
