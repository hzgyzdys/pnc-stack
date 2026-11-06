#!/usr/bin/env python3
# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""Closed-loop smoke test in Gazebo: one lap of the default circle.

Start the simulation first, then run this script in another terminal:

  ros2 launch pnc_bringup sim_tracking.launch.py headless:=true
  python3 tools/sim_smoke_test.py

It passes when the car drives one full lap, stops near the start, and the
cross-track error after a 5 s settling window stays below --max-cte.
Exit code 0 = pass, 1 = fail.
"""

import argparse
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class SmokeTest(Node):

    def __init__(self):
        super().__init__('sim_smoke_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cte = []          # (sim time [s], cte [m])
        self.distance = 0.0    # travelled distance [m]
        self.last_xy = None
        self.odom = None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
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
    parser.add_argument('--lap-length', type=float, default=2 * math.pi * 3.0, help='[m]')
    args = parser.parse_args()

    rclpy.init()
    node = SmokeTest()
    stopped_since = None
    while rclpy.ok() and node.now_s() < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        lap_done = node.distance > 0.9 * args.lap_length
        stopped = node.odom is not None and abs(node.odom.twist.twist.linear.x) < 0.02
        if lap_done and stopped:
            stopped_since = stopped_since or node.now_s()
            if node.now_s() - stopped_since > 1.0:
                break
        else:
            stopped_since = None

    settled = [abs(v) for t, v in node.cte if t > 5.0]
    max_cte = max(settled) if settled else float('nan')
    rms_cte = math.sqrt(sum(v * v for v in settled) / len(settled)) if settled else float('nan')
    end = node.last_xy or (float('nan'), float('nan'))
    end_dist = math.hypot(end[0], end[1])
    print(f'sim time {node.now_s():.1f} s, distance {node.distance:.2f} m, '
          f'end ({end[0]:.2f}, {end[1]:.2f}), '
          f'CTE rms {rms_cte:.4f} m, max {max_cte:.4f} m ({len(settled)} samples)')

    failures = []
    if node.distance < 0.9 * args.lap_length:
        failures.append(f'did not complete a lap ({node.distance:.2f} m)')
    if stopped_since is None:
        failures.append('did not stop at the goal')
    if not settled:
        failures.append('no cross-track error samples')
    elif max_cte > args.max_cte:
        failures.append(f'max CTE {max_cte:.4f} m > {args.max_cte} m')
    if end_dist > 0.3:
        failures.append(f'stopped {end_dist:.2f} m from the start')

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print('FAIL: ' + '; '.join(failures))
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
