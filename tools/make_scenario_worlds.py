#!/usr/bin/env python3
# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""Generate the Gazebo worlds for the test scenarios.

Each scenario's reference path is defined once, in
src/pnc_bringup/config/scenarios/<name>.yaml. This script reads that path and
places the painted centre line, walls and obstacles relative to it, so the
world and the path cannot drift apart. Re-run it after editing a scenario:

  python3 tools/make_scenario_worlds.py

Output: src/pnc_sim/worlds/<name>.sdf (committed, so users need not run this).
"""

import math
import pathlib

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCENARIOS = ROOT / 'src' / 'pnc_bringup' / 'config' / 'scenarios'
WORLDS = ROOT / 'src' / 'pnc_sim' / 'worlds'

# Car footprint, from pnc_sim/models/ackermann_car/model.sdf.
CAR_HALF_WIDTH = 0.145   # outer face of the wheels [m]


# --------------------------------------------------------------------------
# Path geometry (mirrors pnc_core::parseSegment / makeSegmentPath)

def parse_segment(text):
    parts = text.split()
    if parts[0] == 'line':
        return float(parts[1]), 0.0
    radius, angle = float(parts[1]), math.radians(float(parts[2]))
    return radius * abs(angle), math.copysign(1.0 / radius, angle)


def pose_at(x, y, yaw, k, s):
    if abs(k) < 1e-12:
        return x + s * math.cos(yaw), y + s * math.sin(yaw), yaw
    yaw2 = yaw + k * s
    return x + (math.sin(yaw2) - math.sin(yaw)) / k, y - (math.cos(yaw2) - math.cos(yaw)) / k, yaw2


class Path:
    """Chain of (start pose, length, curvature) pieces with arc-length lookup."""

    def __init__(self, params):
        x, y, yaw = params.get('start_x', 0.0), params.get('start_y', 0.0), params.get('start_yaw', 0.0)
        self.pieces = []
        for text in params['segments']:
            length, k = parse_segment(text)
            self.pieces.append((x, y, yaw, length, k))
            x, y, yaw = pose_at(x, y, yaw, k, length)
        self.length = sum(p[3] for p in self.pieces)

    def pose(self, s):
        for x, y, yaw, length, k in self.pieces:
            if s <= length:
                return pose_at(x, y, yaw, k, s)
            s -= length
        x, y, yaw, length, k = self.pieces[-1]
        return pose_at(x, y, yaw, k, length)

    def offset(self, s, d):
        """Point at arc length s, shifted d to the left of the path."""
        x, y, yaw = self.pose(s)
        return x - d * math.sin(yaw), y + d * math.cos(yaw), yaw


# --------------------------------------------------------------------------
# SDF helpers

def box(name, x, y, z, yaw, sx, sy, sz, rgb, collide=True):
    color = f'{rgb[0]} {rgb[1]} {rgb[2]} 1'
    geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    pose = f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>'
    out = (f'        <visual name="{name}_v">{pose}{geom}'
           f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n')
    if collide:
        out += f'        <collision name="{name}_c">{pose}{geom}</collision>\n'
    return out


def cylinder(name, x, y, r, h, rgb):
    color = f'{rgb[0]} {rgb[1]} {rgb[2]} 1'
    geom = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
    pose = f'<pose>{x:.4f} {y:.4f} {h / 2:.4f} 0 0 0</pose>'
    return (f'        <visual name="{name}_v">{pose}{geom}'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n'
            f'        <collision name="{name}_c">{pose}{geom}</collision>\n')


def static_model(name, body):
    return (f'    <model name="{name}">\n      <static>true</static>\n      <link name="link">\n'
            f'{body}      </link>\n    </model>\n')


def painted_line(path, step=0.1):
    """Thin white strip along the reference path, visual only."""
    body = ''
    n = max(1, math.ceil(path.length / step))
    for i in range(n):
        s0, s1 = path.length * i / n, path.length * (i + 1) / n
        x0, y0, _ = path.pose(s0)
        x1, y1, _ = path.pose(s1)
        seg = math.hypot(x1 - x0, y1 - y0)
        body += box(f'p{i}', (x0 + x1) / 2, (y0 + y1) / 2, 0.001, math.atan2(y1 - y0, x1 - x0),
                    seg + 0.005, 0.03, 0.002, (1, 1, 1), collide=False)
    return static_model('reference_line', body)


def walls_along(path, s_from, s_to, half_gap, thickness=0.1, height=0.3, step=0.2):
    """Wall pieces on both sides, inner faces `half_gap` from the centre line."""
    body = ''
    n = max(1, math.ceil((s_to - s_from) / step))
    d = half_gap + thickness / 2
    for side, sign in (('l', 1.0), ('r', -1.0)):
        for i in range(n):
            s = s_from + (s_to - s_from) * (i + 0.5) / n
            x, y, yaw = path.offset(s, sign * d)
            # Pieces on the outside of a bend need to be a little longer to close the gaps.
            body += box(f'{side}{i}', x, y, height / 2, yaw, (s_to - s_from) / n * 1.3 + 0.02,
                        thickness, height, (0.75, 0.35, 0.2))
    return static_model('walls', body)


# --------------------------------------------------------------------------
# Scenarios: what to put around each path

def s1_straight_offset(path):
    return painted_line(path)


def s2_circle(path):
    return painted_line(path)


def s3_narrow_passage(path):
    # 0.3 m clearance each side of the car, from 2 m in to the end.
    return painted_line(path) + walls_along(path, 2.0, path.length, CAR_HALF_WIDTH + 0.3)


def s4_u_turn(path):
    return painted_line(path) + walls_along(path, 1.0, path.length, 0.7)


def s5_static_obstacles(path):
    # A cone 0.6 m inside the apex of every 60 deg bend.
    body = ''
    s = 0.0
    for i, (_, _, _, length, k) in enumerate(path.pieces):
        if abs(k) > 0 and abs(length * k) > math.radians(45):
            x, y, _ = path.offset(s + length / 2, math.copysign(0.6, k))
            body += cylinder(f'cone{i}', x, y, 0.15, 0.4, (1.0, 0.45, 0.0))
        s += length
    return painted_line(path) + static_model('cones', body)


def s7_parallel_parking(path):
    # Kerb 1.0 m right of the path, two parked cars, a 0.9 m slot between them.
    parked = (0.15, 0.15, 0.15)
    body = box('kerb', 4.0, -1.05, 0.05, 0.0, 6.0, 0.1, 0.1, (0.8, 0.8, 0.8))
    body += box('car_a', 2.9, -0.7, 0.08, 0.0, 0.45, 0.25, 0.16, parked)
    body += box('car_b', 4.25, -0.7, 0.08, 0.0, 0.45, 0.25, 0.16, parked)
    # Slot outline (paint, visual only).
    for i, x in enumerate((3.125, 4.025)):
        body += box(f'slot_end{i}', x, -0.775, 0.001, 0.0, 0.03, 0.45, 0.002, (1, 1, 0), False)
    body += box('slot_side', 3.575, -0.55, 0.001, 0.0, 0.93, 0.03, 0.002, (1, 1, 0), False)
    return painted_line(path) + static_model('parking', body)


# --------------------------------------------------------------------------

HEADER = """<?xml version="1.0"?>
<!--
  GENERATED by tools/make_scenario_worlds.py from
  src/pnc_bringup/config/scenarios/{name}.yaml. Edit those, then re-run the script.

  Light on purpose: no sensors, no shadows, static obstacles only, so it runs on
  an integrated GPU and headless in CI.
-->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.9 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{cam_x:.2f} {cam_y:.2f} {cam_z:.2f} 0 1.1 0</pose>
      </camera>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

"""

FOOTER = """
    <!-- Rear-axle centre at the origin heading +x, axle height = wheel radius. -->
    <include>
      <uri>model://ackermann_car</uri>
      <name>ackermann_car</name>
      <pose>0 0 0.052 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def main():
    WORLDS.mkdir(parents=True, exist_ok=True)
    for cfg in sorted(SCENARIOS.glob('*.yaml')):
        name = cfg.stem
        params = yaml.safe_load(cfg.read_text())['reference_path']['ros__parameters']
        path = Path(params)
        # Camera above and behind the middle of the path's bounding box.
        pts = [path.pose(path.length * i / 50) for i in range(51)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 4.0)
        cam_x = (min(xs) + max(xs)) / 2 - 0.45 * span
        cam_y = (min(ys) + max(ys)) / 2
        body = globals()[name](path)
        sdf = (HEADER.format(name=name, cam_x=cam_x, cam_y=cam_y, cam_z=0.9 * span)
               + body + FOOTER)
        out = WORLDS / f'{name}.sdf'
        out.write_text(sdf)
        print(f'{out.relative_to(ROOT)}: path {path.length:.2f} m')


if __name__ == '__main__':
    main()
