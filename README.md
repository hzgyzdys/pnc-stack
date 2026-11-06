# pnc-stack

Planning and control stack for an Ackermann-steered vehicle on **ROS 2 Jazzy**,
with self-written planners and controllers benchmarked against Nav2.

> Status: Phase 1 in progress — Pure Pursuit tracks a reference path in Gazebo
> Harmonic on a 1:10 Ackermann car in six test scenarios, each checked headless in CI. See the
> [design document](docs/design.md) for architecture, interfaces, evaluation plan
> and roadmap.

## Layout

```
src/
  pnc_core/      ROS-independent algorithms (C++17, plain CMake) + GoogleTest
  pnc_control/   ROS 2 nodes: tracking_controller, reference_path
  pnc_sim/       Gazebo model (1:10 Ackermann car), world, ros_gz bridge
  pnc_bringup/   Launch files and parameters
tools/           Smoke tests and helper scripts
docs/            Design document
docker/          Reproducible build environment
```

All algorithms live in `pnc_core` and have no ROS dependency, so they are
unit-tested in isolation and reused by the ROS nodes and the offline tools.

## Build and test

### Algorithms only (no ROS needed)

```bash
sudo apt install cmake g++ libgtest-dev
cmake -S src/pnc_core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/track_sim circle circle.csv     # closed-loop run, prints metrics
```

### Full workspace (ROS 2 Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
source install/setup.bash
ros2 launch pnc_bringup control.launch.py
```

Without a simulator the controller waits for `/odom` and publishes zero
commands.

### Gazebo simulation

```bash
source install/setup.bash
ros2 launch pnc_bringup sim_tracking.launch.py                         # S2 circle, with GUI
ros2 launch pnc_bringup sim_tracking.launch.py scenario:=s4_u_turn     # pick a scenario
ros2 launch pnc_bringup sim_tracking.launch.py headless:=true          # server only
```

This starts Gazebo Harmonic with the car at the origin, the ros_gz bridge
(`/odom`, `/cmd_vel`, `/clock`, `/tf`), the reference path and the tracking
controller. The path is painted white on the ground; the car drives it and
stops at the end.

| Scenario | What it is |
|---|---|
| `s1_straight_offset` | 12 m straight line, car starts 0.5 m off it |
| `s2_circle` | One lap of a circle, R = 3 m (default) |
| `s3_narrow_passage` | S-bend into a corridor, 0.3 m clearance each side |
| `s4_u_turn` | U-turn, R = 1.2 m, between walls |
| `s5_static_obstacles` | Slalom around four cones (fixed path until the planner exists) |
| `s7_parallel_parking` | Parking slot between two parked cars; drives up to the slot (reverse parking comes with the planner) |

Each scenario is `pnc_bringup/config/scenarios/<name>.yaml` (the path, as lines
and arcs) plus `pnc_sim/worlds/<name>.sdf`, generated from the path by
`tools/make_scenario_worlds.py`. To check a run automatically, start it with
`headless:=true` and run in a second terminal:

```bash
python3 tools/sim_smoke_test.py
```

### Docker

```bash
docker build -f docker/Dockerfile -t pnc-stack .
docker run --rm -it pnc-stack
```

## Results

| Scenario | Controller | RMS CTE | Max CTE | Environment |
|---|---|---|---|---|
| Straight, 0.5 m offset | Pure Pursuit | 0.0014 m | 0.0108 m | Offline kinematic model |
| Circle, R = 3 m | Pure Pursuit | 0.0064 m | 0.0183 m | Offline kinematic model |
| S1 straight, 0.5 m offset, 1 m/s | Pure Pursuit | 0.0035 m | 0.0150 m | Gazebo Harmonic |
| S2 circle, R = 3 m, 1 m/s | Pure Pursuit | 0.0057 m | 0.0282 m | Gazebo Harmonic |
| S3 narrow passage, 1 m/s | Pure Pursuit | 0.028 m | 0.075 m | Gazebo Harmonic |
| S4 U-turn, R = 1.2 m, 1 m/s | Pure Pursuit | 0.079 m | 0.134 m | Gazebo Harmonic |
| S5 cone slalom, 1 m/s | Pure Pursuit | 0.074 m | 0.117 m | Gazebo Harmonic |

Gazebo numbers exclude the first 5 s. Pure Pursuit cuts corners where the
curvature jumps (S3–S5); that is the gap Stanley and MPC are meant to close.

Comparisons with Nav2 controllers will be added as each phase completes.

## Roadmap

- [x] Phase 0 — skeleton, Pure Pursuit, tests, CI
- [ ] Phase 1 — Gazebo Ackermann vehicle ✔, Nav2 baseline, Stanley, MPC
- [ ] Phase 2 — Hybrid A*, QP trajectory optimisation
- [ ] Phase 3 — moving obstacles, behaviour FSM, EKF, full evaluation, demo video

## License

MIT
