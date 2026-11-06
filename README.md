# pnc-stack

Planning and control stack for an Ackermann-steered vehicle on **ROS 2 Jazzy**,
with self-written planners and controllers benchmarked against Nav2.

> Status: Phase 0 — repository skeleton, Pure Pursuit controller, unit tests and
> CI are in place. Gazebo simulation arrives in Phase 1. See the
> [design document](docs/design.md) for architecture, interfaces, evaluation plan
> and roadmap.

## Layout

```
src/
  pnc_core/      ROS-independent algorithms (C++17, plain CMake) + GoogleTest
  pnc_control/   ROS 2 nodes: tracking_controller, reference_path
  pnc_bringup/   Launch files and parameters
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
commands. The Gazebo world and vehicle are added in Phase 1.

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

Gazebo results and comparisons with Nav2 controllers will be added as each phase
completes.

## Roadmap

- [x] Phase 0 — skeleton, Pure Pursuit, tests, CI
- [ ] Phase 1 — Gazebo Ackermann vehicle, Nav2 baseline, Stanley, MPC
- [ ] Phase 2 — Hybrid A*, QP trajectory optimisation
- [ ] Phase 3 — moving obstacles, behaviour FSM, EKF, full evaluation, demo video

## License

MIT
