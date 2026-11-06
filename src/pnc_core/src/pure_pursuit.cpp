// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include "pnc_core/pure_pursuit.hpp"

#include <cmath>

#include "pnc_core/math_utils.hpp"
#include "pnc_core/path_utils.hpp"

namespace pnc_core {

PurePursuit::PurePursuit(const VehicleParams& vehicle, const PurePursuitParams& params)
    : vehicle_(vehicle), params_(params) {}

void PurePursuit::reset() {
  last_nearest_ = 0;
  last_steer_ = 0.0;
  last_lookahead_ = 0.0;
}

ControlCommand PurePursuit::computeCommand(const State& state, const Path& path, double dt) {
  ControlCommand cmd;
  if (path.empty()) {
    cmd.accel = -vehicle_.max_decel;
    return cmd;
  }

  // 1. Nearest waypoint, searched forward from the previous one so the vehicle
  //    does not jump back on self-intersecting or closed paths.
  last_nearest_ = nearestIndex(path, state.x, state.y, last_nearest_, params_.search_window);

  // 2. Speed-scheduled lookahead distance.
  const double ld = clamp(params_.lookahead_gain * state.v + params_.lookahead_min,
                          params_.lookahead_min, params_.lookahead_max);
  last_lookahead_ = ld;

  // 3. First waypoint at least `ld` away from the vehicle; fall back to the end.
  std::size_t target = path.size() - 1;
  for (std::size_t i = last_nearest_; i < path.size(); ++i) {
    if (std::hypot(path[i].x - state.x, path[i].y - state.y) >= ld) {
      target = i;
      break;
    }
  }

  // 4. Pure Pursuit law: steer = atan(2 L sin(alpha) / Ld).
  const double alpha =
      normalizeAngle(std::atan2(path[target].y - state.y, path[target].x - state.x) - state.yaw);
  double steer = std::atan2(2.0 * vehicle_.wheelbase * std::sin(alpha), ld);

  // 5. Actuator limits: steering angle and steering rate.
  steer = clamp(steer, -vehicle_.max_steer, vehicle_.max_steer);
  if (dt > 0.0) {
    const double max_delta = vehicle_.max_steer_rate * dt;
    steer = clamp(steer, last_steer_ - max_delta, last_steer_ + max_delta);
  }
  last_steer_ = steer;

  // 6. Longitudinal: track v_ref of the nearest waypoint with a P controller.
  const double v_ref = clamp(path[last_nearest_].v_ref, 0.0, vehicle_.max_speed);
  cmd.steer = steer;
  cmd.accel = clamp(params_.speed_kp * (v_ref - state.v), -vehicle_.max_decel, vehicle_.max_accel);
  return cmd;
}

}  // namespace pnc_core
