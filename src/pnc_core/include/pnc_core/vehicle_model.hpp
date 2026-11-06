// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Kinematic bicycle model referenced at the rear axle:
//   x'   = v cos(yaw)
//   y'   = v sin(yaw)
//   yaw' = v tan(steer) / L
//   v'   = a
// Used for offline closed-loop tests and, later, as the MPC prediction model.

#pragma once

#include "pnc_core/types.hpp"

namespace pnc_core {

class KinematicBicycle {
 public:
  explicit KinematicBicycle(const VehicleParams& params);

  /// Propagate `s` by `dt` seconds under `cmd` (RK4). Steering and acceleration
  /// are clamped to the vehicle limits; speed is clamped to [0, max_speed].
  State step(const State& s, const ControlCommand& cmd, double dt) const;

  /// Yaw rate produced by a steering angle at speed v. Useful when the actuator
  /// interface expects a Twist (v, omega) rather than (v, steer).
  double yawRate(double v, double steer) const;

  /// Inverse of yawRate. Returns 0 when |v| is below a small threshold.
  double steerFromYawRate(double v, double omega) const;

  const VehicleParams& params() const { return params_; }

 private:
  VehicleParams params_;
};

}  // namespace pnc_core
