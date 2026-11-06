// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Plain data types shared by all pnc_core modules. No ROS dependency.

#pragma once

#include <vector>

namespace pnc_core {

/// Vehicle state at the rear-axle centre, expressed in the map frame.
struct State {
  double x{0.0};    ///< [m]
  double y{0.0};    ///< [m]
  double yaw{0.0};  ///< [rad], wrapped to (-pi, pi]
  double v{0.0};    ///< longitudinal speed [m/s]
};

/// A single reference point. `v_ref` is the desired speed at this point.
struct Waypoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v_ref{0.0};
};

using Path = std::vector<Waypoint>;

/// Low-level command for an Ackermann vehicle.
struct ControlCommand {
  double steer{0.0};  ///< front-wheel steering angle [rad]
  double accel{0.0};  ///< longitudinal acceleration [m/s^2]
};

/// Physical limits of the vehicle. Defaults approximate a 1:10 scale car.
struct VehicleParams {
  double wheelbase{0.33};      ///< [m]
  double max_steer{0.45};      ///< [rad]
  double max_steer_rate{2.0};  ///< [rad/s]
  double max_accel{2.0};       ///< [m/s^2]
  double max_decel{3.0};       ///< [m/s^2], positive number
  double max_speed{2.0};       ///< [m/s]
};

}  // namespace pnc_core
