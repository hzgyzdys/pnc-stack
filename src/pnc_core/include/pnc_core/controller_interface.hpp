// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Common interface for every path-tracking controller (Pure Pursuit, Stanley,
// LQR, MPC). The ROS 2 node in pnc_control only talks to this interface, so a
// new controller is added by implementing it and registering it in the factory.

#pragma once

#include <string>

#include "pnc_core/types.hpp"

namespace pnc_core {

class ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;

  /// Compute one command. `dt` is the control period [s]. Implementations may
  /// keep internal state (e.g. previous steer for rate limiting, warm starts).
  virtual ControlCommand computeCommand(const State& state, const Path& path, double dt) = 0;

  /// Clear internal state, e.g. when a new path arrives.
  virtual void reset() = 0;

  virtual std::string name() const = 0;
};

}  // namespace pnc_core
