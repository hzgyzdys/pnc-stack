// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Pure Pursuit with speed-scheduled lookahead, a steering-rate limit and a
// P controller on speed. This is the baseline every later controller is
// compared against.

#pragma once

#include <cstddef>
#include <string>

#include "pnc_core/controller_interface.hpp"

namespace pnc_core {

struct PurePursuitParams {
  double lookahead_gain{0.8};     ///< Ld = gain * v + min
  double lookahead_min{0.5};      ///< [m]
  double lookahead_max{3.0};      ///< [m]
  double speed_kp{1.5};           ///< accel = kp * (v_ref - v)
  std::size_t search_window{50};  ///< waypoints searched ahead of last nearest index
};

class PurePursuit : public ControllerInterface {
 public:
  PurePursuit(const VehicleParams& vehicle, const PurePursuitParams& params);

  ControlCommand computeCommand(const State& state, const Path& path, double dt) override;
  void reset() override;
  std::string name() const override { return "pure_pursuit"; }

  /// Lookahead distance used in the last call (for logging and tests).
  double lastLookahead() const { return last_lookahead_; }

 private:
  VehicleParams vehicle_;
  PurePursuitParams params_;
  std::size_t last_nearest_{0};
  double last_steer_{0.0};
  double last_lookahead_{0.0};
};

}  // namespace pnc_core
