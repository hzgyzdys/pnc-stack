// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Closed-loop tests: the controller drives the kinematic model, and the
// assertions are on tracking performance rather than on internal values.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "pnc_core/math_utils.hpp"
#include "pnc_core/path_utils.hpp"
#include "pnc_core/pure_pursuit.hpp"
#include "pnc_core/vehicle_model.hpp"

using namespace pnc_core;

namespace {

struct RunResult {
  double final_abs_cte;
  double max_abs_cte_after_settle;
  double final_speed;
  State final_state;
};

RunResult runClosedLoop(const Path& path, State s, double sim_time, double settle_time) {
  VehicleParams vp;
  KinematicBicycle model(vp);
  PurePursuit ctrl(vp, PurePursuitParams{});
  const double dt = 0.02;
  const int steps = static_cast<int>(sim_time / dt);
  const int settle = static_cast<int>(settle_time / dt);
  double max_after = 0.0;
  for (int k = 0; k < steps; ++k) {
    s = model.step(s, ctrl.computeCommand(s, path, dt), dt);
    if (k >= settle) {
      max_after = std::max(max_after, std::abs(crossTrackError(path, s.x, s.y)));
    }
  }
  return {std::abs(crossTrackError(path, s.x, s.y)), max_after, s.v, s};
}

}  // namespace

TEST(PurePursuit, ConvergesToStraightLineFromLateralOffset) {
  const Path path = makeStraightPath(0.0, 0.0, 0.0, 0.1, 400, 1.5);
  const RunResult r = runClosedLoop(path, State{0.0, 0.5, 0.0, 0.0}, 20.0, 6.0);
  EXPECT_LT(r.max_abs_cte_after_settle, 0.02);
  EXPECT_NEAR(r.final_speed, 1.5, 0.05);
}

TEST(PurePursuit, TracksCircleWithBoundedError) {
  // Pure Pursuit has a known steady-state inward bias on curves; the bound
  // below documents the baseline that later controllers must beat.
  const Path path = makeCirclePath(0.0, 0.0, 3.0, 400, 1.0);
  const RunResult r = runClosedLoop(path, State{3.3, 0.0, kPi / 2.0, 0.0}, 15.0, 5.0);
  EXPECT_LT(r.max_abs_cte_after_settle, 0.10);
}

TEST(PurePursuit, StopsAtEndOfPathWhenRefSpeedIsZero) {
  Path path = makeStraightPath(0.0, 0.0, 0.0, 0.1, 50, 1.0);
  for (std::size_t i = 40; i < path.size(); ++i) {
    path[i].v_ref = 0.0;
  }
  const RunResult r = runClosedLoop(path, State{0.0, 0.0, 0.0, 0.0}, 15.0, 0.0);
  EXPECT_LT(r.final_speed, 1e-3);
  EXPECT_LT(r.final_state.x, 5.5);
}

TEST(PurePursuit, RespectsSteeringRateLimit) {
  VehicleParams vp;
  PurePursuit ctrl(vp, PurePursuitParams{});
  // Target far to the left forces a large steering request.
  const Path path = makeStraightPath(0.0, 5.0, kPi / 2.0, 0.1, 100, 1.0);
  const double dt = 0.02;
  const ControlCommand c = ctrl.computeCommand(State{0.0, 0.0, 0.0, 1.0}, path, dt);
  EXPECT_LE(std::abs(c.steer), vp.max_steer_rate * dt + 1e-12);
}

TEST(PurePursuit, LookaheadIsSpeedScheduledAndClamped) {
  VehicleParams vp;
  PurePursuitParams pp;
  PurePursuit ctrl(vp, pp);
  const Path path = makeStraightPath(0.0, 0.0, 0.0, 0.1, 200, 1.0);
  ctrl.computeCommand(State{0.0, 0.0, 0.0, 0.0}, path, 0.02);
  EXPECT_DOUBLE_EQ(ctrl.lastLookahead(), pp.lookahead_min);
  ctrl.computeCommand(State{0.0, 0.0, 0.0, 100.0}, path, 0.02);
  EXPECT_DOUBLE_EQ(ctrl.lastLookahead(), pp.lookahead_max);
}

TEST(PurePursuit, EmptyPathCommandsFullBrake) {
  VehicleParams vp;
  PurePursuit ctrl(vp, PurePursuitParams{});
  const ControlCommand c = ctrl.computeCommand(State{0.0, 0.0, 0.0, 1.0}, Path{}, 0.02);
  EXPECT_DOUBLE_EQ(c.steer, 0.0);
  EXPECT_DOUBLE_EQ(c.accel, -vp.max_decel);
}
