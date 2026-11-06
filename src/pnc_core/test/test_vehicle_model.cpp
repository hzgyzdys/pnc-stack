// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include <gtest/gtest.h>

#include <cmath>

#include "pnc_core/math_utils.hpp"
#include "pnc_core/vehicle_model.hpp"

using pnc_core::ControlCommand;
using pnc_core::KinematicBicycle;
using pnc_core::State;
using pnc_core::VehicleParams;

TEST(KinematicBicycle, StraightLineAtConstantSpeed) {
  KinematicBicycle m(VehicleParams{});
  State s{0.0, 0.0, 0.0, 1.0};
  for (int i = 0; i < 100; ++i) {
    s = m.step(s, ControlCommand{0.0, 0.0}, 0.01);
  }
  EXPECT_NEAR(s.x, 1.0, 1e-9);
  EXPECT_NEAR(s.y, 0.0, 1e-12);
  EXPECT_NEAR(s.yaw, 0.0, 1e-12);
}

TEST(KinematicBicycle, ConstantSteerGivesExpectedTurningRadius) {
  VehicleParams p;
  KinematicBicycle m(p);
  const double steer = 0.3;
  const double R = p.wheelbase / std::tan(steer);
  const double v = 1.0;
  // Drive half a circle: time = pi R / v. Start heading +x, centre at (0, R).
  State s{0.0, 0.0, 0.0, v};
  const double dt = 0.001;
  const int n = static_cast<int>(pnc_core::kPi * R / v / dt);
  for (int i = 0; i < n; ++i) {
    s = m.step(s, ControlCommand{steer, 0.0}, dt);
  }
  EXPECT_NEAR(std::hypot(s.x - 0.0, s.y - R), R, 1e-3);  // stays on the circle
  EXPECT_NEAR(s.y, 2.0 * R, 5e-3);                       // reached the far side
}

TEST(KinematicBicycle, LimitsAreEnforced) {
  VehicleParams p;
  KinematicBicycle m(p);
  // Steering beyond the limit must behave like the limit.
  const State s0{0.0, 0.0, 0.0, 1.0};
  const State a = m.step(s0, ControlCommand{10.0, 0.0}, 0.1);
  const State b = m.step(s0, ControlCommand{p.max_steer, 0.0}, 0.1);
  EXPECT_NEAR(a.yaw, b.yaw, 1e-12);
  // Speed never goes negative or above max_speed.
  State s{0.0, 0.0, 0.0, 0.1};
  s = m.step(s, ControlCommand{0.0, -100.0}, 1.0);
  EXPECT_GE(s.v, 0.0);
  for (int i = 0; i < 100; ++i) {
    s = m.step(s, ControlCommand{0.0, 100.0}, 0.1);
  }
  EXPECT_LE(s.v, p.max_speed);
}

TEST(KinematicBicycle, YawRateRoundTrip) {
  KinematicBicycle m(VehicleParams{});
  for (double steer = -0.4; steer <= 0.4; steer += 0.05) {
    const double omega = m.yawRate(1.2, steer);
    EXPECT_NEAR(m.steerFromYawRate(1.2, omega), steer, 1e-12);
  }
  EXPECT_DOUBLE_EQ(m.steerFromYawRate(0.0, 1.0), 0.0);
}
