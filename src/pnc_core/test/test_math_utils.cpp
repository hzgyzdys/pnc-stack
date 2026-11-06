// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include <gtest/gtest.h>

#include "pnc_core/math_utils.hpp"

using pnc_core::kPi;
using pnc_core::normalizeAngle;

TEST(MathUtils, NormalizeAngleRange) {
  for (double a = -20.0; a <= 20.0; a += 0.37) {
    const double n = normalizeAngle(a);
    EXPECT_GT(n, -kPi);
    EXPECT_LE(n, kPi);
    EXPECT_NEAR(std::sin(n), std::sin(a), 1e-9);
    EXPECT_NEAR(std::cos(n), std::cos(a), 1e-9);
  }
}

TEST(MathUtils, NormalizeAngleBoundaries) {
  EXPECT_NEAR(normalizeAngle(kPi), kPi, 1e-12);
  EXPECT_NEAR(normalizeAngle(-kPi), kPi, 1e-12);
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), kPi, 1e-9);
}

TEST(MathUtils, Clamp) {
  EXPECT_DOUBLE_EQ(pnc_core::clamp(5.0, 0.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(pnc_core::clamp(-5.0, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(pnc_core::clamp(0.5, 0.0, 1.0), 0.5);
}
