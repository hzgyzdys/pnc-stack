// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include <gtest/gtest.h>

#include <cmath>

#include "pnc_core/math_utils.hpp"
#include "pnc_core/path_utils.hpp"

using pnc_core::Path;

TEST(PathUtils, StraightPathLengthAndHeading) {
  const Path p = pnc_core::makeStraightPath(0.0, 0.0, pnc_core::kPi / 4.0, 0.1, 11, 1.0);
  EXPECT_NEAR(pnc_core::pathLength(p), 1.0, 1e-9);
  for (const auto& w : p) {
    EXPECT_NEAR(w.yaw, pnc_core::kPi / 4.0, 1e-12);
  }
}

TEST(PathUtils, CircleLengthApproachesCircumference) {
  const Path p = pnc_core::makeCirclePath(0.0, 0.0, 2.0, 1000, 1.0);
  // Open polyline over 999 of 1000 segments.
  EXPECT_NEAR(pnc_core::pathLength(p), 2.0 * pnc_core::kPi * 2.0 * 999.0 / 1000.0, 1e-3);
}

TEST(PathUtils, NearestIndex) {
  const Path p = pnc_core::makeStraightPath(0.0, 0.0, 0.0, 1.0, 10, 1.0);
  EXPECT_EQ(pnc_core::nearestIndex(p, 3.2, 0.5), 3u);
  EXPECT_EQ(pnc_core::nearestIndex(p, 100.0, 0.0), 9u);
  // Windowed search never returns an index before the hint.
  EXPECT_EQ(pnc_core::nearestIndex(p, 0.0, 0.0, 5, 3), 5u);
  EXPECT_EQ(pnc_core::nearestIndex(Path{}, 0.0, 0.0), 0u);
}

TEST(PathUtils, CrossTrackErrorSign) {
  const Path p = pnc_core::makeStraightPath(0.0, 0.0, 0.0, 0.5, 20, 1.0);
  EXPECT_NEAR(pnc_core::crossTrackError(p, 2.1, 0.3), 0.3, 1e-9);    // left of path
  EXPECT_NEAR(pnc_core::crossTrackError(p, 2.1, -0.4), -0.4, 1e-9);  // right of path
}

TEST(PathUtils, ComputeHeadings) {
  Path p{{0, 0, 0, 1}, {1, 1, 0, 1}, {1, 2, 0, 1}};
  pnc_core::computeHeadings(p);
  EXPECT_NEAR(p[0].yaw, pnc_core::kPi / 4.0, 1e-12);
  EXPECT_NEAR(p[1].yaw, pnc_core::kPi / 2.0, 1e-12);
  EXPECT_NEAR(p[2].yaw, pnc_core::kPi / 2.0, 1e-12);
}
