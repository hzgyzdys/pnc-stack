// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

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

TEST(PathUtils, ParseSegment) {
  const auto line = pnc_core::parseSegment("line 2.5");
  ASSERT_TRUE(line.has_value());
  EXPECT_DOUBLE_EQ(line->length, 2.5);
  EXPECT_DOUBLE_EQ(line->curvature, 0.0);

  const auto right = pnc_core::parseSegment("arc 2.0 -90");
  ASSERT_TRUE(right.has_value());
  EXPECT_NEAR(right->length, pnc_core::kPi, 1e-12);
  EXPECT_DOUBLE_EQ(right->curvature, -0.5);

  EXPECT_FALSE(pnc_core::parseSegment("").has_value());
  EXPECT_FALSE(pnc_core::parseSegment("line -1").has_value());
  EXPECT_FALSE(pnc_core::parseSegment("arc 0 90").has_value());
  EXPECT_FALSE(pnc_core::parseSegment("arc 1.0").has_value());
  EXPECT_FALSE(pnc_core::parseSegment("spiral 1 2").has_value());
}

TEST(PathUtils, SegmentPathUTurn) {
  // 2 m straight, left half circle of radius 1, 2 m straight back.
  const std::vector<pnc_core::PathSegment> segs = {
      *pnc_core::parseSegment("line 2"),
      *pnc_core::parseSegment("arc 1 180"),
      *pnc_core::parseSegment("line 2"),
  };
  const Path p = pnc_core::makeSegmentPath(0.0, 0.0, 0.0, segs, 0.05, 1.0);
  const auto& end = p.back();
  EXPECT_NEAR(end.x, 0.0, 1e-9);
  EXPECT_NEAR(end.y, 2.0, 1e-9);
  EXPECT_NEAR(std::abs(end.yaw), pnc_core::kPi, 1e-9);
  EXPECT_NEAR(pnc_core::pathLength(p), 4.0 + pnc_core::kPi, 1e-3);
  for (std::size_t i = 1; i < p.size(); ++i) {
    const double gap = std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    EXPECT_LE(gap, 0.05 + 1e-9);
  }
}

TEST(PathUtils, SegmentPathStartPose) {
  const std::vector<pnc_core::PathSegment> segs = {{1.0, 0.0}};
  const Path p = pnc_core::makeSegmentPath(1.0, 0.5, pnc_core::kPi / 2.0, segs, 0.1, 1.0);
  ASSERT_EQ(p.size(), 11u);
  EXPECT_NEAR(p.front().y, 0.5, 1e-12);
  EXPECT_NEAR(p.back().x, 1.0, 1e-9);
  EXPECT_NEAR(p.back().y, 1.5, 1e-9);
}
