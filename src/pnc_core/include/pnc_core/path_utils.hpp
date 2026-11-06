// Copyright (c) 2026 Hu Zhiguo. MIT License.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "pnc_core/types.hpp"

namespace pnc_core {

/// Index of the waypoint closest to (x, y). Searches the window
/// [hint, hint + window) when window > 0, otherwise the whole path.
/// Returns 0 for an empty path.
std::size_t nearestIndex(const Path& path, double x, double y, std::size_t hint = 0,
                         std::size_t window = 0);

/// Signed lateral distance from (x, y) to the path segment that starts at the
/// nearest waypoint. Positive when the point lies to the left of the path.
double crossTrackError(const Path& path, double x, double y);

/// Total arc length of the polyline.
double pathLength(const Path& path);

/// Recompute each waypoint's yaw from the direction to its successor.
void computeHeadings(Path& path);

/// Straight line from (x0, y0) along heading `yaw`, `n` points spaced `ds` apart.
Path makeStraightPath(double x0, double y0, double yaw, double ds, std::size_t n, double v_ref);

/// Counter-clockwise circle of radius r centred at (cx, cy), starting at angle 0.
Path makeCirclePath(double cx, double cy, double r, std::size_t n, double v_ref);

/// One piece of a reference path: a straight line (curvature 0) or a
/// circular arc. Positive curvature turns left.
struct PathSegment {
  double length{0.0};     ///< arc length [m], > 0
  double curvature{0.0};  ///< [1/m]
};

/// Parse "line <length>" or "arc <radius> <angle_deg>" (positive angle turns
/// left, negative right). Returns nullopt for malformed text or non-positive
/// length or radius.
std::optional<PathSegment> parseSegment(const std::string& text);

/// Chain segments from the start pose (x0, y0, yaw0) and sample them every
/// `ds` metres. Arcs are evaluated in closed form, so the path is exact at
/// every sample and the last sample is the exact end point.
Path makeSegmentPath(double x0, double y0, double yaw0, const std::vector<PathSegment>& segments,
                     double ds, double v_ref);

}  // namespace pnc_core
