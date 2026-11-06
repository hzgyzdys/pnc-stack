// Copyright (c) 2026 Hu Zhiguo. MIT License.

#pragma once

#include <cstddef>

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

}  // namespace pnc_core
