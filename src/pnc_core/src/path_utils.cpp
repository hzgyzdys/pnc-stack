// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include "pnc_core/path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "pnc_core/math_utils.hpp"

namespace pnc_core {

std::size_t nearestIndex(const Path& path, double x, double y, std::size_t hint,
                         std::size_t window) {
  if (path.empty()) {
    return 0;
  }
  const std::size_t begin = std::min(hint, path.size() - 1);
  const std::size_t end = (window == 0) ? path.size() : std::min(path.size(), begin + window);

  std::size_t best = begin;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = begin; i < end; ++i) {
    const double dx = path[i].x - x;
    const double dy = path[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

double crossTrackError(const Path& path, double x, double y) {
  if (path.size() < 2) {
    return 0.0;
  }
  std::size_t i = nearestIndex(path, x, y);
  if (i + 1 >= path.size()) {
    i = path.size() - 2;
  }
  const double sx = path[i + 1].x - path[i].x;
  const double sy = path[i + 1].y - path[i].y;
  const double seg = std::hypot(sx, sy);
  if (seg < 1e-9) {
    return std::hypot(x - path[i].x, y - path[i].y);
  }
  // 2D cross product of segment direction with vector to the point.
  return (sx * (y - path[i].y) - sy * (x - path[i].x)) / seg;
}

double pathLength(const Path& path) {
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

void computeHeadings(Path& path) {
  if (path.size() < 2) {
    return;
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  path.back().yaw = path[path.size() - 2].yaw;
}

Path makeStraightPath(double x0, double y0, double yaw, double ds, std::size_t n, double v_ref) {
  Path p;
  p.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double s = ds * static_cast<double>(i);
    p.push_back({x0 + s * std::cos(yaw), y0 + s * std::sin(yaw), yaw, v_ref});
  }
  return p;
}

Path makeCirclePath(double cx, double cy, double r, std::size_t n, double v_ref) {
  Path p;
  p.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double th = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    p.push_back(
        {cx + r * std::cos(th), cy + r * std::sin(th), normalizeAngle(th + kPi / 2.0), v_ref});
  }
  return p;
}

std::optional<PathSegment> parseSegment(const std::string& text) {
  std::istringstream in(text);
  std::string kind;
  in >> kind;
  if (kind == "line") {
    double length = 0.0;
    if (!(in >> length) || length <= 0.0) {
      return std::nullopt;
    }
    return PathSegment{length, 0.0};
  }
  if (kind == "arc") {
    double radius = 0.0;
    double angle_deg = 0.0;
    if (!(in >> radius >> angle_deg) || radius <= 0.0 || angle_deg == 0.0) {
      return std::nullopt;
    }
    const double angle = angle_deg * kPi / 180.0;
    return PathSegment{radius * std::abs(angle), std::copysign(1.0 / radius, angle)};
  }
  return std::nullopt;
}

Path makeSegmentPath(double x0, double y0, double yaw0, const std::vector<PathSegment>& segments,
                     double ds, double v_ref) {
  Path path;
  if (ds <= 0.0) {
    return path;
  }
  double x = x0;
  double y = y0;
  double yaw = yaw0;
  path.push_back({x, y, normalizeAngle(yaw), v_ref});

  for (const auto& seg : segments) {
    if (seg.length <= 0.0) {
      continue;
    }
    const auto n = static_cast<std::size_t>(std::ceil(seg.length / ds - 1e-9));
    const double step = seg.length / static_cast<double>(n);
    const double k = seg.curvature;
    // Pose after travelling s along this segment from (x, y, yaw).
    auto poseAt = [&](double s) {
      Waypoint w{};
      if (std::abs(k) < 1e-12) {
        w.x = x + s * std::cos(yaw);
        w.y = y + s * std::sin(yaw);
        w.yaw = yaw;
      } else {
        w.yaw = yaw + k * s;
        w.x = x + (std::sin(w.yaw) - std::sin(yaw)) / k;
        w.y = y - (std::cos(w.yaw) - std::cos(yaw)) / k;
      }
      return w;
    };
    for (std::size_t i = 1; i <= n; ++i) {
      Waypoint w = poseAt(step * static_cast<double>(i));
      w.yaw = normalizeAngle(w.yaw);
      w.v_ref = v_ref;
      path.push_back(w);
    }
    const Waypoint end = poseAt(seg.length);
    x = end.x;
    y = end.y;
    yaw = end.yaw;
  }
  return path;
}

}  // namespace pnc_core
