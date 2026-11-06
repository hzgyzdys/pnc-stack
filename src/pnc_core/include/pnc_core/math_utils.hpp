// Copyright (c) 2026 Hu Zhiguo. MIT License.

#pragma once

#include <algorithm>
#include <cmath>

namespace pnc_core {

inline constexpr double kPi = 3.14159265358979323846;

/// Wrap an angle to (-pi, pi].
inline double normalizeAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

inline double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

}  // namespace pnc_core
