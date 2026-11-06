// Copyright (c) 2026 Hu Zhiguo. MIT License.

#include "pnc_core/vehicle_model.hpp"

#include <cmath>

#include "pnc_core/math_utils.hpp"

namespace pnc_core {

namespace {

struct Deriv {
  double dx, dy, dyaw, dv;
};

Deriv f(const State& s, double steer, double accel, double L) {
  return {s.v * std::cos(s.yaw), s.v * std::sin(s.yaw), s.v * std::tan(steer) / L, accel};
}

State add(const State& s, const Deriv& d, double h) {
  return {s.x + h * d.dx, s.y + h * d.dy, s.yaw + h * d.dyaw, s.v + h * d.dv};
}

}  // namespace

KinematicBicycle::KinematicBicycle(const VehicleParams& params) : params_(params) {}

State KinematicBicycle::step(const State& s, const ControlCommand& cmd, double dt) const {
  const double steer = clamp(cmd.steer, -params_.max_steer, params_.max_steer);
  const double accel = clamp(cmd.accel, -params_.max_decel, params_.max_accel);
  const double L = params_.wheelbase;

  const Deriv k1 = f(s, steer, accel, L);
  const Deriv k2 = f(add(s, k1, dt / 2.0), steer, accel, L);
  const Deriv k3 = f(add(s, k2, dt / 2.0), steer, accel, L);
  const Deriv k4 = f(add(s, k3, dt), steer, accel, L);

  State out;
  out.x = s.x + dt / 6.0 * (k1.dx + 2 * k2.dx + 2 * k3.dx + k4.dx);
  out.y = s.y + dt / 6.0 * (k1.dy + 2 * k2.dy + 2 * k3.dy + k4.dy);
  out.yaw = normalizeAngle(s.yaw + dt / 6.0 * (k1.dyaw + 2 * k2.dyaw + 2 * k3.dyaw + k4.dyaw));
  out.v = clamp(s.v + dt * accel, 0.0, params_.max_speed);
  return out;
}

double KinematicBicycle::yawRate(double v, double steer) const {
  return v * std::tan(steer) / params_.wheelbase;
}

double KinematicBicycle::steerFromYawRate(double v, double omega) const {
  constexpr double kMinSpeed = 1e-3;
  if (std::abs(v) < kMinSpeed) {
    return 0.0;
  }
  return std::atan(omega * params_.wheelbase / v);
}

}  // namespace pnc_core
