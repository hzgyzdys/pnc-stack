// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Offline closed-loop simulation: kinematic bicycle + Pure Pursuit on a
// reference path. Writes a CSV log and prints tracking metrics. This is the
// first evaluation harness; Gazebo scenarios reuse the same metrics later.
//
// Usage: track_sim [circle|straight] [out.csv]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "pnc_core/math_utils.hpp"
#include "pnc_core/path_utils.hpp"
#include "pnc_core/pure_pursuit.hpp"
#include "pnc_core/vehicle_model.hpp"

using pnc_core::ControlCommand;
using pnc_core::Path;
using pnc_core::State;

int main(int argc, char** argv) {
  const std::string scenario = (argc > 1) ? argv[1] : "circle";
  const std::string out_path = (argc > 2) ? argv[2] : "track_sim.csv";

  pnc_core::VehicleParams vp;
  pnc_core::PurePursuitParams pp;
  pnc_core::KinematicBicycle model(vp);
  pnc_core::PurePursuit ctrl(vp, pp);

  Path path;
  State s;
  double sim_time = 0.0;
  if (scenario == "straight") {
    path = pnc_core::makeStraightPath(0.0, 0.0, 0.0, 0.1, 300, 1.5);
    s = {0.0, 0.5, 0.0, 0.0};  // 0.5 m lateral offset at start
    sim_time = 25.0;
  } else if (scenario == "circle") {
    path = pnc_core::makeCirclePath(0.0, 0.0, 3.0, 400, 1.0);
    s = {3.3, 0.0, pnc_core::kPi / 2.0, 0.0};
    sim_time = 18.0;  // roughly one lap at 1 m/s
  } else {
    std::cerr << "unknown scenario '" << scenario << "' (use circle|straight)\n";
    return 1;
  }

  std::ofstream csv(out_path);
  if (!csv) {
    std::cerr << "cannot open " << out_path << "\n";
    return 1;
  }
  csv << "t,x,y,yaw,v,steer,accel,cte\n";

  constexpr double dt = 0.02;  // 50 Hz control loop
  const int steps = static_cast<int>(sim_time / dt);
  double sum_sq = 0.0;
  double max_abs = 0.0;
  double max_compute_us = 0.0;
  // Skip the initial transient when computing steady-state metrics.
  const int settle_steps = static_cast<int>(5.0 / dt);
  int n_metric = 0;

  for (int k = 0; k < steps; ++k) {
    const auto t0 = std::chrono::steady_clock::now();
    const ControlCommand cmd = ctrl.computeCommand(s, path, dt);
    const auto t1 = std::chrono::steady_clock::now();
    max_compute_us =
        std::max(max_compute_us, std::chrono::duration<double, std::micro>(t1 - t0).count());

    const double cte = pnc_core::crossTrackError(path, s.x, s.y);
    csv << k * dt << ',' << s.x << ',' << s.y << ',' << s.yaw << ',' << s.v << ',' << cmd.steer
        << ',' << cmd.accel << ',' << cte << '\n';
    if (k >= settle_steps) {
      sum_sq += cte * cte;
      max_abs = std::max(max_abs, std::abs(cte));
      ++n_metric;
    }
    s = model.step(s, cmd, dt);
  }

  const double rms = n_metric > 0 ? std::sqrt(sum_sq / n_metric) : 0.0;
  std::printf("scenario=%s controller=%s\n", scenario.c_str(), ctrl.name().c_str());
  std::printf("steady-state cross-track error: rms=%.4f m, max=%.4f m\n", rms, max_abs);
  std::printf("max compute time per step: %.1f us\n", max_compute_us);
  std::printf("log written to %s\n", out_path.c_str());
  return 0;
}
