#pragma once

#include <algorithm>
#include <cmath>

namespace underbrush {

// UnitreeGo2HVActuator torque-speed envelope (N m, rad/s).
inline double clipGo2HVEffort(double requested, double velocity) {
  double limit = requested * velocity > 0.0 ? 20.2 : 23.4;
  if (std::abs(velocity) >= 13.5) {
    limit *= std::max(0.0, (30.0 - std::abs(velocity)) / 16.5);
  }
  return std::clamp(requested, -limit, limit);
}

// Model-derived torque; this does not replace the measured tau_est signal.
inline double modelTorque(double target, double position, double velocity,
                          double kp = 25.0, double kd = 0.5) {
  return clipGo2HVEffort(kp * (target - position) - kd * velocity, velocity);
}

}  // namespace underbrush
