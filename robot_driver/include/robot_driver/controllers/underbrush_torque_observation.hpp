#pragma once

#include <algorithm>
#include <cmath>

namespace underbrush {

// V90/V92 UnitreeGo2HVActuator algebra for the existing torque observation.
// This is model-derived torque, not a new motor command or measured tau_est.
inline double modelTorque(double target, double position, double velocity,
                          double kp = 25.0, double kd = 0.5) {
  const double requested = kp * (target - position) - kd * velocity;
  double limit = requested * velocity > 0.0 ? 20.2 : 23.4;
  if (std::abs(velocity) >= 13.5) {
    limit *= std::max(0.0, (30.0 - std::abs(velocity)) / 16.5);
  }
  return std::clamp(requested, -limit, limit);
}

}  // namespace underbrush
