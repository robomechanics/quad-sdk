#pragma once
#include <algorithm>
#include <cmath>

namespace effort_controllers {
// Matches underbrush UnitreeGo2HVActuator::_clip_effort (N m, rad/s).
inline double clipGo2HVEffort(double effort, double velocity) {
  double limit = velocity * effort > 0.0 ? 20.2 : 23.4;
  if (std::abs(velocity) >= 13.5) {
    limit = std::max(0.0, limit * (30.0 - std::abs(velocity)) / (30.0 - 13.5));
  }
  return std::min(std::max(effort, -limit), limit);
}
}  // namespace effort_controllers
