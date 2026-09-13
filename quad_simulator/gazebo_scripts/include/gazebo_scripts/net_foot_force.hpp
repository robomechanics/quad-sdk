#pragma once
#include <array>
#include <string>
namespace gazebo_scripts {
// Sum forces on this toe from every collision partner, in the message frame.
// Select the wrench for the toe regardless of collision-pair ordering.
template<class Contacts>
std::array<double, 3> netFootForce(const Contacts& msg, const std::string& toe) {
  std::array<double, 3> sum{};
  for (const auto& contact : msg.contacts) {
    const bool first = contact.collision1.name.find(toe) != std::string::npos;
    const bool second = contact.collision2.name.find(toe) != std::string::npos;
    if (first == second) continue;
    for (const auto& wrench : contact.wrenches) {
      const auto& force = first ? wrench.body_1_wrench.force : wrench.body_2_wrench.force;
      sum[0] += force.x; sum[1] += force.y; sum[2] += force.z;
    }
  }
  return sum;
}
}
