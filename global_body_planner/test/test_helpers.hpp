#ifndef GLOBAL_BODY_PLANNER_TEST_HELPERS_HPP
#define GLOBAL_BODY_PLANNER_TEST_HELPERS_HPP

#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <stdexcept>

#include "global_body_planner/planning_utils.hpp"

namespace global_body_planner_test {

constexpr double kTol = 1e-6;

inline planning_utils::State makeState(double x, double y, double z, double vx,
                                       double vy, double vz) {
  planning_utils::State s;
  s.pos << x, y, z;
  s.vel << vx, vy, vz;
  return s;
}

inline planning_utils::TimedPoseConstraint makeConstraint(
    const Eigen::Vector3d& pos, double yaw,
    const Eigen::Vector3d& half_extents, double t_start, double t_end) {
  planning_utils::TimedPoseConstraint c;
  c.pos = pos;
  c.yaw = yaw;
  c.half_extents = half_extents;
  c.t_start = t_start;
  c.t_end = t_end;
  return c;
}

inline rclcpp::NodeOptions plannerNodeOptions(bool include_topics = false) {
  const char* planner_params =
      std::getenv("GLOBAL_BODY_PLANNER_TEST_PARAMS");
  const char* robot_params = std::getenv("GLOBAL_BODY_PLANNER_ROBOT_PARAMS");
  if (planner_params == nullptr || robot_params == nullptr) {
    throw std::runtime_error(
        "Missing global_body_planner test parameter file environment");
  }

  std::vector<std::string> args = {"--ros-args", "--params-file",
                                   planner_params, "--params-file",
                                   robot_params};
  if (include_topics) {
    const char* topic_params =
        std::getenv("GLOBAL_BODY_PLANNER_TOPIC_PARAMS");
    if (topic_params == nullptr) {
      throw std::runtime_error(
          "Missing global_body_planner topic parameter file environment");
    }
    args.push_back("--params-file");
    args.push_back(topic_params);
  }

  rclcpp::NodeOptions options;
  options.arguments(args);
  return options;
}

}  // namespace global_body_planner_test

#endif  // GLOBAL_BODY_PLANNER_TEST_HELPERS_HPP
