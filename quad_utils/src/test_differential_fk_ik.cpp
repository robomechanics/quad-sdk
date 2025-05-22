

#include <rclcpp/rclcpp.hpp>

#include "quad_utils/quad_kd.h"
#include "quad_utils/ros_utils.h"

// #include <gtest/gtest.h>

#include <grid_map_core/grid_map_core.hpp>

using namespace quad_utils;

const double kinematics_tol = 1e-4;

bool runDifferentialFKIKTest(const std::shared_ptr<rclcpp::Node> &node) {
  // Pass node to kinematics object
  QuadKD kinematics(node);

  for (size_t i = 0; i < 20; i++) {
    quad_msgs::msg::RobotState state, state_out;

    Eigen::VectorXd body_state(12);
    body_state << (double)rand() / RAND_MAX - 0.5,
        (double)rand() / RAND_MAX - 0.5, (double)rand() / RAND_MAX - 0.5,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        10 * (double)rand() / RAND_MAX - 5, 10 * (double)rand() / RAND_MAX - 5,
        10 * (double)rand() / RAND_MAX - 5,
        3.14 * (double)rand() / RAND_MAX - 1.57,
        3.14 * (double)rand() / RAND_MAX - 1.57,
        3.14 * (double)rand() / RAND_MAX - 1.57;

    state.body = eigenToBodyStateMsg(body_state);

    state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                         "10", "4", "5", "11", "6", "7"};
    state.joints.position.clear();
    state.joints.velocity.clear();
    state.joints.effort.clear();

    for (int j = 0; j < 4; j++) {
      state.joints.position.push_back(0.1);
      state.joints.position.push_back(0.2);
      state.joints.position.push_back(0.3);

      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);

      state.joints.effort.push_back(0.0);
      state.joints.effort.push_back(0.0);
      state.joints.effort.push_back(0.0);
    }

    fkRobotState(kinematics, state.body, state.joints, state.feet);
    ikRobotState(kinematics, state.body, state.feet, state_out.joints);

    Eigen::VectorXd vel(12), vel_out(12);
    vectorToEigen(state.joints.velocity, vel);
    vectorToEigen(state_out.joints.velocity, vel_out);

    Eigen::VectorXd error = vel - vel_out;
    // EXPECT_TRUE(error.norm() <= kinematics_tol);
    if (error.norm() > kinematics_tol) {
      std::cerr << "[FAIL] Iteration " << i
                << ": Velocity error norm = " << error.norm() << std::endl;
      return false;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("differential_fk_ik_test_node");

  bool success = runDifferentialFKIKTest(node);

  rclcpp::shutdown();
  return success ? 0 : 1;
}