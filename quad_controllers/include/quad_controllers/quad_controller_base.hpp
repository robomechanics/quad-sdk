#ifndef QUAD_CONTROLLERS__QUAD_CONTROLLER_BASE_HPP_
#define QUAD_CONTROLLERS__QUAD_CONTROLLER_BASE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "quad_msgs/msg/leg_command_array.hpp"

namespace quad_controllers {

//! Shared plumbing for the switchable quad ros2_control controllers.
//! Factors out of the former ControlModeController everything that is common to
//! every control mode: parameter declaration (joints, interface_mode,
//! robot_namespace, torque_limits), the command/state interface_configuration,
//! the on_activate index maps, writeToInterfaces (effort vs motor mode),
//! measured position/velocity reads and the joint name table. Derived classes
//! implement a single computeCommand() and the base update() runs it, writes to
//! the hardware interfaces and publishes control/joint_command.
class QuadControllerBase : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  //! Derived-class command computation. Return false to indicate that the
  //! controller has nothing useful to output (the base still writes whatever is
  //! in `out`, so derived classes that can fail should fill a fallback).
  virtual bool computeCommand(quad_msgs::msg::LegCommandArray& out) = 0;

  //! Optional extension hooks for derived classes.
  virtual controller_interface::CallbackReturn onInitExtra() {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  virtual controller_interface::CallbackReturn onConfigureExtra() {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Shared helpers (verbatim behavior from ControlModeController).
  void writeToInterfaces(const quad_msgs::msg::LegCommandArray& out);
  double measuredPosition(const std::string& joint) const;
  double measuredVelocity(const std::string& joint) const;
  static const char* jointName(int leg, int j);

  // Shared parameters.
  std::string interface_mode_{"effort"};
  std::string robot_ns_;
  std::vector<std::string> joint_names_;
  std::vector<double> torque_limits_;

  // Output publisher.
  rclcpp::Publisher<quad_msgs::msg::LegCommandArray>::SharedPtr leg_command_pub_;

  const int num_feet_{4};
  std::unordered_map<std::string, size_t> cmd_index_;
  std::unordered_map<std::string, size_t> state_index_;
};

}  // namespace quad_controllers

#endif  // QUAD_CONTROLLERS__QUAD_CONTROLLER_BASE_HPP_
