#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

// #include <control_msgs/JointControllerState.h>
// #include <control_toolbox/pid.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>

#include <quad_msgs/msg/leg_command.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <quad_msgs/msg/motor_command.hpp>
#include <quad_utils/ros_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>
#include <map>

namespace effort_controllers {
/**
 * \brief Forward command controller for quad
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 */
class QuadController : public controller_interface::ControllerInterface {
  typedef std::vector<quad_msgs::msg::LegCommand> BufferType;

 public:
  QuadController();
  ~QuadController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::vector<std::string> joint_names_;
  //   std::vector<hardware_interface::JointHandle> joints_;

  std::vector<hardware_interface::LoanedCommandInterface> joint_cmd_handles_;
  std::vector<hardware_interface::LoanedStateInterface> joint_pos_handles_;
  std::vector<hardware_interface::LoanedStateInterface> joint_vel_handles_;

  realtime_tools::RealtimeBuffer<BufferType> commands_buffer_;
  unsigned int n_joints_;

  /// Subscriber for new LegCommandArray messages
  rclcpp::Subscription<quad_msgs::msg::LegCommandArray>::SharedPtr sub_command_;

  /// Store reference to gazebo joints
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  /// Map gazebo/urdf joint indices to leg/joint pair
  std::map<int, std::pair<int, int>> leg_map_;

  /// Torque limits for each motor
  std::vector<double> torque_lims_;
  std::vector<double> speed_lims_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  void commandCB(const quad_msgs::msg::LegCommandArray::SharedPtr msg);
  void enforceJointLimits(double& command, unsigned int index);

  // template <typename InterfaceType>
  // InterfaceType &get_interface(std::vector<InterfaceType> &interfaces, const
  // std::string &name, const std::string &type) { for (auto &interface :
  // interfaces) {
  //     const std::string
  //     if (interface.get_name() == name && interface.get_interface_name() ==
  //     type) { return interface;
  //     }
  // }
  // throw std::runtime_error("Interface not found: " + name + "/" + type);
  // }
  template <typename InterfaceType>
  InterfaceType& get_interface(std::vector<InterfaceType>& interfaces,
                               const std::string& joint_name,
                               const std::string& interface_type) {
    for (auto& interface : interfaces) {
      const std::string& full_name = interface.get_name();  // e.g., "0/effort"
      const std::string& iface_type =
          interface.get_interface_name();  // e.g., "effort"

      // Match against fully-qualified name
      if (full_name == joint_name + "/" + interface_type &&
          iface_type == interface_type) {
        return interface;
      }
    }

    throw std::runtime_error("Interface not found: " + joint_name + "/" +
                             interface_type);
  }
};  // class

}  // namespace effort_controllers
