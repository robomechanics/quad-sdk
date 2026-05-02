#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <quad_msgs/msg/leg_command.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <quad_msgs/msg/motor_command.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <urdf/model.h>

#include <map>
#include <string>
#include <vector>

namespace effort_controllers {

//! Effort controller for wheeled quadrupeds (Go2-W and similar).
/*!
   QuadWheelController extends QuadController's leg behaviour to
   additionally drive four wheel motors. For each leg (4 legs in
   quad-sdk order FL, RL, FR, RR), the inbound LegCommand carries:

       motor_commands[0] = abad   (PD + tau_ff, effort interface)
       motor_commands[1] = hip    (PD + tau_ff, effort interface)
       motor_commands[2] = knee   (PD + tau_ff, effort interface)
       motor_commands[3] = wheel  (velocity control: kp ignored,
                                   kd*(vel_setpoint - vel) + tau_ff,
                                   effort interface)

   The wheel slot is read only when the LegCommand's motor_commands array
   has at least 4 entries. If a leg arrives with only 3 entries (legacy
   quadruped command), wheels coast with zero effort.

   Joint names are pulled from the robot yaml:
       leg_i.joints.{abad,hip,knee}.name   - leg motors
       leg_i.joints.wheel.name             - wheel motor (NEW)
*/
class QuadWheelController : public controller_interface::ControllerInterface {
  using BufferType = std::vector<quad_msgs::msg::LegCommand>;

 public:
  QuadWheelController();
  ~QuadWheelController();

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
  static constexpr int kNumLegs = 4;
  static constexpr int kJointsPerLeg = 3;
  static constexpr int kNumLegJoints = kNumLegs * kJointsPerLeg;
  static constexpr int kNumWheels = kNumLegs;

  // joint_names_ layout: first 12 entries are leg joints in
  // leg_i × {abad, hip, knee} order, last 4 entries are wheel joints
  // in leg_i order.
  std::vector<std::string> joint_names_;

  std::vector<hardware_interface::LoanedCommandInterface> joint_cmd_handles_;
  std::vector<hardware_interface::LoanedStateInterface> joint_pos_handles_;
  std::vector<hardware_interface::LoanedStateInterface> joint_vel_handles_;

  realtime_tools::RealtimeBuffer<BufferType> commands_buffer_;
  unsigned int n_joints_ = 0;

  rclcpp::Subscription<quad_msgs::msg::LegCommandArray>::SharedPtr sub_command_;

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  // For leg joints (index < 12): leg_map_[i] = (leg_idx, motor_idx in {0,1,2})
  // For wheel joints (12 <= i < 16): leg_map_[i] = (leg_idx, 3)
  std::map<int, std::pair<int, int>> leg_map_;

  std::vector<double> torque_lims_;
  std::vector<double> speed_lims_;
  std::vector<double> wheel_torque_lims_;
  std::vector<double> wheel_speed_lims_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  bool load_wheel_joint_names(std::vector<std::string>& wheel_names) const;
  bool load_wheel_motor_limits(std::vector<double>& torque_lims,
                               std::vector<double>& speed_lims) const;

  void commandCB(const quad_msgs::msg::LegCommandArray::SharedPtr msg);
  void enforceJointLimits(double& command, unsigned int index);

  template <typename InterfaceType>
  InterfaceType& get_interface(std::vector<InterfaceType>& interfaces,
                               const std::string& joint_name,
                               const std::string& interface_type) {
    for (auto& interface : interfaces) {
      const std::string& full_name = interface.get_name();
      const std::string& iface_type = interface.get_interface_name();
      if (full_name == joint_name + "/" + interface_type &&
          iface_type == interface_type) {
        return interface;
      }
    }
    throw std::runtime_error("Interface not found: " + joint_name + "/" +
                             interface_type);
  }
};

}  // namespace effort_controllers
