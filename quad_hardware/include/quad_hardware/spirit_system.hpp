#ifndef QUAD_HARDWARE__SPIRIT_SYSTEM_HPP_
#define QUAD_HARDWARE__SPIRIT_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <eigen3/Eigen/Eigen>
#include <mblink/mblink.hpp>

namespace quad_hardware {

//! ros2_control SystemInterface for the Ghost Robotics Spirit 40.
/*!
   Wraps the MBLink mainboard protocol behind the ros2_control
   hardware_interface API, mirroring the legacy robot_driver SpiritInterface
   (send()/recv()) field-for-field so behavior is preserved.

   The read_only=true default and the NaN command guard keep bring-up safe;
   see UnitreeSystem for the same safety model.

   Like the Go2, Spirit joints run an onboard PD loop
   (tau = kp*(position - q) + kd*(velocity - dq) + effort), so each joint
   exposes FIVE command interfaces: position, velocity, kp, kd, effort.
*/
class SpiritSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SpiritSystem)

  static constexpr int kNumLegs = 4;
  static constexpr int kJointsPerLeg = 3;
  static constexpr int kNumJoints = kNumLegs * kJointsPerLeg;  // 12

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 protected:
  // URDF numeric joint name -> MBData motor index (== integer value of name).
  static int motorIndexForJoint(const std::string& joint_name);
  // URDF numeric joint name -> (leg, joint-in-leg) for LimbCmd packing.
  static bool legJointForName(const std::string& joint_name, int& leg, int& j);
  // Torque constant: knees ("1","3","5","7") = 1.092, others = 0.546.
  static double ktForJoint(const std::string& joint_name);

  void sendSafeCommand();  // zero-gain LimbCmd, no torque

  // ros2_control interface storage (per URDF joint, in info_ order).
  std::vector<double> pos_state_, vel_state_, eff_state_;
  std::vector<double> pos_cmd_, vel_cmd_, kp_cmd_, kd_cmd_, eff_cmd_;
  std::vector<int> joint_motor_index_;
  std::vector<int> joint_leg_, joint_in_leg_;
  std::vector<double> joint_kt_;

  // IMU sensor.
  std::array<double, 4> imu_orientation_{{0, 0, 0, 1}};
  std::array<double, 3> imu_angular_velocity_{{0, 0, 0}};
  std::array<double, 3> imu_linear_acceleration_{{0, 0, 0}};
  std::string imu_sensor_name_{"imu"};

  bool read_only_{false};

  // MBLink.
  gr::MBLink mblink_;
  bool mblink_started_{false};
};

}  // namespace quad_hardware

#endif  // QUAD_HARDWARE__SPIRIT_SYSTEM_HPP_
