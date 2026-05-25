#ifndef QUAD_HARDWARE__UNITREE_SYSTEM_HPP_
#define QUAD_HARDWARE__UNITREE_SYSTEM_HPP_

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

namespace quad_hardware {

//! ros2_control SystemInterface for Unitree Go2 / Go2-W quadrupeds.
/*!
   Wraps the Unitree SDK2 DDS contract (rt/lowcmd, rt/lowstate) behind the
   ros2_control hardware_interface API so the controller_manager loop drives
   the robot the same way it drives Gazebo.

   This is the hardware counterpart to the legacy robot_driver
   UnitreeInterface (send()/recv()); the DDS field mapping is intentionally
   identical so behavior is preserved.

   Because Go2 motors run an ONBOARD PD loop
   (tau_motor = kp*(q - q_meas) + kd*(dq - dq_meas) + tau), each joint exposes
   FIVE command interfaces — position, velocity, kp, kd, effort — not just
   effort. A controller that writes all five reproduces the legacy
   LegCommandArray->send() behavior exactly.

   Safety: when command interfaces are unclaimed (NaN), or when the hardware
   parameter read_only=true, write() publishes a zero-gain LowCmd so no torque
   is applied. This makes a broadcaster-only (read-only) bring-up safe.
*/
class UnitreeSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeSystem)

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
  // Map this hardware's URDF joint name -> Unitree LowCmd/LowState motor index.
  // Mirrors UnitreeInterface::kLegMap with the numeric joint names from
  // go2.urdf.xacro. Returns -1 if the name is not a known leg joint.
  static int motorIndexForJoint(const std::string& joint_name);

  void initLowCmd();
  void lowStateHandler(const void* message);
  void writeSafeCommand();  // zero-gain LowCmd, no torque
  static uint32_t crc32Core(uint32_t* ptr, uint32_t len);

  // --- ros2_control interface storage (one slot per URDF joint, in info_ order)
  std::vector<double> pos_state_;
  std::vector<double> vel_state_;
  std::vector<double> eff_state_;

  std::vector<double> pos_cmd_;
  std::vector<double> vel_cmd_;
  std::vector<double> kp_cmd_;
  std::vector<double> kd_cmd_;
  std::vector<double> eff_cmd_;

  // Resolved Unitree motor index for each URDF joint slot.
  std::vector<int> joint_motor_index_;

  // --- IMU sensor (orientation xyzw, angular velocity xyz, linear accel xyz)
  std::array<double, 4> imu_orientation_{{0, 0, 0, 1}};
  std::array<double, 3> imu_angular_velocity_{{0, 0, 0}};
  std::array<double, 3> imu_linear_acceleration_{{0, 0, 0}};
  std::string imu_sensor_name_{"imu"};

  // --- Configuration (from URDF <hardware><param>) ---
  std::string network_interface_{"eth0"};
  bool read_only_{false};

  // --- Unitree DDS ---
  unitree_go::msg::dds_::LowCmd_ low_cmd_{};
  unitree_go::msg::dds_::LowState_ low_state_{};
  mutable std::mutex state_mutex_;
  bool state_received_{false};

  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      state_sub_;
};

}  // namespace quad_hardware

#endif  // QUAD_HARDWARE__UNITREE_SYSTEM_HPP_
