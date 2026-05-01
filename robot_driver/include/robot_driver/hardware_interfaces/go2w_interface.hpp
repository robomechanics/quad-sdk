#ifndef GO2W_INTERFACE_H
#define GO2W_INTERFACE_H

#include <robot_driver/hardware_interfaces/go2_interface.hpp>

//! Hardware interface for the Unitree Go2-W wheeled quadruped.
/*!
   Go2WInterface extends Go2Interface to additionally drive the four
   wheel motors at LowCmd indices 12-15. Wheels are velocity-controlled
   per Unitree's go2w_stand_example: kp=0, kd>0, dq=target_wheel_velocity,
   q is held at zero, tau_ff is forwarded.

   Wheel commands enter via the inherited send() user_tx_data Eigen vector.
   Layout (index [0] is reserved for the existing control_restart_flag
   convention shared with SpiritInterface; wheel commands occupy indices
   [1..12] in quad-sdk leg order FL, RL, FR, RR):
       user_tx_data = [restart_flag,
                       vel_0, kd_0, tau_ff_0,
                       vel_1, kd_1, tau_ff_1,
                       vel_2, kd_2, tau_ff_2,
                       vel_3, kd_3, tau_ff_3]   // size 13
   If user_tx_data has the wrong size, wheels are commanded to zero
   velocity with default damping.

   Wheel state exits via recv() user_rx_data Eigen vector and is also
   appended to joint_state_msg (which the caller must size to
   kNumJoints + kNumWheels = 16).
       user_rx_data = [pos_0..pos_3, vel_0..vel_3, tau_0..tau_3]   // size 12
*/
class Go2WInterface : public Go2Interface {
 public:
  Go2WInterface();

  bool send(
      const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      const Eigen::VectorXd& user_tx_data) override;

  bool recv(sensor_msgs::msg::JointState& joint_state_msg,
            sensor_msgs::msg::Imu& imu_msg,
            Eigen::VectorXd& user_rx_data) override;

 protected:
  // Per-leg layout in user_tx_data: 3 fields (vel, kd, tau_ff)
  static constexpr int kWheelCmdFields = 3;
  // Per-wheel layout in user_rx_data: 3 fields (pos, vel, tau_est)
  static constexpr int kWheelStateFields = 3;
  static constexpr int kNumWheels = 4;
  // user_tx_data layout: [restart_flag, vel_0, kd_0, tau_ff_0, ...]
  static constexpr int kTxRestartFlagOffset = 1;
  static constexpr int kWheelCmdSize =
      kTxRestartFlagOffset + kNumWheels * kWheelCmdFields;
  static constexpr int kWheelStateSize = kNumWheels * kWheelStateFields;

  // Default kd applied if user_tx_data is not provided (matches the value
  // used in Unitree's go2w_stand_example).
  static constexpr float kDefaultWheelKd = 5.0f;

  // Mapping from quad-sdk leg index to Unitree wheel motor index.
  // quad-sdk: leg_0=FL, leg_1=RL, leg_2=FR, leg_3=RR
  // Unitree wheels follow the same per-leg ordering as the leg motors,
  // with a +12 offset: FR=12, FL=13, RR=14, RL=15
  static constexpr int kWheelMap[kNumWheels] = {
      13,  // leg_0 (FL) -> Unitree FL wheel
      15,  // leg_1 (RL) -> Unitree RL wheel
      12,  // leg_2 (FR) -> Unitree FR wheel
      14   // leg_3 (RR) -> Unitree RR wheel
  };

  // URDF wheel joint names (continuous joints in go2w.urdf.xacro).
  // Ordered per quad-sdk leg index: FL, RL, FR, RR.
  std::vector<std::string> wheel_joint_names_ = {
      "jtoe0",  // leg_0 (FL)
      "jtoe1",  // leg_1 (RL)
      "jtoe2",  // leg_2 (FR)
      "jtoe3"   // leg_3 (RR)
  };
};

#endif  // GO2W_INTERFACE_H
