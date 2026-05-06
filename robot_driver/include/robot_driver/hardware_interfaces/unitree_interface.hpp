#ifndef UNITREE_INTERFACE_H
#define UNITREE_INTERFACE_H

#include <robot_driver/hardware_interfaces/hardware_interface.hpp>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <array>
#include <mutex>
#include <string>
#include <vector>

//! Unified hardware interface for Unitree quadrupeds (Go2 and Go2-W).
/*!
   UnitreeInterface converts quad-sdk LegCommandArray messages to
   Unitree SDK2 LowCmd messages (rt/lowcmd) and reads LowState
   (rt/lowstate) for joint state and IMU feedback.

   The DDS contract is identical for Go2 (12 motors) and Go2-W
   (16 motors, with wheels at LowCmd indices 12-15). Pick the variant
   by passing 12 or 16 to the constructor.

   On Go2-W, wheel commands are sourced in this priority order:
     1. A 4th MotorCommand on each leg's motor_commands[] (vel_setpoint
        is forwarded as wheel velocity; kp / kd / torque_ff pass through).
        This is the natural path once controllers emit wheel commands.
     2. user_tx_data laid out as
          [restart_flag,
           vel_0, kd_0, tau_ff_0, ...,
           vel_3, kd_3, tau_ff_3]   // size 13
        (preserved from the legacy Go2WInterface convention).
     3. Zero velocity with kDefaultWheelKd damping.
*/
class UnitreeInterface : public HardwareInterface {
 public:
  static constexpr int kNumLegs = 4;
  static constexpr int kJointsPerLeg = 3;
  static constexpr int kNumJoints = kNumLegs * kJointsPerLeg;
  static constexpr int kNumWheels = 4;

  //! Construct from a quad-sdk robot_name string.
  //! Recognized values: "go2" (12 motors), "go2w" (16 motors).
  //! Anything else falls back to "go2".
  explicit UnitreeInterface(const std::string& robot_name = "go2");

  void loadInterface(int argc, char** argv) override;
  void unloadInterface() override;

  bool send(const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
            const Eigen::VectorXd& user_tx_data) override;

  bool recv(sensor_msgs::msg::JointState& joint_state_msg,
            sensor_msgs::msg::Imu& imu_msg,
            Eigen::VectorXd& user_rx_data) override;

  //! Latest raw foot-force readings from rt/lowstate.
  //! Returned in quad-sdk leg order (FL, RL, FR, RR), int16 units.
  //! Used by robot_driver to publish a custom FootContact message.
  std::array<int16_t, kNumLegs> getFootForcesRaw() const;

 protected:
  void initLowCmd();
  void lowStateHandler(const void* message);
  static uint32_t crc32Core(uint32_t* ptr, uint32_t len);

  std::string robot_name_;
  int num_motors_;
  bool has_wheels_;

  unitree_go::msg::dds_::LowCmd_ low_cmd_{};
  unitree_go::msg::dds_::LowState_ low_state_{};
  std::array<int16_t, kNumLegs> foot_force_quad_order_{};  // FL, RL, FR, RR
  mutable std::mutex state_mutex_;

  unitree::robot::ChannelPublisherPtr<
      unitree_go::msg::dds_::LowCmd_> cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<
      unitree_go::msg::dds_::LowState_> state_sub_;

  // Legacy user_tx_data layout for the wheel-command fallback.
  static constexpr int kWheelCmdFields = 3;       // (vel, kd, tau_ff)
  static constexpr int kTxRestartFlagOffset = 1;  // user_tx_data[0]=restart
  static constexpr int kWheelTxSize =
      kTxRestartFlagOffset + kNumWheels * kWheelCmdFields;  // 13

  // user_rx_data mirror layout (per-wheel state).
  static constexpr int kWheelStateFields = 3;     // (pos, vel, tau_est)
  static constexpr int kWheelRxSize = kNumWheels * kWheelStateFields;  // 12

  static constexpr float kDefaultWheelKd = 5.0f;

  // quad-sdk leg/joint -> Unitree LowCmd motor index.
  // quad-sdk leg order: leg_0=FL, leg_1=RL, leg_2=FR, leg_3=RR.
  // Unitree leg order:  FR=0-2, FL=3-5, RR=6-8, RL=9-11;
  //                     wheels FR=12, FL=13, RR=14, RL=15.
  static constexpr int kLegMap[kNumLegs][kJointsPerLeg] = {
      {3, 4, 5},    // FL
      {9, 10, 11},  // RL
      {0, 1, 2},    // FR
      {6, 7, 8}     // RR
  };
  static constexpr int kWheelMap[kNumWheels] = {13, 15, 12, 14};

  // quad-sdk leg index -> Unitree foot_force array index.
  // Unitree foot_force order: [FR=0, FL=1, RR=2, RL=3].
  // quad-sdk leg order:       [FL=0, RL=1, FR=2, RR=3].
  static constexpr int kFootForceMap[kNumLegs] = {1, 3, 0, 2};

  // URDF joint names (numeric, matching go2.urdf.xacro / go2w.urdf.xacro).
  std::vector<std::string> joint_names_ = {
      "8",  "0",  "1",
      "9",  "2",  "3",
      "10", "4",  "5",
      "11", "6",  "7"};
  std::vector<std::string> wheel_joint_names_ = {
      "jtoe0", "jtoe1", "jtoe2", "jtoe3"};

  bool state_received_ = false;
};

#endif  // UNITREE_INTERFACE_H
