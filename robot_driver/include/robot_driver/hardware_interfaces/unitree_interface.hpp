#ifndef UNITREE_INTERFACE_H
#define UNITREE_INTERFACE_H

#include <robot_driver/hardware_interfaces/hardware_interface.hpp>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include <mutex>

//! Hardware interface for Unitree Go2 quadruped.
/*!
   UnitreeInterface converts quad-sdk LegCommandArray messages to
   Unitree SDK2 LowCmd messages and publishes them over DDS.
   It subscribes to LowState for joint state and IMU feedback.
*/
class UnitreeInterface : public HardwareInterface {
 public:
  UnitreeInterface();

  void loadInterface(int argc, char** argv) override;
  void unloadInterface() override;

  bool send(
      const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      const Eigen::VectorXd& user_tx_data) override;

  bool recv(sensor_msgs::msg::JointState& joint_state_msg,
            sensor_msgs::msg::Imu& imu_msg,
            Eigen::VectorXd& user_rx_data) override;

 private:
  void initLowCmd();
  void lowStateHandler(const void* message);

  static uint32_t crc32Core(uint32_t* ptr, uint32_t len);

  unitree_go::msg::dds_::LowCmd_ low_cmd_{};
  unitree_go::msg::dds_::LowState_ low_state_{};
  std::mutex state_mutex_;

  unitree::robot::ChannelPublisherPtr<
      unitree_go::msg::dds_::LowCmd_> cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<
      unitree_go::msg::dds_::LowState_> state_sub_;

  // Mapping from quad-sdk leg/joint index to Unitree motor index.
  // quad-sdk: leg_0=FL, leg_1=RL, leg_2=FR, leg_3=RR
  // Unitree:  FR=0-2, FL=3-5, RR=6-8, RL=9-11
  // Each leg has 3 joints: [abad, hip, knee]
  static constexpr int kNumLegs = 4;
  static constexpr int kJointsPerLeg = 3;
  static constexpr int kNumJoints = kNumLegs * kJointsPerLeg;
  static constexpr int kLegMap[kNumLegs][kJointsPerLeg] = {
      {3, 4, 5},    // leg_0 (FL) -> Unitree FL
      {9, 10, 11},  // leg_1 (RL) -> Unitree RL
      {0, 1, 2},    // leg_2 (FR) -> Unitree FR
      {6, 7, 8}     // leg_3 (RR) -> Unitree RR
  };

  // Joint names matching go2.yaml ordering
  std::vector<std::string> joint_names_ = {
      "FL_hip_joint",   "FL_thigh_joint",  "FL_calf_joint",
      "RL_hip_joint",   "RL_thigh_joint",  "RL_calf_joint",
      "FR_hip_joint",   "FR_thigh_joint",  "FR_calf_joint",
      "RR_hip_joint",   "RR_thigh_joint",  "RR_calf_joint"};

  bool state_received_ = false;
};

#endif  // UNITREE_INTERFACE_H
