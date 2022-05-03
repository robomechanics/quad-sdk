#include "robot_driver/hardware_interfaces/spirit_interface.h"

SpiritInterface::SpiritInterface() {}

void SpiritInterface::loadInterface(int argc, char** argv) {
  /// Ghost MBLink interface class
  mblink_.start(argc, argv);
  mblink_.rxstart();
  mblink_.setRetry("_UPST_ADDRESS", 255);
  mblink_.setRetry("UPST_LOOP_DELAY", 1);
}

void SpiritInterface::unloadInterface() { mblink_.rxstop(); }

bool SpiritInterface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data) {
  int leg_command_heartbeat = 1;

  bool restart_flag = (user_tx_data[0] == 1);

  LimbCmd_t limbcmd[4];
  for (int i = 0; i < 4; ++i) {  // For each leg
    // std::cout << "leg = " << i << std::endl;
    quad_msgs::LegCommand leg_command =
        last_leg_command_array_msg.leg_commands.at(i);

    for (int j = 0; j < 3; ++j) {  // For each joint
      // std::cout << "joint = " << j << std::endl;
      limbcmd[i].pos[j] =
          leg_command_heartbeat * leg_command.motor_commands.at(j).pos_setpoint;
      limbcmd[i].vel[j] =
          leg_command_heartbeat * leg_command.motor_commands.at(j).vel_setpoint;
      limbcmd[i].tau[j] =
          leg_command_heartbeat * leg_command.motor_commands.at(j).torque_ff;
      limbcmd[i].kp[j] = static_cast<short>(
          leg_command_heartbeat * leg_command.motor_commands.at(j).kp);
      limbcmd[i].kd[j] = static_cast<short>(
          leg_command_heartbeat * leg_command.motor_commands.at(j).kd);
      limbcmd[i].restart_flag = restart_flag;
    }
  }

  float data[58] = {0};
  memcpy(data, limbcmd, 4 * sizeof(LimbCmd_t));
  mblink_.sendUser(Eigen::Map<const Eigen::Matrix<float, 58, 1> >(data));

  return true;
}

bool SpiritInterface::recv(sensor_msgs::JointState& joint_state_msg,
                           sensor_msgs::Imu& imu_msg,
                           Eigen::VectorXd& user_rx_data) {
  // Get the data and appropriate timestamp (this may be blocking)
  MBData_t mbdata = mblink_.get();

  // Check if data exists
  if (mbdata.empty()) {
    return false;
  }

  // Add the data corresponding to each joint
  for (int i = 0; i < joint_names_.size(); i++) {
    joint_state_msg.name[i] = joint_names_[i];
    joint_state_msg.position[i] = mbdata["joint_position"][joint_indices_[i]];
    joint_state_msg.velocity[i] = mbdata["joint_velocity"][joint_indices_[i]];

    // Convert from current to torque using linear motor model
    joint_state_msg.effort[i] =
        kt_vec_[i] * mbdata["joint_current"][joint_indices_[i]];
  }

  // Transform from rpy to quaternion
  geometry_msgs::Quaternion orientation_msg;
  tf2::Quaternion quat_tf;
  Eigen::Vector3f rpy;
  quat_tf.setRPY(mbdata["imu_euler"][0], mbdata["imu_euler"][1],
                 mbdata["imu_euler"][2]);
  tf2::convert(quat_tf, orientation_msg);

  // Load the data into the imu message
  imu_msg.orientation = orientation_msg;
  imu_msg.angular_velocity.x = mbdata["imu_angular_velocity"][0];
  imu_msg.angular_velocity.y = mbdata["imu_angular_velocity"][1];
  imu_msg.angular_velocity.z = mbdata["imu_angular_velocity"][2];

  // I guess the acceleration is opposite
  imu_msg.linear_acceleration.x = -mbdata["imu_linear_acceleration"][0];
  imu_msg.linear_acceleration.y = -mbdata["imu_linear_acceleration"][1];
  imu_msg.linear_acceleration.z = -mbdata["imu_linear_acceleration"][2];

  return true;
}
