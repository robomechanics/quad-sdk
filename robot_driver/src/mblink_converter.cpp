#include "robot_driver/mblink_converter.h"

MBLinkConverter::MBLinkConverter(ros::NodeHandle nh, int argc, char** argv)
{
  nh_ = nh;

  /// Ghost MBLink interface class
  mblink_.start(argc,argv);
  mblink_.rxstart();
  mblink_.setRetry("_UPST_ADDRESS", 255);
  mblink_.setRetry("UPST_LOOP_DELAY", 1);

  // Load rosparams from parameter server
  std::string leg_control_topic, joint_encoder_topic, imu_topic, remote_heartbeat_topic, control_restart_flag_topic;
  quad_utils::loadROSParam(nh_,"topics/control/joint_command",leg_control_topic);
  quad_utils::loadROSParam(nh_,"topics/control/restart_flag",control_restart_flag_topic);
  quad_utils::loadROSParam(nh_,"topics/heartbeat/remote",remote_heartbeat_topic);
  quad_utils::loadROSParam(nh_,"topics/joint_encoder",joint_encoder_topic);
  quad_utils::loadROSParam(nh_,"topics/imu",imu_topic);

  // Setup pubs and subs
  control_restart_flag_sub_ = nh_.subscribe(control_restart_flag_topic,1,&MBLinkConverter::controlRestartFlagCallback, this);
  remote_heartbeat_sub_ = nh_.subscribe(remote_heartbeat_topic,1,&MBLinkConverter::remoteHeartbeatCallback, this);
  // joint_encoder_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_encoder_topic,1);
  // imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic,1);

  last_heartbeat_time_ = std::numeric_limits<double>::max();
  last_leg_command_time_ = std::numeric_limits<double>::max();

  restart_flag_ = false;
  
}

// void MBLinkConverter::updateLegCommandArray(
//   const quad_msgs::LegCommandArray::ConstPtr& msg)
// {
//   last_leg_command_array_msg_ = msg;
//   last_leg_command_time_ = msg->header.stamp.toSec();

//   // double t_now = ros::Time::now().toSec();
//   // ROS_INFO("Current time = %6.4f, msg time = %6.4f, diff = %6.4fs", t_now, last_leg_command_time_, t_now - last_leg_command_time_);
// }

void MBLinkConverter::stop() {
  mblink_.rxstop();
}


void MBLinkConverter::remoteHeartbeatCallback(const std_msgs::Header::ConstPtr& msg) {
  last_heartbeat_time_ = msg->stamp.toSec();
}

void MBLinkConverter::controlRestartFlagCallback(const std_msgs::Bool::ConstPtr& msg) {
  restart_flag_ = msg->data;
  restart_flag_time_ = ros::Time::now();
}

bool MBLinkConverter::sendMBlink(const quad_msgs::LegCommandArray& last_leg_command_array_msg)
{
  // // If we've haven't received a motor control message, exit
  // if (last_leg_command_array_msg == NULL)
  // {
  //   ROS_DEBUG_THROTTLE(1,"MBLinkConverter node has not received MotorCommandArray messages yet.");
  //   return false;
  // }

  int leg_command_heartbeat = 1;

  // if ((ros::Time::now().toSec() - last_leg_command_time_) >= leg_command_timeout_)
  // {
  //   leg_command_heartbeat = 0;
  //   ROS_WARN_THROTTLE(1,"Leg command heartbeat lost in MBLinkConverter, sending zeros");
  // }
  
  LimbCmd_t limbcmd[4];
  for (int i = 0; i < 4; ++i) // For each leg
  {
    // std::cout << "leg = " << i << std::endl;
    quad_msgs::LegCommand leg_command = 
      last_leg_command_array_msg.leg_commands.at(i);

    for (int j = 0; j < 3; ++j) // For each joint
    {
      // std::cout << "joint = " << j << std::endl;
      limbcmd[i].pos[j] = leg_command_heartbeat*leg_command.motor_commands.at(j).pos_setpoint;
      limbcmd[i].vel[j] = leg_command_heartbeat*leg_command.motor_commands.at(j).vel_setpoint;
      limbcmd[i].tau[j] = leg_command_heartbeat*leg_command.motor_commands.at(j).torque_ff;
      limbcmd[i].kp[j] = static_cast<short>(leg_command_heartbeat*leg_command.motor_commands.at(j).kp);
      limbcmd[i].kd[j] = static_cast<short>(leg_command_heartbeat*leg_command.motor_commands.at(j).kd);
      limbcmd[i].restart_flag = restart_flag_;

      // std::cout << "Size of limbcmd[i].pos[j] = " << sizeof(limbcmd[i].pos[j]) << std::endl;
      // std::cout << "Size of limbcmd[i].vel[j] = " << sizeof(limbcmd[i].vel[j]) << std::endl;
      // std::cout << "Size of limbcmd[i].tau[j] = " << sizeof(limbcmd[i].tau[j]) << std::endl;
      // std::cout << "Size of limbcmd[i].kp[j] = " << sizeof(limbcmd[i].kp[j]) << std::endl;
      // std::cout << "Size of limbcmd[i].kd[j] = " << sizeof(limbcmd[i].kd[j]) << std::endl;
      // std::cout << "Size of limbcmd[i].restart_flag = " << sizeof(limbcmd[i].restart_flag) << std::endl;
    }
  }
  
  // std::cout << "ready to send" << std::endl;
  // std::cout << "sizeof(LimbCmd_t) = " << sizeof(LimbCmd_t) << std::endl;
  float data[58] = {0};
  memcpy(data,limbcmd,4*sizeof(LimbCmd_t));
  // std::cout << "limbcmd loaded" << std::endl;
  mblink_.sendUser(Eigen::Map<const Eigen::Matrix<float,58,1> >(data));

  return true;
}

void MBLinkConverter::getMBlink(MBData_t &data)
{

  // Get the data and appropriate timestamp (this may be blocking)
  data.clear();
  data = mblink_.get();

  if (data.empty()) {
    ROS_WARN_THROTTLE(0.5, "No data received from mblink");
    return;
  }

  // ros::Time timestamp = ros::Time::now();

  // // Declare the joint state msg and apply the timestamp
  // sensor_msgs::JointState joint_state_msg;
  // joint_state_msg.header.stamp = timestamp;

  // // Add the data corresponding to each joint
  // for (int i = 0; i < joint_names_.size(); i++)
  // {
  //   joint_state_msg.name.push_back(joint_names_[i]);
  //   joint_state_msg.position.push_back(data["joint_position"][joint_indices_[i]]);
  //   joint_state_msg.velocity.push_back(data["joint_velocity"][joint_indices_[i]]);

  //   // Convert from current to torque
  //   joint_state_msg.effort.push_back(kt_vec_[i]*data["joint_current"][joint_indices_[i]]);
  // }

  // // Publish the joint state message
  // // joint_encoder_pub_.publish(joint_state_msg);

  // // Declare the imu message
  // sensor_msgs::Imu imu_msg;
  // imu_msg.header.stamp = timestamp;

  // // Transform from rpy to quaternion
  // geometry_msgs::Quaternion orientation_msg;
  // tf2::Quaternion quat_tf;
  // quat_tf.setRPY(data["imu_euler"][0],data["imu_euler"][1],data["imu_euler"][2]);
  // tf2::convert(quat_tf, orientation_msg);

  // // Load the data into the imu message
  // imu_msg.orientation = orientation_msg;
  // imu_msg.angular_velocity.x = data["imu_angular_velocity"][0];
  // imu_msg.angular_velocity.y = data["imu_angular_velocity"][1];
  // imu_msg.angular_velocity.z = data["imu_angular_velocity"][2];
  // imu_msg.linear_acceleration.x = data["imu_linear_acceleration"][0];
  // imu_msg.linear_acceleration.y = data["imu_linear_acceleration"][1];
  // imu_msg.linear_acceleration.z = data["imu_linear_acceleration"][2];

  // Publish the imu message
  // imu_pub_.publish(imu_msg);
}

// void MBLinkConverter::spin()
// {
//   ros::Rate r(update_rate_);
//   while (ros::ok()) {

//     // Collect new messages on subscriber topics
//     ros::spinOnce(); 

//     // Unset the restart flag after a timeout duration
//     if ((ros::Time::now() - restart_flag_time_).toSec() >= 0.1) {
//       restart_flag_ = false;
//     }

//     // Send out the most recent one over mblink
//     this->sendMBlink();
//     this->getMBlink();

//     // Enforce update rate
//     r.sleep();
//   }
// }