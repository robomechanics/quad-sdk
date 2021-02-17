#include "mblink_converter/mblink_converter.h"

MBLinkConverter::MBLinkConverter(ros::NodeHandle nh, int argc, char** argv)
{
  nh_ = nh;

    /// Ghost MBLink interface class
  mblink_.start(argc,argv);
  mblink_.rxstart();
  mblink_.setRetry("UPST_ADDRESS", 5);

  // Load rosparams from parameter server
  std::string leg_control_topic, joint_encoder_topic, imu_topic;
  spirit_utils::loadROSParam(nh_,"topics/joint_command",leg_control_topic);
  spirit_utils::loadROSParam(nh_,"topics/joint_encoder",joint_encoder_topic);
  spirit_utils::loadROSParam(nh_,"topics/imu",imu_topic);
  spirit_utils::loadROSParam(nh_,"mblink_converter/update_rate",update_rate_);

  // Setup pubs and subs
  leg_control_sub_ = nh_.subscribe(leg_control_topic,1,&MBLinkConverter::legControlCallback, this);
  joint_encoder_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_encoder_topic,1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic,1);
}

void MBLinkConverter::legControlCallback(
  const spirit_msgs::LegCommandArray::ConstPtr& msg)
{
  last_leg_command_array_msg_ = msg;
}

bool MBLinkConverter::sendMBlink()
{
  // If we've haven't received a motor control message, exit
  if (last_leg_command_array_msg_ == NULL)
  {
    return false;
  }
  
  LimbCmd_t limbcmd[4];
  for (int i = 0; i < 4; ++i) // For each leg
  {
    spirit_msgs::LegCommand leg_command = 
      last_leg_command_array_msg_->leg_commands.at(i);

    for (int j = 0; j < 3; ++j) // For each joint
    {
      limbcmd[i].pos[j] = leg_command.motor_commands.at(j).pos_setpoint;
      limbcmd[i].vel[j] = leg_command.motor_commands.at(j).vel_setpoint;
      limbcmd[i].tau[j] = leg_command.motor_commands.at(j).torque_ff;
    }

    limbcmd[i].kp = leg_command.motor_commands.at(0).kp;
    limbcmd[i].kd = leg_command.motor_commands.at(0).kd;
  }
  
  float data[58] = {0};
  memcpy(data,limbcmd,4*sizeof(LimbCmd_t));
  mblink_.sendUser(Eigen::Map<const Eigen::Matrix<float,58,1> >(data));

  return true;
}

bool MBLinkConverter::publishMBlink()
{

  std::vector<double> joint_positions(12);
  std::vector<double> joint_velocities(12);
  std::vector<double> joint_effort(12);
  std::vector<std::string> joint_name = {"8","0","1","9","2","3","10","4","5","11","6","7"};

  RxData_t data = mblink_.get();
  ros::Time timestamp = ros::Time::now();

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = timestamp;

  for (int i = 0; i < joint_positions.size(); i++)
  {
    joint_state_msg.name.push_back(joint_name[i]);
    joint_state_msg.position.push_back(data["joint_position"][joint_indices_[i]]);
    joint_state_msg.velocity.push_back(data["joint_velocity"][joint_indices_[i]]);
    joint_state_msg.effort.push_back(data["joint_current"][joint_indices_[i]]);
  }

  joint_encoder_pub_.publish(joint_state_msg);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = timestamp;
  geometry_msgs::Quaternion orientation_msg;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(data["imu_euler"][0],data["imu_euler"][1],data["imu_euler"][2]);
  tf2::convert(quat_tf, orientation_msg);

  imu_msg.orientation = orientation_msg;
  imu_msg.angular_velocity.x = data["imu_angular_velocity"][0];
  imu_msg.angular_velocity.y = data["imu_angular_velocity"][1];
  imu_msg.angular_velocity.z = data["imu_angular_velocity"][2];
  imu_msg.linear_acceleration.x = data["imu_linear_acceleration"][0];
  imu_msg.linear_acceleration.y = data["imu_linear_acceleration"][1];
  imu_msg.linear_acceleration.z = data["imu_linear_acceleration"][2];

  imu_pub_.publish(imu_msg);

  return true;
}

void MBLinkConverter::spin()
{
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Send out the most recent one over mblink
    if (!this->sendMBlink())
    {
      ROS_DEBUG_THROTTLE(1,"MBLinkConverter node has not received MotorCommandArray messages yet.");
    }

    // Enforce update rate
    r.sleep();
  }
}