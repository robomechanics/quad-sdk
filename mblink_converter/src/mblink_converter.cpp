#include "mblink_converter/mblink_converter.h"

MBLinkConverter::MBLinkConverter(ros::NodeHandle nh, int argc, char** argv)
{
  nh_ = nh;

    /// Ghost MBLink interface class
  mblink_.start(argc,argv);
  mblink_.rxstart();
  mblink_.setRetry("UPST_ADDRESS", 5);

  // Load rosparams from parameter server
  std::string leg_control_topic;
  nh.param<std::string>("topics/joint_command", leg_control_topic, "/motor_control");
  nh.param<double>("mblink_converter/update_rate", update_rate_, 1000);

  // Setup pubs and subs
  leg_control_sub_ = nh_.subscribe(leg_control_topic,1,&MBLinkConverter::legControlCallback, this);
}

void MBLinkConverter::legControlCallback(const spirit_msgs::LegCommandArray::ConstPtr& msg)
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
    spirit_msgs::LegCommand leg_command = last_leg_command_array_msg_->leg_commands.at(i);
    for (int j = 0; j < 3; ++j) // For each joint
    {
      limbcmd[i].pos[j] = leg_command.motor_commands.at(j).pos_setpoint;
      limbcmd[i].vel[j] = leg_command.motor_commands.at(j).vel_setpoint;
      limbcmd[i].tau[j] = leg_command.motor_commands.at(j).torque_ff;
      limbcmd[i].kp[j] = static_cast<short>(leg_command.motor_commands.at(j).kp);
      limbcmd[i].kd[j] = leg_command.motor_commands.at(j).kd;
    }
  }
  
  float data[58] = {0};
  memcpy(data,limbcmd,4*sizeof(LimbCmd_t));
  mblink_.sendUser(Eigen::Map<const Eigen::Matrix<float,58,1> >(data));

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