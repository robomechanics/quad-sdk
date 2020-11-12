#include "mblink_converter/mblink_converter.h"

MbLinkConverter::MbLinkConverter(ros::NodeHandle nh)
{
  nh_ = nh;

  // Load rosparams from parameter server
  std::string motor_control_topic;
  nh.param<std::string>("topics/motor_control", motor_control_topic, "/motor_control");
  nh.param<double>("mblink_converter/update_rate", update_rate_, 1000);

  // Setup pubs and subs
  motor_control_sub_ = nh_.subscribe(motor_control_topic,1,&MbLinkConverter::motorControlCallback, this);
}

void MbLinkConverter::motorControlCallback(const spirit_msgs::MotorCommandArray::ConstPtr& msg)
{
  last_motor_command_array_msg_ = msg;
}

bool MbLinkConverter::sendMBlink()
{
  // If we've haven't received a motor control message, exit
  if (last_motor_command_array_msg != NULL)
  {
    return false;
  }


  return true;
}

void MbLinkConverter::spin()
{
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Send out the most recent one over mblink
    if (!this->sendMBlink())
    {
      ROS_DEBUG_THROTTLE(1,"MBLinkConverter node has not received MotorCommandArray messages yet.")
    }

    // Enforce update rate
    r.sleep();
  }
}