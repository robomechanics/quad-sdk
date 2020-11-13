#include "mblink_converter/mblink_converter.h"

MBLinkConverter::MBLinkConverter(ros::NodeHandle nh, std::shared_ptr<MBLink> mblink)
{
  nh_ = nh;
  mblink_ = mblink;

  // Load rosparams from parameter server
  std::string motor_control_topic;
  nh.param<std::string>("topics/motor_control", motor_control_topic, "/motor_control");
  nh.param<double>("mblink_converter/update_rate", update_rate_, 1000);

  // Setup pubs and subs
  motor_control_sub_ = nh_.subscribe(motor_control_topic,1,&MBLinkConverter::motorControlCallback, this);
}

void MBLinkConverter::motorControlCallback(const spirit_msgs::MotorCommandArray::ConstPtr& msg)
{
  last_motor_command_array_msg_ = msg;
}

bool MBLinkConverter::sendMBlink()
{
  // If we've haven't received a motor control message, exit
  if (last_motor_command_array_msg_ != NULL)
  {
    return false;
  }

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