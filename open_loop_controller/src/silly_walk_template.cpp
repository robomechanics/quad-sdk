#include "open_loop_controller/silly_walk_template.h"

SillyWalkController::SillyWalkController(ros::NodeHandle nh) {
	
  // Assign the node handle to the class
  nh_ = nh;

  // Get rosparams from the server
  std::string joint_command_topic,control_mode_topic;
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",joint_command_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"silly_walk_template/update_rate",update_rate_);
  
  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(joint_command_topic,1);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&SillyWalkController::controlModeCallback, this);

  // Add any other class initialization goes here
  control_mode_ = SIT;
}

void SillyWalkController::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  // Use this to set any logic for control modes
  if (msg->data == SIT || (msg->data == STAND))
  {  
    control_mode_ = msg->data;
  }
}

void SillyWalkController::computeJointControl()
{
  // Put your control code here
  control_msg_.leg_commands.clear();
  control_msg_.leg_commands.resize(num_legs_);
}

void SillyWalkController::publishJointControl()
{
  control_msg_.header.stamp = ros::Time::now();
	joint_control_pub_.publish(control_msg_);
}

void SillyWalkController::sendJointPositions()
{
	
  // Control state machine can go here if you want one
  switch (control_mode_){
    case SIT: // option 1
    {

    }
    break;
    
    case STAND: // option 2
    {
      
    }
    break;
  }

	msg.header.stamp = ros::Time::now();
	joint_control_pub_.publish(msg);
}

void SillyWalkController::spin() {

  // Set update rate
  ros::Rate r(update_rate_);

  while (ros::ok()) {
    this->computeJointControl(elapsed_time);
    this->sendJointPositions(elapsed_time);

    ros::spinOnce();
    r.sleep();
  }
}