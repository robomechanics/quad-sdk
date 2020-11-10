#include "clark_trot_controller/clark_trot_controller.h"

ClarkTrotController::ClarkTrotController(ros::NodeHandle nh) {
	nh_ = nh;

  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<spirit_msgs::MotorCommandArray>("/spirit/joint_controller/command",1);
}

void ClarkTrotController::spin() {
	double start_time = ros::Time::now().toSec();
	update_rate_ = 10;	
	ros::Rate r(update_rate_);
	while (ros::ok()) {
  	double elapsed_time = ros::Time::now().toSec() - start_time;
  	this->sendJointPositions(elapsed_time);
		ros::spinOnce();
		r.sleep();
	}
}

void ClarkTrotController::sendJointPositions(double &elapsed_time)
{
	spirit_msgs::MotorCommandArray msg;
	msg.motor_commands.resize(12);
	std::vector<double> positions = {0.7,1,0.7,1,0.7,1,0.7,1,0,0,0,0};
	for (int i = 0; i < 12; ++i)
	{
		msg.motor_commands.at(i).kp = 100;
		msg.motor_commands.at(i).kd = 1;
		msg.motor_commands.at(i).torque_ff = 0;
		msg.motor_commands.at(i).pos_setpoint = positions.at(i);
    msg.motor_commands.at(i).vel_setpoint = 0;
	}

	msg.header.stamp = ros::Time::now();
	joint_control_pub_.publish(msg);
}