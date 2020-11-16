#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh) {
	nh_ = nh;

  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>("/spirit/joint_controller/command",1);
}

void OpenLoopController::spin() {
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

void OpenLoopController::sendJointPositions(double &elapsed_time)
{
	spirit_msgs::LegCommandArray msg;
	msg.leg_commands.resize(4);
  double abd_angle = 0.0;
  double hip_angle = 0.7;
  double knee_angle = 1.0;
	for (int i = 0; i < 4; ++i)
	{
    msg.leg_commands[i].motor_commands.resize(3);

		msg.leg_commands[i].motor_commands[0].pos_setpoint = abd_angle;
    msg.leg_commands[i].motor_commands[0].kp = 200;
    msg.leg_commands[i].motor_commands[0].kd = 10;
    msg.leg_commands[i].motor_commands[0].vel_setpoint = 0;
    msg.leg_commands[i].motor_commands[0].torque_ff = 0;

    msg.leg_commands[i].motor_commands[1].pos_setpoint = hip_angle;
    msg.leg_commands[i].motor_commands[1].kp = 200;
    msg.leg_commands[i].motor_commands[1].kd = 10;
    msg.leg_commands[i].motor_commands[1].vel_setpoint = 0;
    msg.leg_commands[i].motor_commands[1].torque_ff = 0;

    msg.leg_commands[i].motor_commands[2].pos_setpoint = knee_angle;
    msg.leg_commands[i].motor_commands[2].kp = 200;
    msg.leg_commands[i].motor_commands[2].kd = 10;
    msg.leg_commands[i].motor_commands[2].vel_setpoint = 0;
    msg.leg_commands[i].motor_commands[2].torque_ff = 0;
	}

	msg.header.stamp = ros::Time::now();
	joint_control_pub_.publish(msg);
}