#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh) {
	nh_ = nh;

  // Get rosparams
  std::string leg_control_topic;
  nh.getParam("topics/joint_command", leg_control_topic);
  nh.getParam("open_loop_controller/t_cycle", t_cycle_);
  nh.getParam("open_loop_controller/control_mode", mode_);

  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>("/spirit/joint_controller/command",1);
}

std::pair<double,double> OpenLoopController::computeIk(double x, double y)
{
  double l1 = 0.2075;
  double l2 = 0.2075;
  double q2 = acos((x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2));
  double q1 = atan2(y,x) - atan2(l2*sin(q2),l1+l2*cos(q2));

  double theta_hip = q1 - M_PI;
  double theta_knee = M_PI - M_PI;

  if (theta_hip < -M_PI) theta_hip += 2*M_PI;
  if (theta_hip > M_PI) theta_hip -= 2*M_PI;
  if (theta_knee < -M_PI) theta_knee += 2*M_PI;
  if (theta_knee > M_PI) theta_knee -= 2*M_PI;
  return std::make_pair(theta_hip,theta_knee);
}

void OpenLoopController::spin() {
	double start_time = ros::Time::now().toSec();
	update_rate_ = 1000;	
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

  switch (mode_){
    case 0: // stand
    {
      double abd_angle = 0.0;
      double hip_angle = 0.7;
      double knee_angle = 1.0;
      for (int i = 0; i < 4; ++i)
      {
        msg.leg_commands[i].motor_commands.resize(3);

        msg.leg_commands[i].motor_commands[0].pos_setpoint = abd_angle;
        msg.leg_commands[i].motor_commands[0].kp = 100;
        msg.leg_commands[i].motor_commands[0].kd = 3;
        msg.leg_commands[i].motor_commands[0].vel_setpoint = 0;
        msg.leg_commands[i].motor_commands[0].torque_ff = 0;

        msg.leg_commands[i].motor_commands[1].pos_setpoint = hip_angle;
        msg.leg_commands[i].motor_commands[1].kp = 100;
        msg.leg_commands[i].motor_commands[1].kd = 3;
        msg.leg_commands[i].motor_commands[1].vel_setpoint = 0;
        msg.leg_commands[i].motor_commands[1].torque_ff = 0;

        msg.leg_commands[i].motor_commands[2].pos_setpoint = knee_angle;
        msg.leg_commands[i].motor_commands[2].kp = 100;
        msg.leg_commands[i].motor_commands[2].kd = 3;
        msg.leg_commands[i].motor_commands[2].vel_setpoint = 0;
        msg.leg_commands[i].motor_commands[2].torque_ff = 0;
      }
    }
    break;

    case 1: //walk
    {

    }
    break;
  }

	msg.header.stamp = ros::Time::now();
	joint_control_pub_.publish(msg);
}