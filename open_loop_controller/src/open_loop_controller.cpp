#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh) {
	nh_ = nh;

  // Get rosparams
  std::string leg_control_topic;
  nh.getParam("topics/joint_command", leg_control_topic);
  nh.getParam("open_loop_controller/control_mode", mode_);

  this->setupTrajectory();
  
  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>("/spirit/joint_controller/command",1);
}

void OpenLoopController::setupTrajectory()
{
  std::vector<double> xs = {-0.12,0,0.12}; 
  std::vector<double> ys = {-0.24,-0.14,0.24};
  std::vector<double> ts = {0.2,0.2,0.2};

  double dt = 0.05;

  // Interpolate between points with fixed dt
  double t_run = 0;
  for (int i = 0; i < ts.size(); ++i)
  {
    double xfirst = xs.at(i);
    double yfirst = ys.at(i);
    double t0 = t_run; 

    int idx_last = i+1;
    if (i == ts.size() - 1) idx_last = 0;

    double xlast = xs.at(idx_last);
    double ylast = ys.at(idx_last);
    double t1 = t_run + ts.at(i);

    double t = t0+dt;
    while(t <= t1)
    {
      double r = (t-t0)/(t1-t0); // ratio of xfirst to xlast to use
      double x = xfirst + r*(xlast-xfirst);
      double y = yfirst + r*(ylast-yfirst);
      target_pts_.push_back(std::make_pair(x,y));
      target_times_.push_back(t);
      t += dt;
    }

    t_run += ts.at(i);
  }

  /*
  for (size_t i = 0; i < target_pts_.size(); ++i)
  {
    std::cout << target_pts_.at(i).first << ", " << target_pts_.at(i).second << std::endl;
    std::cout << target_times_.at(i) << std::endl;
  }
  */

  t_cycle_ = ts.back();
}

std::pair<double,double> OpenLoopController::computeIk(std::pair<double,double> pt)
{
  double x = pt.first;
  double y = pt.second;
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

    case 1: // walk
    {
      double t_base = fmodf(elapsed_time,t_cycle_); // Offset on a leg by leg basis
      for (int i = 0; i < 4; ++i)
      {
        double t = t_base; // Assume all legs are on same phase for now
        auto it = std::upper_bound(target_times_.begin(), target_times_.end(),t);
        int target_idx = it - target_times_.begin();

        std::pair<double,double> hip_knee_angs = this->computeIk(target_pts_.at(target_idx));

        msg.leg_commands[i].motor_commands.resize(3);

        msg.leg_commands[i].motor_commands[0].pos_setpoint = 0;
        msg.leg_commands[i].motor_commands[0].kp = 100;
        msg.leg_commands[i].motor_commands[0].kd = 3;
        msg.leg_commands[i].motor_commands[0].vel_setpoint = 0;
        msg.leg_commands[i].motor_commands[0].torque_ff = 0;

        msg.leg_commands[i].motor_commands[1].pos_setpoint = hip_knee_angs.first;
        msg.leg_commands[i].motor_commands[1].kp = 100;
        msg.leg_commands[i].motor_commands[1].kd = 3;
        msg.leg_commands[i].motor_commands[1].vel_setpoint = 0;
        msg.leg_commands[i].motor_commands[1].torque_ff = 0;

        msg.leg_commands[i].motor_commands[2].pos_setpoint = hip_knee_angs.second;
        msg.leg_commands[i].motor_commands[2].kp = 100;
        msg.leg_commands[i].motor_commands[2].kd = 3;
        msg.leg_commands[i].motor_commands[2].vel_setpoint = 0;
        msg.leg_commands[i].motor_commands[2].torque_ff = 0;
      }

    }
    break;
  }

	msg.header.stamp = ros::Time::now();
	joint_control_pub_.publish(msg);
}