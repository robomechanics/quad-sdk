#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh) {
	nh_ = nh;

  // Get rosparams
  std::string leg_control_topic;
  spirit_utils::loadROSParam(nh_,"topics/joint_command",leg_control_topic);
  spirit_utils::loadROSParamDefault(nh_,"open_loop_controller/control_mode",mode_,0);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/update_rate",update_rate_);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/stand_angles",stand_joint_angles_);

  spirit_utils::loadROSParam(nh_,"open_loop_controller/stand_kp",stand_kp_);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/stand_kd",stand_kd_);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/walk_kp",walk_kp_);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/walk_kd",walk_kd_);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/leg_phases",leg_phases_);

  // Interpolate between waypoints
  this->setupTrajectory();
  
  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>("/spirit/joint_controller/command",1);
}

void OpenLoopController::setupTrajectory()
{
  std::vector<double> xs,ys,ts;
  spirit_utils::loadROSParam(nh_,"open_loop_controller/waypoint_ts",ts);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/waypoint_xs",xs);
  spirit_utils::loadROSParam(nh_,"open_loop_controller/waypoint_ys",ys);
  double dt;
  spirit_utils::loadROSParam(nh_,"open_loop_controller/interp_dt",dt);

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
  t_cycle_ = target_times_.back();
}

std::pair<double,double> OpenLoopController::compute2DIk(std::pair<double,double> pt)
{
  double x = pt.first;
  double y = pt.second;
  double l1 = 0.2075;
  double l2 = 0.2075;
  double q2 = acos((x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2));
  double q1 = atan2(y,x) - atan2(l2*sin(q2),l1+l2*cos(q2));

  double theta_hip = q1 - M_PI;
  double theta_knee = M_PI - q2;

  if (theta_hip < -M_PI) theta_hip += 2*M_PI;
  if (theta_hip > M_PI) theta_hip -= 2*M_PI;
  if (theta_knee < -M_PI) theta_knee += 2*M_PI;
  if (theta_knee > M_PI) theta_knee -= 2*M_PI;
  return std::make_pair(theta_hip,theta_knee);
}

void OpenLoopController::sendJointPositions(double &elapsed_time)
{
	spirit_msgs::LegCommandArray msg;
	msg.leg_commands.resize(4);

  switch (mode_){
    case 0: // stand
    {
      for (int i = 0; i < 4; ++i)
      {
        msg.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j)
        {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).kp = stand_kp_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).kd = stand_kd_.at(j);
          msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    }
    break;

    case 1: // walk
    {
      for (int i = 0; i < 4; ++i)
      {
        // Get relative time through gait for this leg (inc phase info)

        double t = fmodf(elapsed_time + t_cycle_*leg_phases_.at(i),t_cycle_);

        // Find target position from precomputed trajectory
        auto it = std::upper_bound(target_times_.begin(), target_times_.end(),t);
        int target_idx = it - target_times_.begin();
        std::pair<double,double> hip_knee_angs = this->compute2DIk(target_pts_.at(target_idx));

        msg.leg_commands.at(i).motor_commands.resize(3);

        msg.leg_commands.at(i).motor_commands.at(0).pos_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(0).kp = walk_kp_.at(0);
        msg.leg_commands.at(i).motor_commands.at(0).kd = walk_kd_.at(0);
        msg.leg_commands.at(i).motor_commands.at(0).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(0).torque_ff = 0;

        msg.leg_commands.at(i).motor_commands.at(1).pos_setpoint = hip_knee_angs.first;
        msg.leg_commands.at(i).motor_commands.at(1).kp = walk_kp_.at(1);
        msg.leg_commands.at(i).motor_commands.at(1).kd = walk_kd_.at(1);
        msg.leg_commands.at(i).motor_commands.at(1).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(1).torque_ff = 0;

        msg.leg_commands.at(i).motor_commands.at(2).pos_setpoint = hip_knee_angs.second;
        msg.leg_commands.at(i).motor_commands.at(2).kp = walk_kp_.at(2);
        msg.leg_commands.at(i).motor_commands.at(2).kd = walk_kd_.at(2);
        msg.leg_commands.at(i).motor_commands.at(2).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(2).torque_ff = 0;
      }
    }
    break;
  }
	msg.header.stamp = ros::Time::now();
	joint_control_pub_.publish(msg);
}

void OpenLoopController::spin() {
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    double elapsed_time = ros::Time::now().toSec() - start_time;
    this->sendJointPositions(elapsed_time);
    ros::spinOnce();
    r.sleep();
  }
}