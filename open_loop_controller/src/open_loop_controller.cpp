#include "open_loop_controller/open_loop_controller.h"

OpenLoopController::OpenLoopController(ros::NodeHandle nh) {
	nh_ = nh;

  // Get rosparams
  std::string leg_control_topic,control_mode_topic;
  quad_utils::loadROSParam(nh_,"topics/control/joint_command",leg_control_topic);
  quad_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  quad_utils::loadROSParam(nh_,"open_loop_controller/update_rate",update_rate_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/stand_angles",stand_joint_angles_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/stand_kp",stand_kp_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/stand_kd",stand_kd_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/walk_kp",walk_kp_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/walk_kd",walk_kd_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/leg_phases",leg_phases_);
  quad_utils::loadROSParam(nh_,"open_loop_controller/use_diff_for_velocity",use_diff_for_velocity_);

  // Start sitting
  control_mode_ = 0;

  // Interpolate between waypoints
  this->setupTrajectory();
  
  // Setup pubs and subs
	joint_control_pub_ = nh_.advertise<quad_msgs::LegCommandArray>(leg_control_topic,1);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&OpenLoopController::controlModeCallback, this);
}


void OpenLoopController::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  if (0 <= msg->data && msg->data <= 2)
  {
    control_mode_ = msg->data;
  }
}

void OpenLoopController::setupTrajectory()
{
  std::vector<double> xs,ys,ts;
  quad_utils::loadROSParam(nh_,"open_loop_controller/waypoint_ts",ts);
  quad_utils::loadROSParam(nh_,"open_loop_controller/waypoint_xs",xs);
  quad_utils::loadROSParam(nh_,"open_loop_controller/waypoint_ys",ys);
  quad_utils::loadROSParam(nh_,"open_loop_controller/interp_dt",interp_dt_);

  double dt = 0.005;

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

    double t = t0+interp_dt_;
    while(t <= t1)
    {
      double r = (t-t0)/(t1-t0); // ratio of xfirst to xlast to use
      double x = xfirst + r*(xlast-xfirst);
      double y = yfirst + r*(ylast-yfirst);
      target_pts_.push_back(this->compute2DIk(x,y));
      target_times_.push_back(t);
      t += interp_dt_;
    }
    t_run += ts.at(i);
  }
  t_cycle_ = target_times_.back();
}

std::pair<double,double> OpenLoopController::compute2DIk(double x, double y)
{
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
	quad_msgs::LegCommandArray msg;
	msg.leg_commands.resize(4);

  switch (control_mode_){
    case 0: // sit
    {
      for (int i = 0; i < 4; ++i)
      {
        msg.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j)
        {
          msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          msg.leg_commands.at(i).motor_commands.at(j).kp = 5;
          msg.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
          msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    }
    break;
    
    case 1: // stand
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

    case 2: // walk
    {
      for (int i = 0; i < 4; ++i)
      {
        // Get relative time through gait for this leg (inc phase info)
        double t = fmodf(elapsed_time + t_cycle_*leg_phases_.at(i),t_cycle_);

        // Find target position from precomputed trajectory
        auto it = std::upper_bound(target_times_.begin(), target_times_.end(),t);
        int target_idx = it - target_times_.begin();

        // Convert to joint angles
        std::pair<double,double> hip_knee_angs = target_pts_.at(target_idx);

        // Find target velocity from central difference of precomputed trajectory
        int prev_idx = target_idx == 0 ? target_pts_.size() - 1 : target_idx - 1;
        int next_idx = target_idx == target_pts_.size() - 1? 0 : target_idx + 1;
        std::pair<double,double> prev_hip_knee_angs = target_pts_.at(prev_idx);
        std::pair<double,double> next_hip_knee_angs = target_pts_.at(next_idx);
        double hip_vel = (next_hip_knee_angs.first - prev_hip_knee_angs.first)/(2*interp_dt_);
        double knee_vel = (next_hip_knee_angs.second - prev_hip_knee_angs.second)/(2*interp_dt_);

        // Fill out motor command
        msg.leg_commands.at(i).motor_commands.resize(3);

        msg.leg_commands.at(i).motor_commands.at(0).pos_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(0).kp = walk_kp_.at(0);
        msg.leg_commands.at(i).motor_commands.at(0).kd = walk_kd_.at(0);
        msg.leg_commands.at(i).motor_commands.at(0).vel_setpoint = 0;
        msg.leg_commands.at(i).motor_commands.at(0).torque_ff = 0;

        msg.leg_commands.at(i).motor_commands.at(1).pos_setpoint = hip_knee_angs.first;
        msg.leg_commands.at(i).motor_commands.at(1).kp = walk_kp_.at(1);
        msg.leg_commands.at(i).motor_commands.at(1).kd = walk_kd_.at(1);
        msg.leg_commands.at(i).motor_commands.at(1).vel_setpoint = use_diff_for_velocity_ ? hip_vel : 0;
        msg.leg_commands.at(i).motor_commands.at(1).torque_ff = 0;

        msg.leg_commands.at(i).motor_commands.at(2).pos_setpoint = hip_knee_angs.second;
        msg.leg_commands.at(i).motor_commands.at(2).kp = walk_kp_.at(2);
        msg.leg_commands.at(i).motor_commands.at(2).kd = walk_kd_.at(2);
        msg.leg_commands.at(i).motor_commands.at(2).vel_setpoint = use_diff_for_velocity_ ? knee_vel : 0;
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