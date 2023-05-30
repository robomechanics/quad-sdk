#include "spine_controller/spine_controller.h"

SpineController::SpineController(ros::NodeHandle nh) {
  nh_ = nh;

  // Get rosparams
  std::string leg_control_topic, control_mode_topic;
  quad_utils::loadROSParam(nh_, "topics/control/joint_command",
                           leg_control_topic);
  quad_utils::loadROSParam(nh_, "topics/control/mode", control_mode_topic);

  quad_utils::loadROSParam(nh_, "/topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "/topics/state/ground_truth",
                           robot_state_topic);
  quad_utils::loadROSParam(nh_, "/topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "spine_controller/update_rate",
                           update_rate_);
  quad_utils::loadROSParam(nh_, "spine_controller/leg_phases", leg_phases_);
  quad_utils::loadROSParam(nh_, "spine_controller/use_diff_for_velocity",
                           use_diff_for_velocity_);
  quad_utils::loadROSParam(nh_, "spine_controller/set_angle",
                           set_angle_);
  quad_utils::loadROSParam(nh_, "spine_controller/open_kp",
                           open_kp_);
  quad_utils::loadROSParam(nh_, "spine_controller/open_kd",
                           open_kd_);

  // Locked spine angle
  control_mode_ = 0;

  // Plan desired spine angles
  this->planTrajectory();

  // Interpolate between waypoints
  this->setupTrajectory();

  // Setup pubs and subs

  body_plan_sub_ =
      nh_.subscribe(body_plan_topic, 1, &SpineController::robotPlanCallback, this,
                    ros::TransportHints().tcpNoDelay(true));
  robot_state_sub_ =
      nh_.subscribe(robot_state_topic, 1, &SpineController::robotStateCallback,
                    this, ros::TransportHints().tcpNoDelay(true));
  local_plan_sub_ =
      nh_.subscribe(local_plan_topic, 1, &SpineController::localPlanCallback, this,
                    ros::TransportHints().tcpNoDelay(true));
  joint_control_pub_ =
      nh_.advertise<quad_msgs::LegCommandArray>(leg_control_topic, 1);
  control_mode_sub_ = nh_.subscribe(
      control_mode_topic, 1, &SpineController::controlModeCallback, this);

}

// init current states
current_state_ = Eigen::VectorXd::Zero(12);
current_foot_positions_world_ = Eigen::VectorXd::Zero(12);

void SpineController::controlModeCallback(
    const std_msgs::UInt8::ConstPtr& msg) {
  if (0 <= msg->data && msg->data <= 1) {
    control_mode_ = msg->data;
  }
}

void SpineController::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg) {
  body_plan_msg_ = msg;
}

void SpineController::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg) {
  // Make sure the data is actually populated
  if (msg->feet.feet.empty() || msg->joints.position.empty()) return;

  robot_state_msg_ = msg;
}

void SpineController::localPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg) {
  last_local_plan_msg_ = msg;
}



void SpineController::planTrajectory() {
  
  quad_utils::FunctionTimer timer(__FUNCTION__); 
  
  // get current states
  current_state_ = quad_utils::bodyStateMsgToEigen(robot_state_msg_->body);
  quad_utils::multiFootStateMsgToEigen(robot_state_msg_->feet,
                                       current_foot_positions_world_);

  /* TO DO
  * previous: get angle of global plan and use for spine angle, not really working, instead try: 
  * get angle of global plan where front and back body segments are, and have each match global plan orientation
  * send that torque to spine joint
  */

  // NOTE: two lines below use waypoint times and angles hardcoded in the yaml
  // DELETE or comment out to use something else calculated here
  quad_utils::loadROSParam(nh_, "spine_controller/waypoint_ts", waypoints_ts_);
  quad_utils::loadROSParam(nh_, "spine_controller/waypoint_angs", waypoint_angs_);

}

void SpineController::setupTrajectory() {

  quad_utils::loadROSParam(nh_, "spine_controller/interp_dt", interp_dt_);

  double dt = 0.005;

  // Interpolate between points with fixed dt
  double t_run = 0;
  for (int i = 0; i < waypoints_ts_.size(); ++i) {
    double afirst = waypoint_angs_.at(i);
    double t0 = t_run;

    int idx_last = i + 1;
    if (i == waypoint_ts_.size() - 1) idx_last = 0;

    double alast = waypoint_angs_.at(idx_last);
    double t1 = t_run + waypoint_ts_.at(i);

    double t = t0 + interp_dt_;
    while (t <= t1) {
      double r = (t - t0) / (t1 - t0);  // ratio of afirst to alast to use
      double a = afirst + r * (alast - afirst);
      target_pts_.push_back(a);
      target_times_.push_back(t);
      t += interp_dt_;
    }
    t_run += waypoint_ts_.at(i);
  }
}

void SpineController::sendJointPositions(double& elapsed_time) {
  quad_msgs::LegCommandArray spine_msg;
  spine_msg.leg_commands.resize(1); // usually 4, for 4 legs
  spine_msg.leg_commands.at(0).motor_commands.resize(1); // usually 3, for 3DOF per leg

  switch (control_mode_) {
    case 0:  // locked spine angle
    {
        spine_msg.leg_commands.at(0).motor_commands.at(0).pos_setpoint = set_angle_;
        spine_msg.leg_commands.at(0).motor_commands.at(0).vel_setpoint = 0;
        spine_msg.leg_commands.at(0).motor_commands.at(0).kp = 5;
        spine_msg.leg_commands.at(0).motor_commands.at(0).kd = 0.1;
        spine_msg.leg_commands.at(0).motor_commands.at(0).torque_ff = 0;

    } break;

    case 1:  // send to lower level PD
    {
        // Find target position from precomputed trajectory
        auto it =
            std::upper_bound(target_times_.begin(), target_times_.end(), t);
        int target_idx = it - target_times_.begin();

        // Get desired angle for given index
        double spine_ang = target_pts_.at(target_idx);

        // Make desired velocity central difference of precomputed
        // trajectory
        // turn this on/off with use_diff_for_velocity in yaml
        int prev_idx =
            target_idx == 0 ? target_pts_.size() - 1 : target_idx - 1;
        int next_idx =
            target_idx == target_pts_.size() - 1 ? 0 : target_idx + 1;
        double prev_spine_ang = target_pts_.at(prev_idx);
        double next_spine_ang = target_pts_.at(next_idx);
        double spine_vel = (next_spine_ang.first - prev_spine_ang.first) /
                         (2 * interp_dt_);

        // Fill out motor command
        spine_msg.leg_commands.at(0).motor_commands.at(0).pos_setpoint = spine_ang;
        spine_msg.leg_commands.at(0).motor_commands.at(0).kp = open_kp_;
        spine_msg.leg_commands.at(0).motor_commands.at(0).kd = open_kd_;
        spine_msg.leg_commands.at(0).motor_commands.at(0).vel_setpoint = use_diff_for_velocity_ ? spine_vel : 0;
        spine_msg.leg_commands.at(0).motor_commands.at(0).torque_ff = 0;

    } break;

  }
  msg.header.stamp = ros::Time::now();
  joint_control_pub_.publish(msg);
}

void SpineController::spin() {
  double start_time = ros::Time::now().toSec();
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    double elapsed_time = ros::Time::now().toSec() - start_time;
    this->sendJointPositions(elapsed_time);
    ros::spinOnce();
    r.sleep();
  }
}
