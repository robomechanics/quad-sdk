#include "local_footstep_planner/local_footstep_planner.h"
#include <tf/tf.h>
#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string footstep_plan_topic, swing_leg_plan_topic;
  nh.param<std::string>("topics/terrain_map", terrain_map_topic_, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic_, "/body_plan");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<std::string>("topics/swing_leg_plan", swing_leg_plan_topic, "/swing_leg_plan");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("local_footstep_planner/update_rate", update_rate_, 1);
  nh.param<double>("local_footstep_planner/alpha", alpha_, 0.5);
  nh.param<double>("local_footstep_planner/max_footstep_horizon", max_footstep_horizon_, 1.5);
  nh.param<double>("local_footstep_planner/period", period_, 0.25);
  nh.param<double>("local_footstep_planner/ground_clearance", ground_clearance_, 0.1);
  nh.param<double>("local_footstep_planner/interp_dt", interp_dt_, 0.01);

  if (alpha_>1 || alpha_<0) {
    alpha_ = std::min(std::max(alpha_,0.0),1.0);
    ROS_WARN("Invalid alpha in footstep planner, clamping to %4.2f", alpha_);
  }

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic_,1,&LocalFootstepPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic_,1,&LocalFootstepPlanner::bodyPlanCallback, this);
  footstep_plan_pub_ = nh_.advertise<spirit_msgs::FootstepPlan>(footstep_plan_topic,1);
  swing_leg_plan_pub_ = nh_.advertise<spirit_msgs::SwingLegPlan>(swing_leg_plan_topic,1);
}

void LocalFootstepPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);
}

void LocalFootstepPlanner::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  t_plan_.clear();
  body_plan_.clear();
  body_wrench_plan_.clear();

  plan_timestamp_ = msg->header.stamp;

  // Loop through the message to get the state info and add to private vector
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    // Convert orientation from quaternion to rpy
    tf::Quaternion q(
        msg->states[i].pose.pose.orientation.x,
        msg->states[i].pose.pose.orientation.y,
        msg->states[i].pose.pose.orientation.z,
        msg->states[i].pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get the time associated with this data
    ros::Duration t_plan = msg->states[i].header.stamp - msg->states[0].header.stamp;
    t_plan_.push_back(t_plan.toSec());

    // Get the state associated with this data
    std::vector<double> s(12);
    s[0] = msg->states[i].pose.pose.position.x;
    s[1] = msg->states[i].pose.pose.position.y;
    s[2] = msg->states[i].pose.pose.position.z;
    s[3] = msg->states[i].twist.twist.linear.x;
    s[4] = msg->states[i].twist.twist.linear.y;
    s[5] = msg->states[i].twist.twist.linear.z;
    s[6] = roll;
    s[7] = pitch;
    s[8] = yaw;
    s[9] = msg->states[i].twist.twist.angular.x;
    s[10] = msg->states[i].twist.twist.angular.y;
    s[11] = msg->states[i].twist.twist.angular.z;
    body_plan_.push_back(s);

    Eigen::Vector3d force;
    tf::vectorMsgToEigen(msg->wrenches[i].force, force);
    body_wrench_plan_.push_back(force);
  }
}

void LocalFootstepPlanner::updatePlan() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  if (body_plan_.empty())
    return;

  // Clear out the old footstep plan
  footstep_plan_.clear();
  footstep_plan_.resize(num_feet_);

  // Define the gait sequence (trot)
  double t_offsets[num_feet_] = {0.0, 0.5*period_, 0.5*period_, 0.0};
  double t_s[num_feet_] = {0.5*period_, 0.5*period_, 0.5*period_, 0.5*period_};

  double footstep_horizon = std::min(max_footstep_horizon_, t_plan_.back());
  int num_cycles = footstep_horizon/period_;

  // Specify the number of feet and their offsets from the COM
  double x_offsets[num_feet_] = {0.25, -0.25, 0.25, -0.25};
  double y_offsets[num_feet_] = {0.15, 0.15, -0.15, -0.15};

  ros::Duration t = ros::Time::now() - plan_timestamp_;
  int start_index = t.toSec()/period_;
  int end_index = start_index + num_cycles;

  // Loop through each gait cycle
  for (int i = start_index; i < end_index; i++) {
    
    // Compute the initial time for this cycle
    double t_cycle = i*period_;

    // Loop through each foot
    for (int j=0; j<num_feet_; j++) {
      FootstepState footstep(5);

      // Compute the touchdown and midstance times
      double t_touchdown = t_cycle + t_offsets[j];
      double t_midstance = t_cycle + t_offsets[j] + 0.5*t_s[j];

      BodyState s_touchdown = math_utils::interpMat(t_plan_, body_plan_, t_touchdown);
      BodyState s_midstance = math_utils::interpMat(t_plan_, body_plan_, t_midstance);
      BodyWrench grf_midstance = math_utils::interpVector3d(t_plan_, body_wrench_plan_, t_midstance);

      // Compute the body and hip positions and velocities
      double x_body = s_touchdown[0];
      double y_body = s_touchdown[1];
      double z_body = s_touchdown[2];
      double yaw = s_touchdown[8];
      double x_hip = x_body + x_offsets[j]*cos(yaw) - y_offsets[j]*sin(yaw);
      double y_hip = y_body + x_offsets[j]*sin(yaw) + y_offsets[j]*cos(yaw);
      double z_hip = z_body; // TODO add in pitch here

      double dx_body = s_midstance[3];
      double dy_body = s_midstance[4];
      double dz_body = s_midstance[5];
      double x_hip_midstance = x_hip + 0.5*t_s[j]*dx_body;
      double y_hip_midstance = y_hip + 0.5*t_s[j]*dy_body;
      double z_hip_midstance = z_hip + 0.5*t_s[j]*dz_body;

      // Project along GRF from hips to the ground
      Eigen::Vector3d hip_midstance = {x_hip_midstance, y_hip_midstance, z_hip_midstance};
      Eigen::Vector3d footstep_grf = terrain_.projectToMap(hip_midstance, -1.0*grf_midstance);

      // Define the nominal footstep location to lie on a line between the hips projected vertically and along GRF (third entry is garbage)
      Eigen::Vector3d footstep_nom = (1-alpha_)*hip_midstance + alpha_*footstep_grf;

      // Load the data into the footstep array and push into the plan
      footstep[0] = j;
      footstep[1] = footstep_nom[0];
      footstep[2] = footstep_nom[1];
      footstep[3] = t_touchdown;
      footstep[4] = t_s[j];

      footstep_plan_[j].push_back(footstep);

    }
  }

  // timer.report();
}

void LocalFootstepPlanner::publishSwingLegPlan() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  if (footstep_plan_.empty()){
    ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not publishing");
    return;
  }

  spirit_msgs::SwingLegPlan swing_leg_plan_all;
  swing_leg_plan_all.header.frame_id = map_frame_;
  swing_leg_plan_all.header.stamp = plan_timestamp_;

  for (int i=0; i<num_feet_; i++) {
    nav_msgs::Path current_swing_leg_plan;

    current_swing_leg_plan.header = swing_leg_plan_all.header;

    for (int j = 0; j < footstep_plan_[i].size()-1; j++) {

      FootstepState footstep = footstep_plan_[i][j];
      FootstepState next_footstep = footstep_plan_[i][j+1];

      // Add current footstep state and correct time
      double t_touchdown = footstep[3];

      // Incrementally compute swing leg trajectory until time for next footstep
      double t_liftoff = t_touchdown + footstep[4];
      double t_next_touchdown = next_footstep[3];
      double t_f = t_next_touchdown - t_liftoff;

      // Get knot points for cubic hermite interpolation
      double x = footstep[1];
      double x_next = next_footstep[1];
      double y = footstep[2];
      double y_next = next_footstep[2];
      double z = terrain_.getGroundHeight(x,y);
      double z_next = terrain_.getGroundHeight(x_next,y_next);
      double z_mid = ground_clearance_ + std::max(z, z_next);

      // Add the foot location at the beginning of stance with the correct time
      geometry_msgs::PoseStamped foot_msg;
      foot_msg.header.stamp = current_swing_leg_plan.header.stamp + ros::Duration(t_touchdown);
      foot_msg.pose.position.x = x;
      foot_msg.pose.position.y = y;
      foot_msg.pose.position.z = z;
      current_swing_leg_plan.poses.push_back(foot_msg);

      // Add all the foot locations during the swing phase with the correct times
      for (double t = 0; t < t_f; t+=interp_dt_) {
        // cubic hermite interpolation: http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
        double u = t/t_f;
        double u3 = u*u*u;
        double u2 = u*u;
        double basis_3 = 2*u3-3*u2+1;
        double basis_2 = -2*u3+3*u2;
       
        double x_current = basis_3*x + basis_2*x_next;
        double y_current = basis_3*y + basis_2*y_next;
        double z_current;

        if (t <0.5*t_f) {
          u = 2*t/t_f;
          u3 = u*u*u;
          u2 = u*u;
          double basis_3 = 2*u3-3*u2+1;
          double basis_2 = -2*u3+3*u2;
          z_current = basis_3*z + basis_2*z_mid;
        } else {
          u = 2*t/t_f - 1;
          u3 = u*u*u;
          u2 = u*u;
          double basis_3 = 2*u3-3*u2+1;
          double basis_2 = -2*u3+3*u2;
          z_current = basis_3*z_mid + basis_2*z_next;
        }

        geometry_msgs::PoseStamped foot_msg;
        foot_msg.header.stamp = current_swing_leg_plan.header.stamp + ros::Duration(t_liftoff+t);
        foot_msg.pose.position.x = x_current;
        foot_msg.pose.position.y = y_current;
        foot_msg.pose.position.z = z_current;
        current_swing_leg_plan.poses.push_back(foot_msg);

      }
    }

    swing_leg_plan_all.legs.push_back(current_swing_leg_plan);
  }

  swing_leg_plan_pub_.publish(swing_leg_plan_all);

  // timer.report();
}

void LocalFootstepPlanner::publishPlan() {

  if (footstep_plan_.empty()){
    ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not publishing");
    return;
  }
  // Initialize FootstepPlan message
  spirit_msgs::FootstepPlan footstep_plan_msg;
  footstep_plan_msg.header.stamp = plan_timestamp_;
  footstep_plan_msg.header.frame_id = map_frame_;

  // Loop through the plan
  for (int i=0;i<footstep_plan_.size(); ++i) {
    spirit_msgs::SingleFootstepPlan single_footstep_plan_msg;
    for (int j=0;j<footstep_plan_[i].size(); ++j) {

      // Initialize a footstep message and load the data
      spirit_msgs::Footstep footstep;

      footstep.index = footstep_plan_[i][j][0];
      footstep.position.x = footstep_plan_[i][j][1];
      footstep.position.y = footstep_plan_[i][j][2];
      footstep.position.z = terrain_.getGroundHeight(footstep.position.x,footstep.position.y);
      footstep.td = ros::Duration(footstep_plan_[i][j][3]);
      footstep.ts = ros::Duration(footstep_plan_[i][j][4]);

      single_footstep_plan_msg.steps.push_back(footstep);
    }

    footstep_plan_msg.feet.push_back(single_footstep_plan_msg);
  }

  // Publish the whole plan to the topic
  footstep_plan_pub_.publish(footstep_plan_msg);
}

void LocalFootstepPlanner::waitForData() {
    // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while((shared_map == nullptr) && ros::ok())
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<spirit_msgs::BodyPlan const> shared_body_plan;
  while((shared_body_plan == nullptr) && ros::ok())
  {
    shared_body_plan = ros::topic::waitForMessage<spirit_msgs::BodyPlan>(body_plan_topic_, nh_);
    ros::spinOnce();
  }
}

void LocalFootstepPlanner::spin() {
  ros::Rate r(update_rate_);

  waitForData();

  // Enter spin
  while (ros::ok()) {
    // ROS_INFO("In LocalFootstepPlanner spin, updating at %4.1f Hz", update_rate_);
    
    // Update the plan and publish it
    updatePlan();
    publishPlan();
    publishSwingLegPlan();

    ros::spinOnce();
    r.sleep();
  }
}