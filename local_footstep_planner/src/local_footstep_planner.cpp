#include "local_footstep_planner/local_footstep_planner.h"
#include <tf/tf.h>
#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string foot_plan_discrete_topic, foot_plan_continuous_topic;
  nh.param<std::string>("topics/terrain_map", terrain_map_topic_, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic_, "/body_plan");
  nh.param<std::string>("topics/foot_plan_discrete", foot_plan_discrete_topic, "/foot_plan_discrete");
  nh.param<std::string>("topics/foot_plan_continuous", foot_plan_continuous_topic, "/foot_plan_continuous");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("local_footstep_planner/update_rate", update_rate_, 1);
  nh.param<double>("local_footstep_planner/grf_weight", grf_weight_, 0.5);
  nh.param<double>("local_footstep_planner/max_footstep_horizon", max_footstep_horizon_, 1.5);
  nh.param<int>("local_footstep_planner/num_cycles", num_cycles_, 3);
  nh.param<double>("local_footstep_planner/period", period_, 0.25);
  nh.param<double>("local_footstep_planner/ground_clearance", ground_clearance_, 0.1);
  nh.param<double>("local_footstep_planner/interp_dt", interp_dt_, 0.01);

  if (grf_weight_>1 || grf_weight_<0) {
    grf_weight_ = std::min(std::max(grf_weight_,0.0),1.0);
    ROS_WARN("Invalid alpha in footstep planner, clamping to %4.2f", grf_weight_);
  }

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic_,1,
    &LocalFootstepPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic_,1,
    &LocalFootstepPlanner::bodyPlanCallback, this);
  foot_plan_discrete_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);
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
    s[3] = roll;
    s[4] = pitch;
    s[5] = yaw;
    s[6] = msg->states[i].twist.twist.linear.x;
    s[7] = msg->states[i].twist.twist.linear.y;
    s[8] = msg->states[i].twist.twist.linear.z;
    s[9] = msg->states[i].twist.twist.angular.x;
    s[10] = msg->states[i].twist.twist.angular.y;
    s[11] = msg->states[i].twist.twist.angular.z;
    body_plan_.push_back(s);

    Eigen::Vector3d force;
    tf::vectorMsgToEigen(msg->wrenches[i].force, force);
    body_wrench_plan_.push_back(force);
  }
}

void LocalFootstepPlanner::updateDiscretePlan() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  if (body_plan_.empty()) {
    ROS_WARN_THROTTLE(0.5, "No body plan found in LocalFootstepPlanner, exiting");
    return;
  }

  // Clear out the old footstep plan
  footstep_plan_.clear();
  footstep_plan_.resize(num_feet_);

  // Define the gait sequence (trot)
  double t_offsets[num_feet_] = {0.0, 0.5*period_, 0.5*period_, 0.0};
  double t_s[num_feet_] = {0.5*period_, 0.5*period_, 0.5*period_, 0.5*period_};

  // Specify the number of feet and their offsets from the COM
  double x_offsets[num_feet_] = {0.2263, -0.2263, 0.2263, -0.2263};
  double y_offsets[num_feet_] = {0.15, 0.15, -0.15, -0.15};

  // ros::Duration t = 0;ros::Time::now() - plan_timestamp_;
  // int start_index = t.toSec()/period_;
  int start_index = 0;
  int end_index = start_index + num_cycles_;

   // Loop through each foot
  for (int j=0; j<num_feet_; j++) {

    // Loop through each gait cycle
    for (int i = start_index; i < end_index; i++) {
      
      // Compute the initial time for this cycle
      double t_cycle = i*period_;
      double t_cycle_end = (i+1)*period_;
      if (t_cycle_end >=t_plan_.back()) {
        break;
      }

 
      FootstepState footstep(4);

      // Compute the touchdown and midstance times
      double t_touchdown = t_cycle + t_offsets[j];
      double t_midstance = t_cycle + t_offsets[j] + 0.5*t_s[j];

      BodyState s_touchdown = math_utils::interpMat(t_plan_, body_plan_, t_touchdown);
      BodyState s_midstance = math_utils::interpMat(t_plan_, body_plan_, t_midstance);
      BodyWrench grf_midstance = math_utils::interpVector3d(t_plan_, body_wrench_plan_, t_midstance);

      // Skip if this would occur during a flight phase
      double grf_tolerance = 1e-3;
      if (grf_midstance.norm() < grf_tolerance) {
        continue;
      }

      // Compute the body and hip positions and velocities
      double x_body = s_touchdown[0];
      double y_body = s_touchdown[1];
      double z_body = s_touchdown[2];
      double yaw = s_touchdown[5];
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
      Eigen::Vector3d footstep_nom = (1-grf_weight_)*hip_midstance + grf_weight_*footstep_grf;

      // Load the data into the footstep array and push into the plan
      footstep[0] = footstep_nom[0];
      footstep[1] = footstep_nom[1];
      footstep[2] = t_touchdown;
      footstep[3] = t_s[j];

      footstep_plan_[j].push_back(footstep);

    }

    // Add final foot configuration
    FootstepState footstep(4);

    BodyState s_final = body_plan_.back();

    double yaw = s_final[5];
    double x_hip = s_final[0] + x_offsets[j]*cos(yaw) - y_offsets[j]*sin(yaw);
    double y_hip = s_final[1] + x_offsets[j]*sin(yaw) + y_offsets[j]*cos(yaw);
    double z_hip = s_final[2]; // TODO add in pitch here

    // Load the data into the footstep array and push into the plan
    footstep[0] = x_hip;
    footstep[1] = y_hip;
    footstep[2] = t_plan_.back() - period_ +  t_offsets[j];
    footstep[3] = std::numeric_limits<double>::max();

    footstep_plan_[j].push_back(footstep);
  }

  // publishDiscretePlan();
  // publishContinuousPlan();
  // timer.report();
}

void LocalFootstepPlanner::publishContinuousPlan() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  // Make sure we already have footstep data
  if (footstep_plan_.empty()){
    ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not updating or publishing swing leg plan");
    return;
  }

  // Initialize the plan message, match overall plan timestamp
  spirit_msgs::MultiFootPlanContinuous multi_foot_plan_continuous_msg;
  multi_foot_plan_continuous_msg.header.frame_id = map_frame_;
  multi_foot_plan_continuous_msg.header.stamp = plan_timestamp_;

  // Make sure the footstep horizon is within bounds
  double footstep_horizon = std::min((num_cycles_)*period_, t_plan_.back());

  // Iterate through the footstep horizon
  for (double t = 0; t < footstep_horizon; t+=interp_dt_) {
    
    // Initialize MultiFootState message
    spirit_msgs::MultiFootState multi_foot_state_msg;
    multi_foot_state_msg.header.frame_id = 
      multi_foot_plan_continuous_msg.header.frame_id;
    multi_foot_state_msg.header.stamp = 
      multi_foot_plan_continuous_msg.header.stamp + ros::Duration(t);

    // Iterate through each foot
    for (int i=0; i<num_feet_; i++) {

      spirit_msgs::FootState foot_state_msg;
      foot_state_msg.header = multi_foot_state_msg.header;
      int state_index = 0;

      // Get the index of the current foot location
      for (int j = 0; j < (footstep_plan_[i].size()-1); j++) {
        state_index = j;
        if ( (t >= footstep_plan_[i][j][2] && t < footstep_plan_[i][j+1][2]) || 
          (t < footstep_plan_[i].front()[2]) ) {
          break;
        }
      }

      // Get current footstep state and correct timing
      FootstepState footstep = footstep_plan_[i][state_index];
      FootstepState next_footstep = footstep_plan_[i][state_index+1];

      double x = footstep[0];
      double y = footstep[1];
      double z = terrain_.getGroundHeight(x,y);
      double t_touchdown = footstep[2];
      double t_liftoff = t_touchdown + footstep[3];
      double t_next_touchdown = next_footstep[2];
      double t_f = t_next_touchdown - t_liftoff;

      if (t < t_liftoff) {

        // If in stance, just record the correct position
        foot_state_msg.position.x = x;
        foot_state_msg.position.y = y;
        foot_state_msg.position.z = z;
        foot_state_msg.velocity.x = 0;
        foot_state_msg.velocity.y = 0;
        foot_state_msg.velocity.z = 0;
        foot_state_msg.contact = true;

      } else if (t > t_next_touchdown) {

        // If reached the end of the sequence, just apply the last foot stance
        foot_state_msg.position.x = next_footstep[0];
        foot_state_msg.position.y = next_footstep[1];
        foot_state_msg.position.z = 
          terrain_.getGroundHeight(next_footstep[0],next_footstep[1]);
        foot_state_msg.velocity.x = 0;
        foot_state_msg.velocity.y = 0;
        foot_state_msg.velocity.z = 0;
        foot_state_msg.contact = true;

      } else {

        // If in swing, interpolate
        double x_next = next_footstep[0];
        double y_next = next_footstep[1];
        double z_next = terrain_.getGroundHeight(x_next,y_next);
        double z_mid = ground_clearance_ + std::max(z, z_next);
        double t_swing = t - t_liftoff;

        // cubic hermite interpolation: 
        // http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
        double u = t_swing/t_f;
        double u3 = u*u*u;
        double u2 = u*u;
        double basis_0 = 2*u3-3*u2+1;
        double basis_1 = -2*u3+3*u2;
        double basis_2 = 6*(u2-u);
        double basis_3 = 6*(u-u2);

        double x_current = basis_0*x + basis_1*x_next;
        double y_current = basis_0*y + basis_1*y_next;
        double dx_current = basis_2*x + basis_3*x_next;
        double dy_current = basis_2*y + basis_3*y_next;

        double z_current, dz_current;

        if (t_swing <0.5*t_f) {
          u = 2*t_swing/t_f;
          u3 = u*u*u;
          u2 = u*u;
          double basis_0 = 2*u3-3*u2+1;
          double basis_1 = -2*u3+3*u2;
          double basis_2 = 6*(u2-u);
          double basis_3 = 6*(u-u2);
          z_current = basis_0*z + basis_1*z_mid;
          dz_current = basis_2*z + basis_3*z_mid;
        } else {
          u = 2*t_swing/t_f - 1;
          u3 = u*u*u;
          u2 = u*u;
          double basis_0 = 2*u3-3*u2+1;
          double basis_1 = -2*u3+3*u2;
          double basis_2 = 6*(u2-u);
          double basis_3 = 6*(u-u2);
          z_current = basis_0*z_mid + basis_1*z_next;
          dz_current = basis_2*z_mid + basis_3*z_next;

        }

        foot_state_msg.position.x = x_current;
        foot_state_msg.position.y = y_current;
        foot_state_msg.position.z = z_current;
        foot_state_msg.velocity.x = dx_current;
        foot_state_msg.velocity.y = dy_current;
        foot_state_msg.velocity.z = dz_current;
        foot_state_msg.contact = false;

      }

      multi_foot_state_msg.feet.push_back(foot_state_msg);
    }

    multi_foot_plan_continuous_msg.states.push_back(multi_foot_state_msg);
  }

  foot_plan_continuous_pub_.publish(multi_foot_plan_continuous_msg);

  // timer.report();
}

void LocalFootstepPlanner::publishDiscretePlan() {

  if (footstep_plan_.empty()){
    ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not publishing");
    return;
  }
  
  // Initialize MultiFootPlanDiscrete message
  spirit_msgs::MultiFootPlanDiscrete multi_foot_plan_discrete_msg;
  multi_foot_plan_discrete_msg.header.stamp = plan_timestamp_;
  multi_foot_plan_discrete_msg.header.frame_id = map_frame_;

  // Loop through each foot
  for (int i=0;i<footstep_plan_.size(); ++i) {

    // Initialize to match the MultiFootPlanDiscrete header
    spirit_msgs::FootPlanDiscrete foot_plan_discrete_msg;
    foot_plan_discrete_msg.header = multi_foot_plan_discrete_msg.header;

    // Loop through each state for this foot
    for (int j=0;j<footstep_plan_[i].size(); ++j) {

      // Initialize a footstep message and load the data
      spirit_msgs::FootState foot_state_msg;

      foot_state_msg.position.x = footstep_plan_[i][j][0];
      foot_state_msg.position.y = footstep_plan_[i][j][1];
      foot_state_msg.position.z = terrain_.getGroundHeight(foot_state_msg.position.x,foot_state_msg.position.y);
      foot_state_msg.velocity.x = 0;
      foot_state_msg.velocity.y = 0;
      foot_state_msg.velocity.z = 0;
      foot_state_msg.contact = true;

      foot_state_msg.header.stamp = plan_timestamp_ +
        ros::Duration(footstep_plan_[i][j][2]);

      // foot_state.ts = ros::Duration(footstep_plan_[i][j][4]);

      foot_plan_discrete_msg.footholds.push_back(foot_state_msg);
    }

    multi_foot_plan_discrete_msg.feet.push_back(foot_plan_discrete_msg);
  }

  // Publish the whole plan to the topic
  foot_plan_discrete_pub_.publish(multi_foot_plan_discrete_msg);
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
    updateDiscretePlan();
    publishDiscretePlan();
    publishContinuousPlan();

    ros::spinOnce();
    r.sleep();
  }
}