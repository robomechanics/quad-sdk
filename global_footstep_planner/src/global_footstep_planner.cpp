#include "global_footstep_planner/global_footstep_planner.h"
#include <tf/tf.h>
#include <chrono>

GlobalFootstepPlanner::GlobalFootstepPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string foot_plan_discrete_topic, foot_plan_continuous_topic, robot_state_topic;

  nh.param<std::string>("topics/terrain_map", 
    terrain_map_topic_, "/terrain_map");
  nh.param<std::string>("topics/global_plan", body_plan_topic_, "/body_plan");
  nh.param<std::string>("topics/foot_plan_discrete", 
    foot_plan_discrete_topic, "/foot_plan_discrete");
  nh.param<std::string>("topics/foot_plan_continuous", 
    foot_plan_continuous_topic, "/foot_plan_continuous");
  nh.param<std::string>("topics/state/ground_truth", 
    robot_state_topic, "/state/ground_truth");
  nh.param<std::string>("map_frame",map_frame_,"map");

  nh.param<double>("global_footstep_planner/update_rate", update_rate_, 1);
  nh.param<double>("global_footstep_planner/grf_weight", grf_weight_, 0.5);
  nh.param<double>("global_footstep_planner/max_footstep_horizon", 
    max_footstep_horizon_, 1.5);
  nh.param<int>("global_footstep_planner/num_cycles", num_cycles_, 3);
  nh.param<double>("global_footstep_planner/period", period_, 0.25);
  nh.param<double>("global_footstep_planner/ground_clearance", 
    ground_clearance_, 0.1);
  nh.param<double>("global_footstep_planner/interp_dt", interp_dt_, 0.01);

  if (grf_weight_>1 || grf_weight_<0) {
    grf_weight_ = std::min(std::max(grf_weight_,0.0),1.0);
    ROS_WARN("Invalid grf weight, clamping to %4.2f", grf_weight_);
  }

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic_,1,
    &GlobalFootstepPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic_,1,
    &GlobalFootstepPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,
    &GlobalFootstepPlanner::robotStateCallback, this);
  foot_plan_discrete_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);
  foot_plan_continuous_pub_ = nh_.advertise<
    spirit_msgs::MultiFootPlanContinuous>(foot_plan_continuous_topic,1);

}

void GlobalFootstepPlanner::terrainMapCallback(
  const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);
}

void GlobalFootstepPlanner::robotPlanCallback(
  const spirit_msgs::RobotPlan::ConstPtr& msg) {

  // Only update if the new plan is different
  if (body_plan_msg_ != NULL) {
    if (msg->states.back().header.stamp == body_plan_msg_->states.back().header.stamp) {
      return;
    }
  }
  
  update_flag_ = true;
  body_plan_msg_ = msg;
  plan_timestamp_ = msg->header.stamp;

}

void GlobalFootstepPlanner::robotStateCallback(
  const spirit_msgs::RobotState::ConstPtr& msg) {

  if (msg->feet.feet.empty())
    return;

  if (body_plan_msg_ == NULL)
    return;

  if (robot_state_msg_ == NULL) {
    robot_state_msg_ = msg;
  }

}

double GlobalFootstepPlanner::computeTimeUntilNextFlight(double t) {
  double t_remaining = std::numeric_limits<double>::max();
  for (int i = 1; i < t_plan_.size(); i++) {
    if (t_plan_[i] > t && primitive_id_plan_[i] == FLIGHT) {

      if (primitive_id_plan_[i-1] == FLIGHT) {
        t_remaining = 0;
      } else {
        t_remaining = t_plan_[i] - t;

      }
      break;
    }
  }

  return t_remaining;
}

void GlobalFootstepPlanner::updateDiscretePlan() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  if (body_plan_msg_ == NULL) {
    ROS_WARN_THROTTLE(0.5, "No body plan in GlobalFootstepPlanner, exiting");
    return;
  }

  // Get the time associated with this data
  t_plan_.clear();
  primitive_id_plan_.clear();
  for (int i = 0; i < body_plan_msg_->states.size(); i++) {
    ros::Duration t_plan = body_plan_msg_->states[i].header.stamp - 
      body_plan_msg_->states[0].header.stamp;
    t_plan_.push_back(t_plan.toSec());
    primitive_id_plan_.push_back(body_plan_msg_->primitive_ids[i]);
  }

  spirit_utils::SpiritKinematics kinematics;
  
  // Clear out the old footstep plan
  footstep_plan_.clear();
  footstep_plan_.resize(num_feet_);

  // Define the gait sequence (trot)
  double t_offsets_trot[num_feet_] = {0.0, 0.5*period_, 0.5*period_, 0.0};
  double t_offsets_bound[num_feet_] = {0.0, 0.0, 0.5*period_, 0.5*period_};
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

    FootstepState footstep(4);

    // If we have robot state data, apply the current foot position for the first cycle
    if (robot_state_msg_ != NULL) {
      // Load the data into the footstep array and push into the plan
      footstep[0] = robot_state_msg_->feet.feet[j].position.x;
      footstep[1] = robot_state_msg_->feet.feet[j].position.y;
      footstep[2] = 0.0;
      if (t_offsets_trot[j] < 0.5*period_) {
        footstep[3] = t_s[j];
      } else {
        footstep[3] = period_;
      }

      footstep_plan_[j].push_back(footstep);
      start_index = 1;
    }

    double t_cycle;
    double t_cycle_end;

    // Loop through each gait cycle
    for (int i = start_index; i < end_index; i++) {

      // Emptry prior footsteps
      footstep.clear();
      footstep.resize(4);
      
      // Compute the initial time for this cycle
      t_cycle = i*period_;
      t_cycle_end = (i+1)*period_;
      if (t_cycle_end >=t_plan_.back()) {
        break;
      }
 
      // Compute the touchdown and midstance times
      double t_touchdown = t_cycle + t_offsets_trot[j];
      double t_midstance = t_cycle + t_offsets_trot[j] + 0.5*t_s[j];

      spirit_msgs::RobotState state_touchdown, state_midstance;
      int primitive_id_touchdown, primitive_id_midstance;
      spirit_msgs::GRFArray grf_array_touchdown, grf_array_midstance;

      spirit_utils::interpRobotPlan((*body_plan_msg_), t_touchdown, state_touchdown,
        primitive_id_touchdown, grf_array_touchdown);
      spirit_utils::interpRobotPlan((*body_plan_msg_), t_midstance, state_midstance,
        primitive_id_midstance, grf_array_midstance);

      // Skip if this would occur during a flight phase
      if (primitive_id_midstance == FLIGHT) {
        continue;
      }

      // Compute the body and hip positions and velocities
      Eigen::Vector3d body_pos_touchdown = {state_touchdown.body.pose.pose.position.x,
        state_touchdown.body.pose.pose.position.y,
        state_touchdown.body.pose.pose.position.z};

      Eigen::Vector3d body_vel_midstance = {state_midstance.body.twist.twist.linear.x,
        state_midstance.body.twist.twist.linear.y,
        state_midstance.body.twist.twist.linear.z};

      // Convert orientation from quaternion to rpy
      tf2::Quaternion q;
      tf2::convert(state_touchdown.body.pose.pose.orientation,q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      Eigen::Vector3d body_rpy_touchdown = {roll,pitch,yaw};

      Eigen::Vector3d nominal_footstep_pos_touchdown;
      kinematics.nominalFootstepFK(j, body_pos_touchdown, body_rpy_touchdown, nominal_footstep_pos_touchdown);

      Eigen::Vector3d grf_midstance = {grf_array_midstance.vectors[0].x,
        grf_array_midstance.vectors[0].y,
        grf_array_midstance.vectors[0].z,};

      // Project along GRF from hips to the ground
      // Eigen::Vector3d hip_midstance = {x_hip_midstance, y_hip_midstance, 
      //   z_hip_midstance};
      Eigen::Vector3d nominal_footstep_pos_midstance = nominal_footstep_pos_touchdown + 0.5*t_s[j]*body_vel_midstance;
      Eigen::Vector3d footstep_grf = terrain_.projectToMap(nominal_footstep_pos_midstance, 
        -1.0*grf_midstance);

      // Define the nominal footstep location to lie on a line between the hips 
      // projected vertically and along GRF (third entry is garbage)
      Eigen::Vector3d footstep_nom = (1-grf_weight_)*nominal_footstep_pos_midstance + 
        grf_weight_*footstep_grf;
      // Eigen::Vector3d footstep_nom = nominal_footstep_pos_midstance;

      // Load the data into the footstep array and push into the plan
      footstep[0] = footstep_nom[0];
      footstep[1] = footstep_nom[1];
      footstep[2] = t_touchdown;
      footstep[3] = t_s[j];

      footstep_plan_[j].push_back(footstep);

    }
    
    if (t_cycle_end >=t_plan_.back()) {
      // Add final foot configuration
      spirit_msgs::RobotState state_final = body_plan_msg_->states.back();

      Eigen::Vector3d body_pos_final = {state_final.body.pose.pose.position.x,
          state_final.body.pose.pose.position.y,
          state_final.body.pose.pose.position.z};
      
      tf2::Quaternion q;
      tf2::convert(state_final.body.pose.pose.orientation,q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      Eigen::Vector3d body_rpy_final = {roll,pitch,yaw};

      Eigen::Vector3d nominal_footstep_pos_final;
        kinematics.nominalFootstepFK(j, body_pos_final, body_rpy_final, nominal_footstep_pos_final);

      footstep.clear();
      footstep.resize(4);

      // Load the data into the footstep array and push into the plan
      footstep[0] = nominal_footstep_pos_final[0];
      footstep[1] = nominal_footstep_pos_final[1];
      footstep[2] = t_plan_.back() - period_ +  t_offsets_trot[j];
      footstep[3] = std::numeric_limits<double>::max();

      footstep_plan_[j].push_back(footstep);
    }
  }

  // publishDiscretePlan();
  // publishContinuousPlan();
  // timer.report();
}

void GlobalFootstepPlanner::updateContinuousPlan() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  // Make sure we already have footstep data
  if (footstep_plan_.empty()){
    ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not updating or publishing"
      " swing leg plan");
    return;
  }

  // Initialize the plan message, match overall plan timestamp
  multi_foot_plan_continuous_msg_.header.frame_id = map_frame_;
  multi_foot_plan_continuous_msg_.header.stamp = plan_timestamp_;
  multi_foot_plan_continuous_msg_.states.clear();

  // Make sure the footstep horizon is within bounds
  double footstep_horizon = std::min((num_cycles_)*period_, t_plan_.back());

  // Iterate through the footstep horizon
  for (double t = 0; t < footstep_horizon; t+=interp_dt_) {
    
    // Initialize MultiFootState message
    spirit_msgs::MultiFootState multi_foot_state_msg;
    multi_foot_state_msg.header.frame_id = 
      multi_foot_plan_continuous_msg_.header.frame_id;
    multi_foot_state_msg.header.stamp = 
      multi_foot_plan_continuous_msg_.header.stamp + ros::Duration(t);

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
        double dx_current = (basis_2*x + basis_3*x_next)/t_f;
        double dy_current = (basis_2*y + basis_3*y_next)/t_f;

        double z_current, dz_current;

        if (t_swing <0.5*t_f) {
          u = t_swing/(0.5*t_f);
          u3 = u*u*u;
          u2 = u*u;
          double basis_0 = 2*u3-3*u2+1;
          double basis_1 = -2*u3+3*u2;
          double basis_2 = 6*(u2-u);
          double basis_3 = 6*(u-u2);
          z_current = basis_0*z + basis_1*z_mid;
          dz_current = (basis_2*z + basis_3*z_mid)/(0.5*t_f);
        } else {
          u = t_swing/(0.5*t_f) - 1;
          u3 = u*u*u;
          u2 = u*u;
          double basis_0 = 2*u3-3*u2+1;
          double basis_1 = -2*u3+3*u2;
          double basis_2 = 6*(u2-u);
          double basis_3 = 6*(u-u2);
          z_current = basis_0*z_mid + basis_1*z_next;
          dz_current = (basis_2*z_mid + basis_3*z_next)/(0.5*t_f);

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

    multi_foot_plan_continuous_msg_.states.push_back(multi_foot_state_msg);
  }

  // timer.report();
}

void GlobalFootstepPlanner::publishDiscretePlan() {

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

      foot_state_msg.position.z = terrain_.getGroundHeight(
        foot_state_msg.position.x,foot_state_msg.position.y);
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

void GlobalFootstepPlanner::publishContinuousPlan() {
  foot_plan_continuous_pub_.publish(multi_foot_plan_continuous_msg_);
}

void GlobalFootstepPlanner::waitForData() {
    // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while((shared_map == nullptr) && ros::ok())
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(
      terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<spirit_msgs::RobotPlan const> shared_body_plan;
  while((shared_body_plan == nullptr) && ros::ok())
  {
    shared_body_plan = ros::topic::waitForMessage<spirit_msgs::RobotPlan>(
      body_plan_topic_, nh_);
    ros::spinOnce();
  }
}

void GlobalFootstepPlanner::spin() {
  ros::Rate r(update_rate_);

  waitForData();

  // Enter spin
  while (ros::ok()) {
    
    // Update the plan and publish it
    if (update_flag_ == true) {
      updateDiscretePlan();
      updateContinuousPlan();
      publishDiscretePlan();
      publishContinuousPlan();
      update_flag_ = false;
    }    

    ros::spinOnce();
    r.sleep();
  }
}