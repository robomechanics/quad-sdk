#include "local_planner/local_footstep_planner.h"
#include <tf/tf.h>
#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner() {

  nominal_contact_schedule_.resize(horizon_length_);

  for (int i = 0; i < horizon_length_; i++) { // For each finite element
      
    nominal_contact_schedule_.at(i).resize(num_feet_);

    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) { // For each leg

      // If this index in the horizon is between touchdown and liftoff, set contact to true
      if ((i % period_) >= period_*phase_offsets_[leg_idx] && 
        (i % period_) < period_*(phase_offsets_[leg_idx] + duty_cycles_[leg_idx])) {

        nominal_contact_schedule_.at(i).at(leg_idx) = true;
      } else {
        nominal_contact_schedule_.at(i).at(leg_idx) = false;
      }
    }
  }

}

void LocalFootstepPlanner::setTemporalParams(double dt, int period, int horizon_length) {
  
  dt_ = dt;
  period_ = period;
  horizon_length_ = horizon_length;
}


void LocalFootstepPlanner::setSpatialParams(double ground_clearance, double grf_weight, 
  std::shared_ptr<spirit_utils::SpiritKinematics> kinematics) {

  ground_clearance_ = ground_clearance;
  grf_weight_ = grf_weight;
  kinematics_ = kinematics;
}

void LocalFootstepPlanner::computeContactSchedule(int phase,
  std::vector<std::vector<bool>> &contact_schedule) {

  contact_schedule = nominal_contact_schedule_;

  std::rotate(contact_schedule.begin(), contact_schedule.begin()+phase, contact_schedule.end());
}

void LocalFootstepPlanner::computeContactSchedule(double t,
  std::vector<std::vector<bool>> &contact_schedule) {

  int phase = std::ceil(t/(period_*dt_));
  computeContactSchedule(phase, contact_schedule);
}

void LocalFootstepPlanner::computeFootPositions(const Eigen::MatrixXd &body_plan,
  const Eigen::MatrixXd &grf_plan, const std::vector<std::vector<bool>> &contact_schedule,
  Eigen::MatrixXd &foot_positions) {

  foot_positions.setZero();

  // Loop through each foot
  for (int j=0; j<num_feet_; j++) {

    // Compute the number of timesteps corresponding to half the stance phase
    int half_duty_cycle = (period_*duty_cycles_[j])/2;

    // Loop through the horizon to identify instances of touchdown
    for (int i = 1; i < contact_schedule.size(); i++) {
      if (!contact_schedule.at(i-1).at(j) && contact_schedule.at(i).at(j)) {
        
        // Declare position vectors
        Eigen::Vector3d foot_position;
        Eigen::Vector3d foot_position_grf;
        Eigen::Vector3d foot_position_nominal;

        // Extract body and grf information
        Eigen::Vector3d body_pos_midstance = body_plan.block<1,3>(i + half_duty_cycle,0);
        Eigen::Vector3d body_rpy_midstance = body_plan.block<1,3>(i + half_duty_cycle,3);
        Eigen::Vector3d grf_midstance = grf_plan.block<1,3>(i + half_duty_cycle,3*j);

        // Compute nominal foot positions for kinematic and grf-projection measures
        Eigen::Vector3d hip_position_midstance;
        kinematics_->nominalFootstepFK(j, body_pos_midstance, body_rpy_midstance, 
          hip_position_midstance);
        foot_position_grf = terrain_.projectToMap(hip_position_midstance, -1.0*grf_midstance);

        // Combine these measures to get the nominal foot position
        foot_position_nominal = grf_weight_*foot_position_grf +
          (1-grf_weight_)*hip_position_midstance;
        foot_position_nominal.z() = terrain_.getGroundHeight(foot_position_nominal.x(),
          foot_position_nominal.y());

        // (Optional) Optimize the foothold location to get the final position
        // foot_position = map_search::optimizeFoothold(foot_position_nominal, stuff); ADAM
        foot_position = foot_position_nominal;

        // Store foot position in the Eigen matrix
        foot_positions.block<1,3>(i,3*j) = foot_position;

      } else {
        // If this isn't a new contact just hold the previous position
        // Note: this should get ignored for a foot in flight
        foot_positions.block<1,3>(i,3*j) = foot_positions.block<1,3>(i-1,3*j);
      }
    }
  }
}

void LocalFootstepPlanner::computeFootPlanMsgs(
  const std::vector<std::vector<bool>> &contact_schedule, const Eigen::MatrixXd &foot_positions
  spirit_msgs::MultiFootPlanDiscrete &multi_foot_plan_discrete_msg,
  spirit_msgs::MultiFootPlanContinuous &multi_foot_plan_continuous_msg) {

  // Loop through the horizon to construct the continuous foot plan message
  for (int i = 1; i < contact_schedule.size(); i++) {

    // Loop through each foot
    for (int j=0; j<num_feet_; j++) {

      // Create the foot state message
      spirit_msgs::FootState foot_state_msg;
      foot_state_msg.header = multi_foot_plan_discrete_msg.header;
      foot_state_msg.header.stamp = multi_foot_plan_discrete_msg.header.stamp + 
        ros::Duration(i*dt_);

      if (contact_schedule.at(i).at(j)) { // In contact

        foot_state_msg.position = eigenToFootStateMsg(foot_positions)

      } else { // In swing

      }

      if (!contact_schedule[i-1][j] && contact_schedule[i][j]) {

      }

    }
  }


}

// void LocalFootstepPlanner::updateDiscretePlan() {
//   // spirit_utils::FunctionTimer timer(__FUNCTION__);

//   if (body_plan_msg_ == NULL) {
//     ROS_WARN_THROTTLE(0.5, "No body plan in LocalFootstepPlanner, exiting");
//     return;
//   }

//   // Get the time associated with this data
//   t_plan_.clear();
//   primitive_id_plan_.clear();
//   for (int i = 0; i < body_plan_msg_->states.size(); i++) {
//     ros::Duration t_plan = body_plan_msg_->states[i].header.stamp - 
//       body_plan_msg_->states[0].header.stamp;
//     t_plan_.push_back(t_plan.toSec());
//     primitive_id_plan_.push_back(body_plan_msg_->primitive_ids[i]);
//   }

//   spirit_utils::SpiritKinematics kinematics;
  
//   // Clear out the old footstep plan
//   footstep_plan_.clear();
//   footstep_plan_.resize(num_feet_);

//   // Define the gait sequence (trot)
//   double t_offsets_trot[num_feet_] = {0.0, 0.5*period_, 0.5*period_, 0.0};
//   double t_offsets_bound[num_feet_] = {0.0, 0.0, 0.5*period_, 0.5*period_};
//   double t_s[num_feet_] = {0.5*period_, 0.5*period_, 0.5*period_, 0.5*period_};

//   // ros::Duration t = 0;ros::Time::now() - plan_timestamp_;
//   // int start_index = t.toSec()/period_;
//   int start_index = 0;
//   int end_index = start_index + num_cycles_;

//    // Loop through each foot
//   for (int j=0; j<num_feet_; j++) {

//     FootstepState footstep(4);

//     // If we have robot state data, apply the current foot position for the first cycle
//     if (robot_state_msg_ != NULL) {
//       // Load the data into the footstep array and push into the plan
//       footstep[0] = robot_state_msg_->feet.feet[j].position.x;
//       footstep[1] = robot_state_msg_->feet.feet[j].position.y;
//       footstep[2] = 0.0;
//       if (t_offsets_trot[j] < 0.5*period_) {
//         footstep[3] = t_s[j];
//       } else {
//         footstep[3] = period_;
//       }

//       footstep_plan_[j].push_back(footstep);
//       start_index = 1;
//     }

//     double t_cycle;
//     double t_cycle_end;

//     // Loop through each gait cycle
//     for (int i = start_index; i < end_index; i++) {

//       // Emptry prior footsteps
//       footstep.clear();
//       footstep.resize(4);
      
//       // Compute the initial time for this cycle
//       t_cycle = i*period_;
//       t_cycle_end = (i+1)*period_;
//       if (t_cycle_end >=t_plan_.back()) {
//         break;
//       }
 
//       // Compute the touchdown and midstance times
//       double t_touchdown = t_cycle + t_offsets_trot[j];
//       double t_midstance = t_cycle + t_offsets_trot[j] + 0.5*t_s[j];

//       nav_msgs::Odometry body_touchdown, body_midstance;
//       int primitive_id_touchdown, primitive_id_midstance;
//       spirit_msgs::GRFArray grf_array_touchdown, grf_array_midstance;

//       spirit_utils::interpBodyPlan((*body_plan_msg_), t_touchdown, body_touchdown,
//         primitive_id_touchdown, grf_array_touchdown);
//       spirit_utils::interpBodyPlan((*body_plan_msg_), t_midstance, body_midstance,
//         primitive_id_midstance, grf_array_midstance);

//       // Skip if this would occur during a flight phase
//       if (primitive_id_midstance == FLIGHT) {
//         continue;
//       }

//       // Compute the body and hip positions and velocities
//       Eigen::Vector3d body_pos_touchdown = {body_touchdown.pose.pose.position.x,
//         body_touchdown.pose.pose.position.y,
//         body_touchdown.pose.pose.position.z};

//       Eigen::Vector3d body_vel_midstance = {body_midstance.twist.twist.linear.x,
//         body_midstance.twist.twist.linear.y,
//         body_midstance.twist.twist.linear.z};

//       // Convert orientation from quaternion to rpy
//       tf2::Quaternion q;
//       tf2::convert(body_touchdown.pose.pose.orientation,q);
//       tf2::Matrix3x3 m(q);
//       double roll, pitch, yaw;
//       m.getRPY(roll, pitch, yaw);
//       Eigen::Vector3d body_rpy_touchdown = {roll,pitch,yaw};

//       Eigen::Vector3d nominal_footstep_pos_touchdown;
//       kinematics.nominalFootstepFK(j, body_pos_touchdown, body_rpy_touchdown, nominal_footstep_pos_touchdown);

//       Eigen::Vector3d grf_midstance = {grf_array_midstance.vectors[0].x,
//         grf_array_midstance.vectors[0].y,
//         grf_array_midstance.vectors[0].z,};

//       // Project along GRF from hips to the ground
//       // Eigen::Vector3d hip_midstance = {x_hip_midstance, y_hip_midstance, 
//       //   z_hip_midstance};
//       Eigen::Vector3d nominal_footstep_pos_midstance = nominal_footstep_pos_touchdown + 0.5*t_s[j]*body_vel_midstance;
//       Eigen::Vector3d footstep_grf = terrain_.projectToMap(nominal_footstep_pos_midstance, 
//         -1.0*grf_midstance);

//       // Define the nominal footstep location to lie on a line between the hips 
//       // projected vertically and along GRF (third entry is garbage)
//       Eigen::Vector3d footstep_nom = (1-grf_weight_)*nominal_footstep_pos_midstance + 
//         grf_weight_*footstep_grf;
//       // Eigen::Vector3d footstep_nom = nominal_footstep_pos_midstance;

//       // Load the data into the footstep array and push into the plan
//       footstep[0] = footstep_nom[0];
//       footstep[1] = footstep_nom[1];
//       footstep[2] = t_touchdown;
//       footstep[3] = t_s[j];

//       footstep_plan_[j].push_back(footstep);

//     }
    
//     if (t_cycle_end >=t_plan_.back()) {
//       // Add final foot configuration
//       nav_msgs::Odometry body_final = body_plan_msg_->states.back();

//       Eigen::Vector3d body_pos_final = {body_final.pose.pose.position.x,
//           body_final.pose.pose.position.y,
//           body_final.pose.pose.position.z};
      
//       tf2::Quaternion q;
//       tf2::convert(body_final.pose.pose.orientation,q);
//       tf2::Matrix3x3 m(q);
//       double roll, pitch, yaw;
//       m.getRPY(roll, pitch, yaw);
//       Eigen::Vector3d body_rpy_final = {roll,pitch,yaw};

//       Eigen::Vector3d nominal_footstep_pos_final;
//         kinematics.nominalFootstepFK(j, body_pos_final, body_rpy_final, nominal_footstep_pos_final);

//       footstep.clear();
//       footstep.resize(4);

//       // Load the data into the footstep array and push into the plan
//       footstep[0] = nominal_footstep_pos_final[0];
//       footstep[1] = nominal_footstep_pos_final[1];
//       footstep[2] = t_plan_.back() - period_ +  t_offsets_trot[j];
//       footstep[3] = std::numeric_limits<double>::max();

//       footstep_plan_[j].push_back(footstep);
//     }
//   }

// //   // publishDiscretePlan();
// //   // publishContinuousPlan();
// //   // timer.report();
// }

// void LocalFootstepPlanner::updateContinuousPlan() {
//   // spirit_utils::FunctionTimer timer(__FUNCTION__);

//   // Make sure we already have footstep data
//   if (footstep_plan_.empty()){
//     ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not updating or publishing"
//       " swing leg plan");
//     return;
//   }

//   // Initialize the plan message, match overall plan timestamp
//   multi_foot_plan_continuous_msg_.header.frame_id = map_frame_;
//   multi_foot_plan_continuous_msg_.header.stamp = plan_timestamp_;
//   multi_foot_plan_continuous_msg_.states.clear();

//   // Make sure the footstep horizon is within bounds
//   double footstep_horizon = std::min((num_cycles_)*period_, t_plan_.back());

//   // Iterate through the footstep horizon
//   for (double t = 0; t < footstep_horizon; t+=dt_) {
    
//     // Initialize MultiFootState message
//     spirit_msgs::MultiFootState multi_foot_state_msg;
//     multi_foot_state_msg.header.frame_id = 
//       multi_foot_plan_continuous_msg_.header.frame_id;
//     multi_foot_state_msg.header.stamp = 
//       multi_foot_plan_continuous_msg_.header.stamp + ros::Duration(t);

//     // Iterate through each foot
//     for (int i=0; i<num_feet_; i++) {

//       spirit_msgs::FootState foot_state_msg;
//       foot_state_msg.header = multi_foot_state_msg.header;
//       int state_index = 0;

//       // Get the index of the current foot location
//       for (int j = 0; j < (footstep_plan_[i].size()-1); j++) {
//         state_index = j;
//         if ( (t >= footstep_plan_[i][j][2] && t < footstep_plan_[i][j+1][2]) || 
//           (t < footstep_plan_[i].front()[2]) ) {
//           break;
//         }
//       }

//       // Get current footstep state and correct timing
//       FootstepState footstep = footstep_plan_[i][state_index];
//       FootstepState next_footstep = footstep_plan_[i][state_index+1];

//       double x = footstep[0];
//       double y = footstep[1];
//       double z = terrain_.getGroundHeight(x,y);
//       double t_touchdown = footstep[2];
//       double t_liftoff = t_touchdown + footstep[3];
//       double t_next_touchdown = next_footstep[2];
//       double t_f = t_next_touchdown - t_liftoff;

//       if (t < t_liftoff) {

//         // If in stance, just record the correct position
//         foot_state_msg.position.x = x;
//         foot_state_msg.position.y = y;
//         foot_state_msg.position.z = z;
//         foot_state_msg.velocity.x = 0;
//         foot_state_msg.velocity.y = 0;
//         foot_state_msg.velocity.z = 0;
//         foot_state_msg.contact = true;

//       } else if (t > t_next_touchdown) {

//         // If reached the end of the sequence, just apply the last foot stance
//         foot_state_msg.position.x = next_footstep[0];
//         foot_state_msg.position.y = next_footstep[1];
//         foot_state_msg.position.z = 
//           terrain_.getGroundHeight(next_footstep[0],next_footstep[1]);
//         foot_state_msg.velocity.x = 0;
//         foot_state_msg.velocity.y = 0;
//         foot_state_msg.velocity.z = 0;
//         foot_state_msg.contact = true;

//       } else {

//         // If in swing, interpolate
//         double x_next = next_footstep[0];
//         double y_next = next_footstep[1];
//         double z_next = terrain_.getGroundHeight(x_next,y_next);
//         double z_mid = ground_clearance_ + std::max(z, z_next);
//         double t_swing = t - t_liftoff;

//         // cubic hermite interpolation: 
//         // http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
//         double u = t_swing/t_f;
//         double u3 = u*u*u;
//         double u2 = u*u;
//         double basis_0 = 2*u3-3*u2+1;
//         double basis_1 = -2*u3+3*u2;
//         double basis_2 = 6*(u2-u);
//         double basis_3 = 6*(u-u2);

//         double x_current = basis_0*x + basis_1*x_next;
//         double y_current = basis_0*y + basis_1*y_next;
//         double dx_current = (basis_2*x + basis_3*x_next)/t_f;
//         double dy_current = (basis_2*y + basis_3*y_next)/t_f;

//         double z_current, dz_current;

//         if (t_swing <0.5*t_f) {
//           u = t_swing/(0.5*t_f);
//           u3 = u*u*u;
//           u2 = u*u;
//           double basis_0 = 2*u3-3*u2+1;
//           double basis_1 = -2*u3+3*u2;
//           double basis_2 = 6*(u2-u);
//           double basis_3 = 6*(u-u2);
//           z_current = basis_0*z + basis_1*z_mid;
//           dz_current = (basis_2*z + basis_3*z_mid)/(0.5*t_f);
//         } else {
//           u = t_swing/(0.5*t_f) - 1;
//           u3 = u*u*u;
//           u2 = u*u;
//           double basis_0 = 2*u3-3*u2+1;
//           double basis_1 = -2*u3+3*u2;
//           double basis_2 = 6*(u2-u);
//           double basis_3 = 6*(u-u2);
//           z_current = basis_0*z_mid + basis_1*z_next;
//           dz_current = (basis_2*z_mid + basis_3*z_next)/(0.5*t_f);

//         }

//         foot_state_msg.position.x = x_current;
//         foot_state_msg.position.y = y_current;
//         foot_state_msg.position.z = z_current;
//         foot_state_msg.velocity.x = dx_current;
//         foot_state_msg.velocity.y = dy_current;
//         foot_state_msg.velocity.z = dz_current;
//         foot_state_msg.contact = false;

//       }

//       multi_foot_state_msg.feet.push_back(foot_state_msg);
//     }

//     multi_foot_plan_continuous_msg_.states.push_back(multi_foot_state_msg);
//   }

//   // timer.report();
// }

// void LocalFootstepPlanner::publishDiscretePlan() {

//   if (footstep_plan_.empty()){
//     ROS_WARN_THROTTLE(0.5, "Footstep plan is empty, not publishing");
//     return;
//   }
  
//   // Initialize MultiFootPlanDiscrete message
//   spirit_msgs::MultiFootPlanDiscrete multi_foot_plan_discrete_msg;
//   multi_foot_plan_discrete_msg.header.stamp = plan_timestamp_;
//   multi_foot_plan_discrete_msg.header.frame_id = map_frame_;

//   // Loop through each foot
//   for (int i=0;i<footstep_plan_.size(); ++i) {

//     // Initialize to match the MultiFootPlanDiscrete header
//     spirit_msgs::FootPlanDiscrete foot_plan_discrete_msg;
//     foot_plan_discrete_msg.header = multi_foot_plan_discrete_msg.header;

//     // Loop through each state for this foot
//     for (int j=0;j<footstep_plan_[i].size(); ++j) {

//       // Initialize a footstep message and load the data
//       spirit_msgs::FootState foot_state_msg;

//       foot_state_msg.position.x = footstep_plan_[i][j][0];
//       foot_state_msg.position.y = footstep_plan_[i][j][1];

//       foot_state_msg.position.z = terrain_.getGroundHeight(
//         foot_state_msg.position.x,foot_state_msg.position.y);
//       foot_state_msg.velocity.x = 0;
//       foot_state_msg.velocity.y = 0;
//       foot_state_msg.velocity.z = 0;
//       foot_state_msg.contact = true;

//       foot_state_msg.header.stamp = plan_timestamp_ +
//         ros::Duration(footstep_plan_[i][j][2]);

//       // foot_state.ts = ros::Duration(footstep_plan_[i][j][4]);

//       foot_plan_discrete_msg.footholds.push_back(foot_state_msg);
//     }

//     multi_foot_plan_discrete_msg.feet.push_back(foot_plan_discrete_msg);
//   }

//   // Publish the whole plan to the topic
//   foot_plan_discrete_pub_.publish(multi_foot_plan_discrete_msg);
// }

// void LocalFootstepPlanner::publishContinuousPlan() {
//   foot_plan_continuous_pub_.publish(multi_foot_plan_continuous_msg_);
// }

// void LocalFootstepPlanner::waitForData() {
//     // Spin until terrain map message has been received and processed
//   boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
//   while((shared_map == nullptr) && ros::ok())
//   {
//     shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(
//       terrain_map_topic_, nh_);
//     ros::spinOnce();
//   }

//   boost::shared_ptr<spirit_msgs::BodyPlan const> shared_body_plan;
//   while((shared_body_plan == nullptr) && ros::ok())
//   {
//     shared_body_plan = ros::topic::waitForMessage<spirit_msgs::BodyPlan>(
//       body_plan_topic_, nh_);
//     ros::spinOnce();
//   }
// }

// void LocalFootstepPlanner::spin() {
//   ros::Rate r(update_rate_);

//   waitForData();

//   // Enter spin
//   while (ros::ok()) {
    
//     // Update the plan and publish it
//     if (update_flag_ == true) {
//       updateDiscretePlan();
//       updateContinuousPlan();
//       publishDiscretePlan();
//       publishContinuousPlan();
//       update_flag_ = false;
//     }    

//     ros::spinOnce();
//     r.sleep();
//   }
// }