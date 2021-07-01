#include "local_planner/local_footstep_planner.h"
#include <tf/tf.h>
#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner() {

}

void LocalFootstepPlanner::setTemporalParams(double dt, int period, int horizon_length) {
  
  dt_ = dt;
  period_ = period;
  horizon_length_ = horizon_length;

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
        // ROS_WARN("Contact mode is fixed in full stance for testing purposes!");
      }
    }
  }
}


void LocalFootstepPlanner::setSpatialParams(double ground_clearance, double grf_weight, 
  double standing_error_threshold, std::shared_ptr<spirit_utils::SpiritKinematics> kinematics) {

  ground_clearance_ = ground_clearance;
  standing_error_threshold_ = standing_error_threshold;
  grf_weight_ = grf_weight;
  kinematics_ = kinematics;
}

void LocalFootstepPlanner::updateMap(const FastTerrainMap &terrain) {
  terrain_ = terrain;
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(const Eigen::VectorXd &body_plan,
  const Eigen::VectorXd &foot_positions_world, Eigen::VectorXd &foot_positions_body) {

  for (int i = 0; i < num_feet_; i++) {
    foot_positions_body.segment<3>(3*i) = foot_positions_world.segment<3>(3*i) - 
      body_plan.segment<3>(0);
  }

}

void LocalFootstepPlanner::getFootPositionsBodyFrame(const Eigen::MatrixXd &body_plan,
  const Eigen::MatrixXd &foot_positions_world, Eigen::MatrixXd &foot_positions_body) {

  Eigen::VectorXd foot_pos = Eigen::VectorXd::Zero(3*num_feet_);
  for (int i = 0; i < horizon_length_; i++) {
    foot_pos.setZero();
    getFootPositionsBodyFrame(body_plan.row(i), foot_positions_world.row(i),
      foot_pos);
    foot_positions_body.row(i) = foot_pos;
  }
}

void LocalFootstepPlanner::computeStanceContactSchedule(int current_plan_index,
  std::vector<std::vector<bool>> &contact_schedule) {

  for (int i = 0; i < horizon_length_; i++) { // For each finite element
    contact_schedule.at(i).resize(num_feet_);
    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) { // For each leg
        nominal_contact_schedule_.at(i).at(leg_idx) = true;
    }
  }
}

void LocalFootstepPlanner::computeContactSchedule(int current_plan_index,
  Eigen::VectorXd current_state, Eigen::MatrixXd ref_body_plan,
  std::vector<std::vector<bool>> &contact_schedule) {

  int phase = current_plan_index % period_;
  contact_schedule = nominal_contact_schedule_;
  std::rotate(contact_schedule.begin(), contact_schedule.begin()+phase, contact_schedule.end());

}

void LocalFootstepPlanner::computeSwingFootState(const Eigen::Vector3d &foot_position_prev,
  const Eigen::Vector3d &foot_position_next, double swing_phase, int swing_duration,
  Eigen::Vector3d &foot_position, Eigen::Vector3d &foot_velocity) {

  assert((swing_phase >= 0) && (swing_phase <= 1));

  // Compute interpolation parameters
  double phi = swing_phase;
  double phi3 = phi*phi*phi;
  double phi2 = phi*phi;
  double basis_0 = 2*phi3-3*phi2+1;
  double basis_1 = -2*phi3+3*phi2;
  double basis_2 = 6*(phi2-phi);
  double basis_3 = 6*(phi-phi2);

  // Perform cubic hermite interpolation
  foot_position = basis_0*foot_position_prev.array() + basis_1*foot_position_next.array();
  foot_velocity = (basis_2*foot_position_prev.array() +
    basis_3*foot_position_next.array())/(swing_duration*dt_);

  // Update z to clear both footholds by the specified height
  double swing_apex = ground_clearance_ + std::max(foot_position_prev.z(), foot_position_next.z());
  double phi_z = 2*fmod(phi, 0.5);
  phi3 = phi_z*phi_z*phi_z;
  phi2 = phi_z*phi_z;
  basis_0 = 2*phi3-3*phi2+1;
  basis_1 = -2*phi3+3*phi2;
  basis_2 = 6*(phi2-phi_z);
  basis_3 = 6*(phi_z-phi2);

  if (phi<0.5) {
    foot_position.z() = basis_0*foot_position_prev.z() + basis_1*swing_apex;
    foot_velocity.z() = (basis_2*foot_position_prev.z() + basis_3*swing_apex)/
      (0.5*swing_duration*dt_);
  } else {
    foot_position.z() = basis_0*swing_apex + basis_1*foot_position_next.z();
    foot_velocity.z() = (basis_2*swing_apex + basis_3*foot_position_next.z())/
      (0.5*swing_duration*dt_);
  }  
}

void LocalFootstepPlanner::computeFootPositions(const Eigen::MatrixXd &body_plan,
  const Eigen::MatrixXd &grf_plan, const std::vector<std::vector<bool>> &contact_schedule,
  Eigen::MatrixXd &foot_positions) {

  // Loop through each foot
  for (int j=0; j<num_feet_; j++) {

    // Compute the number of timesteps corresponding to half the stance phase
    int half_duty_cycle = (period_*duty_cycles_[j])/2;

    // Loop through the horizon to identify instances of touchdown
    for (int i = 1; i < contact_schedule.size(); i++) {

      if (isNewContact(contact_schedule, i, j)) {
      
        // Declare foot position vectors
        Eigen::Vector3d foot_position, foot_position_grf,
          foot_position_nominal, hip_position_midstance, centrifugal;

        // Declare body and grf vectors
        Eigen::Vector3d body_pos_midstance, body_rpy_midstance, 
          body_vel_touchdown, body_ang_vel_touchdown, grf_midstance;

        // Extract body and grf information
        int midstance = std::min(i + half_duty_cycle, horizon_length_-1);
        body_pos_midstance = body_plan.block<1,3>(midstance,0);
        body_rpy_midstance = body_plan.block<1,3>(midstance,3);
        body_vel_touchdown = body_plan.block<1,3>(i,6);
        body_ang_vel_touchdown = body_plan.block<1,3>(i,9);
        grf_midstance = grf_plan.block<1,3>(midstance,3*j);

        // Compute nominal foot positions for kinematic and grf-projection measures
        kinematics_->nominalHipFK(j, body_pos_midstance, body_rpy_midstance, 
          hip_position_midstance);
        double hip_height = hip_position_midstance.z() - 
          terrain_.getGroundHeight(hip_position_midstance.x(), hip_position_midstance.y());
        centrifugal = sqrt(hip_height/9.81)*body_vel_touchdown.cross(body_ang_vel_touchdown);
        foot_position_grf = terrain_.projectToMap(hip_position_midstance, -1.0*grf_midstance);

        // Combine these measures to get the nominal foot position and grab correct height
        foot_position_nominal = grf_weight_*foot_position_grf +
          (1-grf_weight_)*(hip_position_midstance + centrifugal);
        foot_position_nominal.z() = terrain_.getGroundHeight(foot_position_nominal.x(),
          foot_position_nominal.y());

        // (Optional) Optimize the foothold location to get the final position
        // foot_position = map_search::optimizeFoothold(foot_position_nominal, stuff); // ADAM
        foot_position = foot_position_nominal;

        // Store foot position in the Eigen matrix
        foot_positions.block<1,3>(i,3*j) = foot_position;    

      } else {
        // If this isn't a new contact just hold the previous position
        // Note: this should get ignored for a foot in flight
        foot_positions.block<1,3>(i,3*j) = getFootData(foot_positions, i-1, j);
      }
    }
  }
}

void LocalFootstepPlanner::computeFootPlanMsgs(
  const std::vector<std::vector<bool>> &contact_schedule, const Eigen::MatrixXd &foot_positions,
  int current_plan_index, spirit_msgs::MultiFootPlanDiscrete &past_footholds_msg,
  spirit_msgs::MultiFootPlanDiscrete &future_footholds_msg,
  spirit_msgs::MultiFootPlanContinuous &foot_plan_continuous_msg) {

  foot_plan_continuous_msg.states.resize(contact_schedule.size());
  future_footholds_msg.feet.resize(num_feet_);

  // Loop through each foot to construct the continuous foot plan message
  for (int j=0; j<num_feet_; j++) {

    future_footholds_msg.feet[j].header = future_footholds_msg.header;

    // Declare variables for computing initial swing foot state
    // Identify index for the liftoff and touchdown events
    spirit_msgs::FootState most_recent_foothold_msg = past_footholds_msg.feet[j].footholds.back();
    
    int i_liftoff = most_recent_foothold_msg.traj_index - current_plan_index;
    int i_touchdown = getNextContactIndex(contact_schedule, 0, j);
    int swing_duration = i_touchdown - i_liftoff;

    // Identify positions of the previous and next footholds
    Eigen::Vector3d foot_position_prev;
    spirit_utils::footStateMsgToEigen(most_recent_foothold_msg, foot_position_prev);
    Eigen::Vector3d foot_position_next = getFootData(foot_positions, i_touchdown, j);

    // Loop through the horizon
    for (int i = 0; i < contact_schedule.size(); i++) {

      // Create the foot state message
      spirit_msgs::FootState foot_state_msg;
      foot_state_msg.header = foot_plan_continuous_msg.header;
      foot_state_msg.header.stamp = foot_plan_continuous_msg.header.stamp + 
        ros::Duration(i*dt_);
      foot_state_msg.traj_index = current_plan_index + i;

      Eigen::Vector3d foot_position;
      Eigen::Vector3d foot_velocity;

      // Determine the foot state at this index
      if (isContact(contact_schedule, i, j)) { // In contact

        // Log current foot position and zero velocity
        foot_position = getFootData(foot_positions, i, j);
        foot_velocity = Eigen::VectorXd::Zero(3);
        foot_state_msg.contact = true;

      } else { // In swing

        // If this is a new swing phase, update the swing phase variables
        if (isNewLiftoff(contact_schedule, i, j)) {

          // Set the indices for liftoff and touchdown
          i_liftoff = i;
          i_touchdown = getNextContactIndex(contact_schedule, i, j);
          swing_duration = i_touchdown - i_liftoff;

          // Loop through contact schedule to find the next touchdown, otherwise keep the default
          foot_position_prev = getFootData(foot_positions, i_liftoff, j);
          foot_position_next = getFootData(foot_positions, i_touchdown, j);
        }

        // Compute the current position and velocity from the swing phase variables
        double swing_phase = (i - i_liftoff)/(double)swing_duration;
        computeSwingFootState(foot_position_prev, foot_position_next, swing_phase, swing_duration,
          foot_position, foot_velocity);
        foot_state_msg.contact = false;

      }

      // Load state data into the message
      spirit_utils::eigenToFootStateMsg(foot_position, foot_velocity, foot_state_msg);
      foot_plan_continuous_msg.states[i].feet.push_back(foot_state_msg);

      // If this is a touchdown event, add to the future footholds message
      if (isNewContact(contact_schedule, i, j)) {
        future_footholds_msg.feet[j].footholds.push_back(foot_state_msg);
      }

      // If this is the end of a contact, add to the past footholds message
      if (i == 1 && isNewLiftoff(contact_schedule, i, j)) {
        past_footholds_msg.feet[j].footholds.push_back(foot_state_msg);
      }
    }
  }
}
