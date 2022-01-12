#include "local_planner/local_footstep_planner.h"
#include <tf/tf.h>
#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner() {

}

void LocalFootstepPlanner::setTemporalParams(double dt, int period, int horizon_length, 
  const std::vector<double> &duty_cycles, const std::vector<double> &phase_offsets) {
  
  dt_ = dt;
  period_ = period;
  horizon_length_ = horizon_length;
  duty_cycles_ = duty_cycles;
  phase_offsets_ = phase_offsets;

  nominal_contact_schedule_.resize(period_);

  for (int i = 0; i < period_; i++) { // For each finite element
      
    nominal_contact_schedule_.at(i).resize(num_feet_);

    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) { // For each leg

      // If this index in the horizon is between touchdown and liftoff, set contact to true
      if ((i >= period_*phase_offsets_[leg_idx] && 
        i < period_*(phase_offsets_[leg_idx] + duty_cycles_[leg_idx])) || 
        i < period_*(phase_offsets_[leg_idx] + duty_cycles_[leg_idx] - 1.0)) {

        nominal_contact_schedule_.at(i).at(leg_idx) = true;
      } else {
        nominal_contact_schedule_.at(i).at(leg_idx) = false;
      }
    }
  }
}


void LocalFootstepPlanner::setSpatialParams(double ground_clearance, double hip_clearance, double grf_weight, 
  double standing_error_threshold, std::shared_ptr<quad_utils::QuadKD> kinematics) {

  ground_clearance_ = ground_clearance;
  hip_clearance_ = hip_clearance;
  standing_error_threshold_ = standing_error_threshold;
  grf_weight_ = grf_weight;
  quadKD_ = kinematics;
}

void LocalFootstepPlanner::updateMap(const FastTerrainMap &terrain) {
  terrain_ = terrain;
}

void LocalFootstepPlanner::updateMap(const grid_map::GridMap &terrain) {
  terrain_grid_ = terrain;
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

  // Compute the current phase in the nominal contact schedule
  int phase = current_plan_index % period_;

  // Compute the current contact schedule by looping through the nominal starting at the phase
  contact_schedule.resize(horizon_length_);
  for (int i = 0; i < horizon_length_; i++) {
    contact_schedule[i] = nominal_contact_schedule_[(i + phase) % period_];
  }
}

void LocalFootstepPlanner::computeSwingFootState(const Eigen::Vector3d &foot_position_prev,
  const Eigen::Vector3d &foot_position_next, double swing_phase, int swing_duration, const Eigen::VectorXd &body_plan, int leg_index,
  Eigen::Vector3d &foot_position, Eigen::Vector3d &foot_velocity, Eigen::Vector3d &foot_acceleration) {

  assert((swing_phase >= 0) && (swing_phase <= 1));

  // Compute interpolation parameters
  double phi = swing_phase;
  double phi3 = phi*phi*phi;
  double phi2 = phi*phi;
  double basis_0 = 2*phi3-3*phi2+1;
  double basis_1 = -2*phi3+3*phi2;
  double basis_2 = 6*(phi2-phi);
  double basis_3 = 6*(phi-phi2);
  double basis_4 = 12*phi-6;
  double basis_5 = 6-12*phi;

  // Perform cubic hermite interpolation
  foot_position = basis_0*foot_position_prev.array() + basis_1*foot_position_next.array();
  foot_velocity = (basis_2*foot_position_prev.array() +
    basis_3*foot_position_next.array())/(swing_duration*dt_);
  foot_acceleration = (basis_4*foot_position_prev.array() +
    basis_5*foot_position_next.array())/pow(swing_duration*dt_, 2);

  // Compute hip height according to the MPC plan
  Eigen::Vector3d body_pos, body_rpy;
  body_pos = body_plan.segment(0, 3);
  body_rpy = body_plan.segment(3, 3);
  Eigen::Matrix4d g_world_legbase;
  quadKD_->worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);
  
  // Update z to clear both footholds by the specified height under the constraints of hip height
  double swing_apex = std::min(ground_clearance_ + std::max(foot_position_prev.z(), foot_position_next.z()), g_world_legbase(2, 3) - hip_clearance_);
  double phi_z = 2*fmod(phi, 0.5);
  phi3 = phi_z*phi_z*phi_z;
  phi2 = phi_z*phi_z;
  basis_0 = 2*phi3-3*phi2+1;
  basis_1 = -2*phi3+3*phi2;
  basis_2 = 6*(phi2-phi_z);
  basis_3 = 6*(phi_z-phi2);
  basis_4 = 12*phi_z-6;
  basis_5 = 6-12*phi_z;

  if (phi<0.5) {
    foot_position.z() = basis_0*foot_position_prev.z() + basis_1*swing_apex;
    foot_velocity.z() = (basis_2*foot_position_prev.z() + basis_3*swing_apex)/
      (0.5*swing_duration*dt_);
    foot_acceleration.z() = (basis_4*foot_position_prev.z() + basis_5*swing_apex)/
      pow(swing_duration*dt_, 2);
  } else {
    foot_position.z() = basis_0*swing_apex + basis_1*foot_position_next.z();
    foot_velocity.z() = (basis_2*swing_apex + basis_3*foot_position_next.z())/
      (0.5*swing_duration*dt_);
    foot_acceleration.z() = (basis_4*swing_apex + basis_5*foot_position_next.z())/
      pow(swing_duration*dt_, 2);
  }  
}

void LocalFootstepPlanner::computeFootPositions(const Eigen::MatrixXd &body_plan,
  const Eigen::MatrixXd &grf_plan, const std::vector<std::vector<bool>> &contact_schedule,
  const Eigen::MatrixXd &ref_body_plan, Eigen::MatrixXd &foot_positions) {

  // Loop through each foot
  for (int j=0; j<num_feet_; j++) {

    // Compute the number of timesteps corresponding to half the stance phase
    int half_duty_cycle = (period_*duty_cycles_[j])/2;

    // Loop through the horizon to identify instances of touchdown
    for (int i = 1; i < contact_schedule.size(); i++) {

      if (isNewContact(contact_schedule, i, j)) {
      
        // Declare foot position vectors
        Eigen::Vector3d foot_position, foot_position_grf,
          foot_position_nominal, hip_position_midstance, centrifugal, vel_tracking;

        // Declare body and grf vectors
        Eigen::Vector3d body_pos_midstance, body_rpy_midstance, 
          body_vel_touchdown, ref_body_vel_touchdown, body_ang_vel_touchdown, 
          ref_body_ang_vel_touchdown, grf_midstance;

        // Compute the horizon index of midstance
        int midstance = i + half_duty_cycle;

        // Compute body pose at midstance from either the trajectory or Raibert heuristic directly
        if (midstance < horizon_length_) {
          body_pos_midstance = body_plan.block<1,3>(midstance,0);
          body_rpy_midstance = body_plan.block<1,3>(midstance,3);
          grf_midstance = grf_plan.block<1,3>(midstance,3*j);
        } else {
          body_pos_midstance = body_plan.block<1,3>(horizon_length_-1,0) +
            body_plan.block<1,3>(horizon_length_-1,6)*(midstance-(horizon_length_-1))*dt_;
          body_rpy_midstance = body_plan.block<1,3>(horizon_length_-1,3);
          grf_midstance = grf_plan.block<1,3>(horizon_length_-1,3*j);
        }

        // Get touchdown information for body state
        body_vel_touchdown = body_plan.block<1,3>(i,6);
        ref_body_vel_touchdown = ref_body_plan.block<1,3>(i,6);
        body_ang_vel_touchdown = body_plan.block<1,3>(i,9);
        ref_body_ang_vel_touchdown = ref_body_plan.block<1,3>(i,9);

        // Compute nominal foot positions for kinematic and grf-projection measures
        quadKD_->worldToNominalHipFKWorldFrame(j, body_pos_midstance, body_rpy_midstance, 
            hip_position_midstance);
        grid_map::Position hip_position_grid_map = {hip_position_midstance.x(), hip_position_midstance.y()};
        double hip_height = hip_position_midstance.z() - terrain_grid_.atPosition(
          "z",hip_position_grid_map, grid_map::InterpolationMethods::INTER_NEAREST);
        centrifugal = (hip_height/9.81)*body_vel_touchdown.cross(ref_body_ang_vel_touchdown);
        vel_tracking = 0.03*(body_vel_touchdown - ref_body_vel_touchdown);
        // foot_position_grf = terrain_.projectToMap(hip_position_midstance, -1.0*grf_midstance);

        // Combine these measures to get the nominal foot position and grab correct height
        // foot_position_nominal = grf_weight_*foot_position_grf +
        //   (1-grf_weight_)*(hip_position_midstance + vel_tracking);
        foot_position_nominal = hip_position_midstance + centrifugal + vel_tracking;
        grid_map::Position foot_position_grid_map = {foot_position_nominal.x(), foot_position_nominal.y()};
        foot_position_nominal.z() = terrain_grid_.atPosition("z", foot_position_grid_map,
          grid_map::InterpolationMethods::INTER_NEAREST);

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
  int current_plan_index, const Eigen::MatrixXd &body_plan, const double &time_ahead, quad_msgs::MultiFootPlanDiscrete &past_footholds_msg,
  quad_msgs::MultiFootPlanDiscrete &future_footholds_msg,
  quad_msgs::MultiFootPlanContinuous &foot_plan_continuous_msg) {

  foot_plan_continuous_msg.states.resize(contact_schedule.size());
  future_footholds_msg.feet.resize(num_feet_);

  // Loop through each foot to construct the continuous foot plan message
  for (int j=0; j<num_feet_; j++) {

    future_footholds_msg.feet[j].header = future_footholds_msg.header;

    // Declare variables for computing initial swing foot state
    // Identify index for the liftoff and touchdown events
    quad_msgs::FootState most_recent_foothold_msg = past_footholds_msg.feet[j].footholds.back();
    
    int i_liftoff = most_recent_foothold_msg.traj_index - current_plan_index;
    int i_touchdown = getNextContactIndex(contact_schedule, 0, j);
    int swing_duration = i_touchdown - i_liftoff;

    // Identify positions of the previous and next footholds
    Eigen::Vector3d foot_position_prev;
    quad_utils::footStateMsgToEigen(most_recent_foothold_msg, foot_position_prev);
    Eigen::Vector3d foot_position_next = getFootData(foot_positions, i_touchdown, j);

    // Loop through the horizon
    for (int i = 0; i < contact_schedule.size(); i++) {

      // Update header for multi foot state if first time through
      if (j == 0) {
        foot_plan_continuous_msg.states[i].header = foot_plan_continuous_msg.header;
        foot_plan_continuous_msg.states[i].header.stamp = foot_plan_continuous_msg.header.stamp + 
          ros::Duration(i*dt_);
        foot_plan_continuous_msg.states[i].traj_index = current_plan_index + i;
      }

      // Create the foot state message
      quad_msgs::FootState foot_state_msg;
      foot_state_msg.header = foot_plan_continuous_msg.header;
      foot_state_msg.traj_index = foot_plan_continuous_msg.states[i].traj_index;

      Eigen::Vector3d foot_position;
      Eigen::Vector3d foot_velocity;
      Eigen::Vector3d foot_acceleration;

      // Determine the foot state at this index
      if (isContact(contact_schedule, i, j)) { // In contact

        // Log current foot position and zero velocity
        foot_position = getFootData(foot_positions, i, j);
        foot_velocity = Eigen::VectorXd::Zero(3);
        foot_acceleration = Eigen::VectorXd::Zero(3);
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
        double swing_phase;
        if (swing_duration == 0)
        {
          // Avoid division by zero when liftoff is the terminal state
          swing_phase = 0;
          swing_duration = 1;
        }
        else
        {
          // For the first step, it might be duplicated in the same plan index so we need to refine the phase based on the time duration to next plan index
          if (i == 0)
          {
            swing_phase = (i - i_liftoff + (dt_ - time_ahead) / dt_) / (double)swing_duration;
          }
          else
          {
            swing_phase = (i - i_liftoff) / (double)swing_duration;
          }
        }
        computeSwingFootState(foot_position_prev, foot_position_next, swing_phase, swing_duration, body_plan.row(i).transpose(), j,
          foot_position, foot_velocity, foot_acceleration);
        foot_state_msg.contact = false;

      }

      // Load state data into the message
      quad_utils::eigenToFootStateMsg(foot_position, foot_velocity, foot_acceleration, foot_state_msg);
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
