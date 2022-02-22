#include "local_planner/local_footstep_planner.h"

#include <tf/tf.h>

#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner() {}

void LocalFootstepPlanner::setTemporalParams(
    double dt, int period, int horizon_length,
    const std::vector<double> &duty_cycles,
    const std::vector<double> &phase_offsets) {
  dt_ = dt;
  period_ = period;
  horizon_length_ = horizon_length;
  duty_cycles_ = duty_cycles;
  phase_offsets_ = phase_offsets;

  nominal_contact_schedule_.resize(period_);

  for (int i = 0; i < period_; i++) {  // For each finite element
    nominal_contact_schedule_.at(i).resize(num_feet_);
    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) {  // For each leg
      // If this index in the horizon is between touchdown and liftoff, set
      // contact to true
      if ((i >= period_ * phase_offsets_[leg_idx] &&
           i < period_ * (phase_offsets_[leg_idx] + duty_cycles_[leg_idx])) ||
          i < period_ *
                  (phase_offsets_[leg_idx] + duty_cycles_[leg_idx] - 1.0)) {
        nominal_contact_schedule_.at(i).at(leg_idx) = true;
      } else {
        nominal_contact_schedule_.at(i).at(leg_idx) = false;
      }
    }
  }
}

void LocalFootstepPlanner::setSpatialParams(
    double ground_clearance, double hip_clearance, double grf_weight,
    double standing_error_threshold,
    std::shared_ptr<quad_utils::QuadKD> kinematics,
    double foothold_search_radius, double foothold_obj_threshold,
    std::string obj_fun_layer) {
  ground_clearance_ = ground_clearance;
  hip_clearance_ = hip_clearance;
  standing_error_threshold_ = standing_error_threshold;
  grf_weight_ = grf_weight;
  quadKD_ = kinematics;

  foothold_search_radius_ = foothold_search_radius;
  foothold_obj_threshold_ = foothold_obj_threshold;
  obj_fun_layer_ = obj_fun_layer;
}

void LocalFootstepPlanner::updateMap(const FastTerrainMap &terrain) {
  terrain_ = terrain;
}

void LocalFootstepPlanner::updateMap(const grid_map::GridMap &terrain) {
  terrain_grid_ = terrain;
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(
    const Eigen::VectorXd &body_plan,
    const Eigen::VectorXd &foot_positions_world,
    Eigen::VectorXd &foot_positions_body) {
  for (int i = 0; i < num_feet_; i++) {
    foot_positions_body.segment<3>(3 * i) =
        foot_positions_world.segment<3>(3 * i) - body_plan.segment<3>(0);
  }
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(
    const Eigen::MatrixXd &body_plan,
    const Eigen::MatrixXd &foot_positions_world,
    Eigen::MatrixXd &foot_positions_body) {
  Eigen::VectorXd foot_pos = Eigen::VectorXd::Zero(3 * num_feet_);
  for (int i = 0; i < horizon_length_; i++) {
    foot_pos.setZero();
    getFootPositionsBodyFrame(body_plan.row(i), foot_positions_world.row(i),
                              foot_pos);
    foot_positions_body.row(i) = foot_pos;
  }
}

void LocalFootstepPlanner::computeStanceContactSchedule(
    int current_plan_index, std::vector<std::vector<bool>> &contact_schedule) {
  for (int i = 0; i < horizon_length_; i++) {  // For each finite element
    contact_schedule.at(i).resize(num_feet_);
    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) {  // For each leg
      nominal_contact_schedule_.at(i).at(leg_idx) = true;
    }
  }
}

void LocalFootstepPlanner::computeContactSchedule(
    int current_plan_index, std::vector<std::vector<bool>> &contact_schedule) {
  // Compute the current phase in the nominal contact schedule
  int phase = current_plan_index % period_;

  // Compute the current contact schedule by looping through the nominal
  // starting at the phase
  contact_schedule.resize(horizon_length_);
  for (int i = 0; i < horizon_length_; i++) {
    contact_schedule[i] = nominal_contact_schedule_[(i + phase) % period_];
  }
}

void LocalFootstepPlanner::cubicHermiteSpline(double pos_prev, double vel_prev,
                                              double pos_next, double vel_next,
                                              double phase, double duration,
                                              double &pos, double &vel,
                                              double &acc) {
  // Sometimes phase will be slightly smaller than zero due to numerical issues
  phase = std::min(std::max(phase, 0.), 1.);

  double t = phase * duration;
  double t2 = t * t;
  double t3 = t * t * t;
  double duration2 = duration * duration;
  double duration3 = duration * duration * duration;

  pos = pos_prev + vel_prev * t +
        (t3 * (2 * pos_prev - 2 * pos_next + duration * vel_prev +
               duration * vel_next)) /
            duration3 -
        (t2 * (3 * pos_prev - 3 * pos_next + 2 * duration * vel_prev +
               duration * vel_next)) /
            duration2;
  vel = vel_prev +
        (3 * t2 *
         (2 * pos_prev - 2 * pos_next + duration * vel_prev +
          duration * vel_next)) /
            duration3 -
        (2 * t *
         (3 * pos_prev - 3 * pos_next + 2 * duration * vel_prev +
          duration * vel_next)) /
            duration2;
  acc = (6 * t *
         (2 * pos_prev - 2 * pos_next + duration * vel_prev +
          duration * vel_next)) /
            duration3 -
        (2 * (3 * pos_prev - 3 * pos_next + 2 * duration * vel_prev +
              duration * vel_next)) /
            duration2;
}

void LocalFootstepPlanner::computeFootPlan(
    int current_plan_index,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::MatrixXd &body_plan, const Eigen::MatrixXd &grf_plan,
    const Eigen::MatrixXd &ref_body_plan,
    const Eigen::VectorXd &foot_positions_current,
    const Eigen::VectorXd &foot_velocities_current,
    double first_element_duration,
    quad_msgs::MultiFootState &past_footholds_msg,
    Eigen::MatrixXd &foot_positions, Eigen::MatrixXd &foot_velocities,
    Eigen::MatrixXd &foot_accelerations) {
  // Loop through each foot to compute the new footholds
  for (int j = 0; j < num_feet_; j++) {
    // Compute the number of timesteps corresponding to half the stance phase
    int half_duty_cycle = (period_ * duty_cycles_[j]) / 2;

    // Loop through the horizon to identify instances of touchdown
    for (int i = 1; i < contact_schedule.size(); i++) {
      if (isNewContact(contact_schedule, i, j)) {
        // Declare foot position vectors
        Eigen::Vector3d foot_position, foot_position_grf, foot_position_nominal,
            hip_position_midstance, centrifugal, vel_tracking,
            foot_position_raibert;

        // Declare body and grf vectors
        Eigen::Vector3d body_pos_midstance, body_rpy_midstance,
            body_vel_touchdown, ref_body_vel_touchdown, body_ang_vel_touchdown,
            ref_body_ang_vel_touchdown, grf_midstance;

        // Compute the horizon index of midstance
        int midstance = i + half_duty_cycle;

        // Compute body pose at midstance from either the trajectory or Raibert
        // heuristic directly
        if (midstance < horizon_length_) {
          body_pos_midstance = body_plan.block<1, 3>(midstance, 0);
          body_rpy_midstance = body_plan.block<1, 3>(midstance, 3);
          grf_midstance = grf_plan.block<1, 3>(midstance, 3 * j);
        } else {
          Eigen::VectorXd body_plan_midstance =
              computeFutureState(midstance - (horizon_length_ - 1),
                                 body_plan.row(horizon_length_ - 1));
          body_pos_midstance = body_plan_midstance.segment(0, 3);
          body_rpy_midstance = body_plan_midstance.segment(3, 3);
          grf_midstance = grf_plan.block<1, 3>(horizon_length_ - 1, 3 * j);
        }

        // Get touchdown information for body state
        body_vel_touchdown = body_plan.block<1, 3>(i, 6);
        ref_body_vel_touchdown = ref_body_plan.block<1, 3>(i, 6);
        body_ang_vel_touchdown = body_plan.block<1, 3>(i, 9);
        ref_body_ang_vel_touchdown = ref_body_plan.block<1, 3>(i, 9);

        // Compute nominal foot positions for kinematic and grf-projection
        // measures
        quadKD_->worldToNominalHipFKWorldFrame(
            j, body_pos_midstance, body_rpy_midstance, hip_position_midstance);
        grid_map::Position hip_position_grid_map = {hip_position_midstance.x(),
                                                    hip_position_midstance.y()};

        double hip_height =
            hip_position_midstance.z() -
            terrain_grid_.atPosition(
                "z",
                terrain_grid_.getClosestPositionInMap(hip_position_grid_map),
                grid_map::InterpolationMethods::INTER_NEAREST);

        // Ref: Highly Dynamic Quadruped Locomotion via Whole-Body Impulse
        // Control and Model Predictive Control
        centrifugal = std::sqrt(hip_height / 9.81) / 2 *
                      body_vel_touchdown.cross(ref_body_ang_vel_touchdown);

        // Ref: MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped
        // Robot
        vel_tracking = std::sqrt(hip_height / 9.81) *
                       (body_vel_touchdown - ref_body_vel_touchdown);

        // foot_position_grf =
        //     terrain_.projectToMap(hip_position_midstance, -1.0 *
        //     grf_midstance);

        // Combine these measures to get the nominal foot position and grab
        // correct height
        foot_position_raibert =
            hip_position_midstance + centrifugal + vel_tracking;
        foot_position_nominal = foot_position_raibert;
        grid_map::Position foot_position_grid_map = {foot_position_nominal.x(),
                                                     foot_position_nominal.y()};

        if (!terrain_grid_.isInside(foot_position_grid_map)) {
          ROS_WARN(
              "Foot position is outside the map. Steer the robot in "
              "another direction");
          foot_positions = foot_positions_current;
          continue;
        }
        // Toe has 20cm radius so we need to shift the foot height from terrain
        foot_position_nominal.z() =
            terrain_grid_.atPosition(
                "z",
                terrain_grid_.getClosestPositionInMap(foot_position_grid_map),
                grid_map::InterpolationMethods::INTER_NEAREST) +
            toe_radius;

        // Optimize the foothold location to get the final position
        foot_position = getNearestValidFoothold(foot_position_nominal);

        // Store foot position in the Eigen matrix
        foot_positions.block<1, 3>(i, 3 * j) = foot_position;

      } else {
        // If this isn't a new contact just hold the previous position
        // Note: this should get ignored for a foot in flight
        foot_positions.block<1, 3>(i, 3 * j) =
            getFootData(foot_positions, i - 1, j);
      }
    }
  }

  // Loop through each foot to interpolate the swing foot states
  for (int j = 0; j < num_feet_; j++) {
    // Declare variables for computing initial swing foot state
    // Identify index for the liftoff and touchdown events
    quad_msgs::FootState most_recent_foothold_msg = past_footholds_msg.feet[j];

    int i_liftoff = most_recent_foothold_msg.traj_index - current_plan_index;
    int i_touchdown = getNextContactIndex(contact_schedule, 0, j);
    double swing_duration = i_touchdown - i_liftoff;

    // Identify positions of the previous and next footholds
    Eigen::Vector3d foot_position_prev, foot_position_prev_nominal;
    quad_utils::footStateMsgToEigen(most_recent_foothold_msg,
                                    foot_position_prev);
    quad_utils::footStateMsgToEigen(most_recent_foothold_msg,
                                    foot_position_prev_nominal);
    Eigen::Vector3d foot_velocity_prev;
    foot_velocity_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d foot_position_next =
        getFootData(foot_positions, i_touchdown, j);
    Eigen::Vector3d foot_velocity_next = Eigen::Vector3d::Zero();

    // Compute the midair index
    int midair = std::round(i_liftoff + swing_duration / 2);

    // Initialize variables
    Eigen::VectorXd body_plan_midair;
    double swing_apex;

    if (midair < 0) {
      // If the midair is something in previous, take it from the memory
      swing_apex = most_recent_foothold_msg.velocity.z;
    } else {
      // Otherwise, update it using the latest plan
      body_plan_midair = body_plan.row(midair);

      // Compute the swing apex
      swing_apex = computeSwingApex(
          j, body_plan_midair, foot_position_prev_nominal, foot_position_next);

      // Update the memory, we borrow the past_footholds_msg since its velocity
      // is unused
      past_footholds_msg.feet[j].velocity.z = swing_apex;
    }

    // Compute the acceleration maximum
    Eigen::Vector3d foot_acceleration_max = computeMaximumAcceleration(
        foot_position_prev_nominal, Eigen::Vector3d::Zero(), foot_position_next,
        Eigen::Vector3d::Zero(), swing_apex, swing_duration);

    // Loop through the horizon
    for (int i = 0; i < contact_schedule.size(); i++) {
      // Create the foot state message
      quad_msgs::FootState foot_state_msg;
      foot_state_msg.traj_index = current_plan_index + i;

      Eigen::Vector3d foot_position;
      Eigen::Vector3d foot_velocity;
      Eigen::Vector3d foot_acceleration;

      // Determine the foot state at this index
      if (!isContact(contact_schedule, i, j) ||
          (i != 0 && isNewContact(contact_schedule, i, j))) {
        // In swing (or new contact, we compute acceleration for interplation in
        // inverse dynamics)
        if (isNewLiftoff(contact_schedule, i, j)) {
          // Set the indices and data for liftoff
          i_liftoff = i;
          foot_position_prev = getFootData(foot_positions, i_liftoff, j);
          foot_position_prev_nominal =
              getFootData(foot_positions, i_liftoff, j);
          foot_velocity_prev = Eigen::Vector3d::Zero();

          // Set the indices and data for touchdown
          i_touchdown = getNextContactIndex(contact_schedule, i, j);

          // If the touchdown index would extend past the horizon, use heuristic
          // to compute it
          if (!isContact(contact_schedule, i_touchdown, j)) {
            int stance_duration = period_ * (duty_cycles_[j]);
            swing_duration = period_ - stance_duration;

            // Apply basic footstep heuristic, no searching
            Eigen::VectorXd body_plan_midstance = computeFutureState(
                (i_liftoff + swing_duration + 0.5 * stance_duration) -
                    (horizon_length_ - 1),
                body_plan.row(horizon_length_ - 1));
            Eigen::Vector3d body_pos_midstance =
                body_plan_midstance.segment(0, 3);
            Eigen::Vector3d body_rpy_midstance =
                body_plan_midstance.segment(3, 3);
            quadKD_->worldToNominalHipFKWorldFrame(
                j, body_pos_midstance, body_rpy_midstance, foot_position_next);

            // Toe has 20cm radius so we need to shift the foot height from
            // terrain
            foot_position_next.z() =
                terrain_grid_.atPosition(
                    "z",
                    terrain_grid_.getClosestPositionInMap(
                        foot_position_next.head<2>()),
                    grid_map::InterpolationMethods::INTER_NEAREST) +
                toe_radius;

          } else {
            foot_position_next = getFootData(foot_positions, i_touchdown, j);
            swing_duration = i_touchdown - i_liftoff;
          }

          // Compute the midair index
          midair = std::round(i_liftoff + swing_duration / 2);
          if (midair > horizon_length_ - 1) {
            // Apply basic footstep heuristic, no searching
            body_plan_midair = computeFutureState(
                (i_liftoff + swing_duration / 2) - (horizon_length_ - 1),
                body_plan.row(horizon_length_ - 1));
          } else {
            body_plan_midair = body_plan.row(midair);
          }

          // Compute the swing apex
          swing_apex =
              computeSwingApex(j, body_plan_midair, foot_position_prev_nominal,
                               foot_position_next);

          // Compute the acceleration maximum
          foot_acceleration_max = computeMaximumAcceleration(
              foot_position_prev_nominal, Eigen::Vector3d::Zero(),
              foot_position_next, Eigen::Vector3d::Zero(), swing_apex,
              swing_duration);
        } else if (i_liftoff < 0) {
          // If the leg is currently swinging, we interplate from the current
          // states
          foot_position_prev = foot_positions_current.segment(3 * j, 3);
          foot_velocity_prev = foot_velocities_current.segment(3 * j, 3);
        }

        // Compute the period index of plan and current states
        // For the first step, it might be duplicated in the same plan index so
        // we need to refine the phase based on the time duration to next plan
        // index
        double swing_idx =
            (i == 0) ? (i - i_liftoff + (dt_ - first_element_duration) / dt_)
                     : (i - i_liftoff);
        double swing_idx_current =
            0 - i_liftoff + (dt_ - first_element_duration) / dt_;

        // Define interpolate phase and duration
        double interp_phase;
        double interp_duration;

        if (i_liftoff < 0) {
          // If the leg is currently swinging, we interplate from the current
          // states
          interp_phase = (swing_idx - swing_idx_current) /
                         (swing_duration - swing_idx_current);
          interp_duration = (swing_duration - swing_idx_current) * dt_;
        } else {
          // Otherwise we just use the plan
          interp_phase = swing_idx / swing_duration;
          interp_duration = swing_duration * dt_;
        }

        // Interplate x and y
        cubicHermiteSpline(foot_position_prev.x(), foot_velocity_prev.x(),
                           foot_position_next.x(), foot_velocity_next.x(),
                           interp_phase, interp_duration, foot_position.x(),
                           foot_velocity.x(), foot_acceleration.x());
        cubicHermiteSpline(foot_position_prev.y(), foot_velocity_prev.y(),
                           foot_position_next.y(), foot_velocity_next.y(),
                           interp_phase, interp_duration, foot_position.y(),
                           foot_velocity.y(), foot_acceleration.y());

        // Interplate z
        if (i_liftoff < 0) {
          // Start from current
          if (swing_idx / swing_duration < 0.5) {
            // Swing upwards, current should also be upwards
            interp_phase = (swing_idx - swing_idx_current) /
                           (swing_duration / 2 - swing_idx_current);
            interp_duration = (swing_duration / 2 - swing_idx_current) * dt_;

            cubicHermiteSpline(foot_position_prev.z(), foot_velocity_prev.z(),
                               swing_apex, 0, interp_phase, interp_duration,
                               foot_position.z(), foot_velocity.z(),
                               foot_acceleration.z());
          } else {
            // Swing downwards
            if (swing_idx_current / swing_duration < 0.5) {
              // Current is upwards
              interp_phase =
                  (swing_idx - swing_duration / 2) / (swing_duration / 2);
              interp_duration = swing_duration / 2 * dt_;

              cubicHermiteSpline(swing_apex, 0, foot_position_next.z(),
                                 foot_velocity_next.z(), interp_phase,
                                 interp_duration, foot_position.z(),
                                 foot_velocity.z(), foot_acceleration.z());
            } else {
              // Current is downwards
              interp_phase = (swing_idx - swing_idx_current) /
                             (swing_duration - swing_idx_current);
              interp_duration = (swing_duration - swing_idx_current) * dt_;

              cubicHermiteSpline(foot_position_prev.z(), foot_velocity_prev.z(),
                                 foot_position_next.z(), foot_velocity_next.z(),
                                 interp_phase, interp_duration,
                                 foot_position.z(), foot_velocity.z(),
                                 foot_acceleration.z());
            }
          }
        } else {
          // Start from plan
          interp_phase = (2 * swing_idx >= swing_duration)
                             ? (2 * swing_idx / swing_duration - 1)
                             : (2 * swing_idx / swing_duration);
          interp_duration = swing_duration * dt_ / 2;

          if (swing_idx / swing_duration < 0.5) {
            // Swing upwards
            cubicHermiteSpline(foot_position_prev.z(), foot_velocity_prev.z(),
                               swing_apex, 0, interp_phase, interp_duration,
                               foot_position.z(), foot_velocity.z(),
                               foot_acceleration.z());
          } else {
            // Swing downwards
            cubicHermiteSpline(swing_apex, 0, foot_position_next.z(),
                               foot_velocity_next.z(), interp_phase,
                               interp_duration, foot_position.z(),
                               foot_velocity.z(), foot_acceleration.z());
          }
        }

        // Apply the acceleration bounds
        foot_acceleration.x() =
            std::min(std::max(foot_acceleration.x(),
                              -std::abs(foot_acceleration_max.x())),
                     std::abs(foot_acceleration_max.x()));
        foot_acceleration.y() =
            std::min(std::max(foot_acceleration.y(),
                              -std::abs(foot_acceleration_max.y())),
                     std::abs(foot_acceleration_max.y()));
        foot_acceleration.z() =
            std::min(std::max(foot_acceleration.z(),
                              -std::abs(foot_acceleration_max.z())),
                     std::abs(foot_acceleration_max.z()));

        foot_state_msg.contact = false;
      }

      if (isContact(contact_schedule, i, j)) {
        // In contact
        // Log current foot position and zero velocity
        foot_position = getFootData(foot_positions, i, j);
        foot_velocity = Eigen::VectorXd::Zero(3);
        if (!(i != 0 && isNewContact(contact_schedule, i, j))) {
          // If it's a new contact, keep the acceleration for swing ID
          // interplation It should not be used after contact since it will be
          // GRF instead
          foot_acceleration = Eigen::VectorXd::Zero(3);
        }

        foot_state_msg.contact = true;
      }

      // Load state data into the message
      quad_utils::eigenToFootStateMsg(foot_position, foot_velocity,
                                      foot_acceleration, foot_state_msg);
      foot_positions.block<1, 3>(i, 3 * j) = foot_position;
      foot_velocities.block<1, 3>(i, 3 * j) = foot_velocity;
      foot_accelerations.block<1, 3>(i, 3 * j) = foot_acceleration;

      // If this is the end of a contact, add to the past footholds message
      if (i < period_ * duty_cycles_[j] &&
          isNewLiftoff(contact_schedule, i, j)) {
        past_footholds_msg.feet[j] = foot_state_msg;
      }
    }
  }
}

void LocalFootstepPlanner::loadFootPlanMsgs(
    const std::vector<std::vector<bool>> &contact_schedule,
    int current_plan_index, double first_element_duration,
    const Eigen::MatrixXd &foot_positions,
    const Eigen::MatrixXd &foot_velocities,
    const Eigen::MatrixXd &foot_accelerations,
    quad_msgs::MultiFootPlanDiscrete &future_footholds_msg,
    quad_msgs::MultiFootPlanContinuous &foot_plan_continuous_msg) {
  foot_plan_continuous_msg.states.resize(contact_schedule.size());
  future_footholds_msg.feet.resize(num_feet_);

  // Loop through each foot to construct the continuous foot plan message
  for (int j = 0; j < num_feet_; j++) {
    future_footholds_msg.feet[j].header = future_footholds_msg.header;

    // Loop through the horizon
    for (int i = 0; i < contact_schedule.size(); i++) {
      // Update header for multi foot state if first time through
      if (j == 0) {
        foot_plan_continuous_msg.states[i].header =
            foot_plan_continuous_msg.header;
        if (i == 0) {
          foot_plan_continuous_msg.states[i].header.stamp =
              foot_plan_continuous_msg.header.stamp;
        } else {
          foot_plan_continuous_msg.states[i].header.stamp =
              foot_plan_continuous_msg.header.stamp +
              ros::Duration(first_element_duration) +
              ros::Duration((i - 1) * dt_);
        }
        foot_plan_continuous_msg.states[i].traj_index = current_plan_index + i;
      }

      // Create the foot state message
      quad_msgs::FootState foot_state_msg;
      foot_state_msg.header = foot_plan_continuous_msg.header;
      foot_state_msg.traj_index = foot_plan_continuous_msg.states[i].traj_index;
      quad_utils::eigenToFootStateMsg(foot_positions.block<1, 3>(i, 3 * j),
                                      foot_velocities.block<1, 3>(i, 3 * j),
                                      foot_accelerations.block<1, 3>(i, 3 * j),
                                      foot_state_msg);
      foot_state_msg.contact = isContact(contact_schedule, i, j);

      // If this is a touchdown event, add to the future footholds message
      if (isNewContact(contact_schedule, i, j)) {
        future_footholds_msg.feet[j].footholds.push_back(foot_state_msg);
      }

      // Add the message to the plan
      foot_plan_continuous_msg.states[i].feet.push_back(foot_state_msg);
    }
  }
}

Eigen::Vector3d LocalFootstepPlanner::getNearestValidFoothold(
    const Eigen::Vector3d &foot_position) {
  // Declare
  Eigen::Vector3d foot_position_valid = foot_position;
  grid_map::Position pos_center, pos_center_aligned, offset, pos_valid;

  // Compute the closest index to the nominal and find the offset
  pos_center = {foot_position.x(), foot_position.y()};
  grid_map::Index i;
  terrain_grid_.getIndex(pos_center, i);
  terrain_grid_.getPosition(i, pos_center_aligned);
  offset = pos_center - pos_center_aligned;

  // Spiral outwards from the nominal until we find a valid foothold
  for (grid_map::SpiralIterator iterator(terrain_grid_, pos_center_aligned,
                                         foothold_search_radius_);
       !iterator.isPastEnd(); ++iterator) {
    double obj = terrain_grid_.at(obj_fun_layer_, *iterator);
    if (obj > foothold_obj_threshold_) {
      // Add the offset back in and return this new foothold
      terrain_grid_.getPosition(*iterator, pos_valid);
      // Shouldn't we update it?
      pos_valid += offset;

      if (!terrain_grid_.isInside(pos_valid)) {
        continue;
      }

      foot_position_valid << pos_valid.x(), pos_valid.y(),
          terrain_grid_.atPosition(
              "z", pos_valid, grid_map::InterpolationMethods::INTER_LINEAR) +
              toe_radius;
      return foot_position_valid;
    }
  }

  // If no foothold is found in the radius, keep the nominal and issue a warning
  ROS_WARN_THROTTLE(
      0.1, "No valid foothold found in radius of nominal, returning nominal");
  return foot_position_valid;
}
