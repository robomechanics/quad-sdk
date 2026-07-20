#include "local_planner/local_footstep_planner.hpp"

#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner(rclcpp::Node::SharedPtr node)
    : node_(node) {}

void LocalFootstepPlanner::setTemporalParams(
    double dt, int period, int horizon_length,
    const std::vector<double>& duty_cycles,
    const std::vector<double>& phase_offsets) {
  dt_ = dt;
  period_ = period;
  horizon_length_ = horizon_length;
  duty_cycles_ = duty_cycles;
  phase_offsets_ = phase_offsets;

  nominal_contact_schedule_.resize(period_);

  // Build one gait-period contact table from duty cycles and phase offsets.
  for (int i = 0; i < period_; i++) {
    nominal_contact_schedule_.at(i).resize(num_feet_);
    for (int leg_idx = 0; leg_idx < num_feet_; leg_idx++) {
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
    std::shared_ptr<quad_utils::QuadKD2> kinematics,
    double foothold_search_radius, double foothold_obj_threshold,
    std::string obj_fun_layer, double toe_radius) {
  ground_clearance_ = ground_clearance;
  hip_clearance_ = hip_clearance;
  standing_error_threshold_ = standing_error_threshold;
  grf_weight_ = grf_weight;
  quadKD_ = kinematics;
  foothold_search_radius_ = foothold_search_radius;
  foothold_obj_threshold_ = foothold_obj_threshold;
  obj_fun_layer_ = obj_fun_layer;
  toe_radius_ = toe_radius;
}

void LocalFootstepPlanner::updateMap(const FastTerrainMap& terrain) {
  terrain_ = terrain;
}

void LocalFootstepPlanner::updateMap(const grid_map::GridMap& terrain) {
  terrain_grid_ = terrain;
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(
    const Eigen::VectorXd& body_plan,
    const Eigen::VectorXd& foot_positions_world,
    Eigen::VectorXd& foot_positions_body) {
  for (int i = 0; i < num_feet_; i++) {
    foot_positions_body.segment<3>(3 * i) =
        foot_positions_world.segment<3>(3 * i) - body_plan.segment<3>(0);
  }
}

void LocalFootstepPlanner::getFootPositionsBodyFrame(
    const Eigen::MatrixXd& body_plan,
    const Eigen::MatrixXd& foot_positions_world,
    Eigen::MatrixXd& foot_positions_body) {
  Eigen::VectorXd foot_pos = Eigen::VectorXd::Zero(3 * num_feet_);
  for (int i = 0; i < horizon_length_; i++) {
    foot_pos.setZero();
    getFootPositionsBodyFrame(body_plan.row(i), foot_positions_world.row(i),
                              foot_pos);
    foot_positions_body.row(i) = foot_pos;
  }
}

void LocalFootstepPlanner::computeContactSchedule(
    int current_plan_index, const Eigen::MatrixXd& body_plan,
    const Eigen::VectorXi& ref_primitive_plan, int control_mode,
    std::vector<std::vector<bool>>& contact_schedule) {
  int phase = current_plan_index % period_;

  // Tile the nominal gait from the current phase across the horizon.
  contact_schedule.resize(horizon_length_);
  for (int i = 0; i < horizon_length_; i++) {
    contact_schedule[i].resize(num_feet_);
    if (control_mode == LocalPlannerMode::STAND) {
      for (size_t j = 0; j < contact_schedule[i].size(); j++) {
        contact_schedule[i][j] = true;
      }
    } else {
      contact_schedule[i] = nominal_contact_schedule_[(i + phase) % period_];
    }
  }
  // Override nominal gait during leap, flight, and landing primitives.
  for (int i = 0; i < horizon_length_; i++) {
    if (ref_primitive_plan(i) == LEAP_STANCE) {
      int leading_leg_liftoff_idx = std::min(i, horizon_length_ - 1);

      if (ref_primitive_plan(leading_leg_liftoff_idx) == FLIGHT) {
        contact_schedule.at(i) = {false, true, false, true};
      } else {
        contact_schedule.at(i) = {true, true, true, true};
      }
    } else if (ref_primitive_plan(i) == FLIGHT) {
      std::fill(contact_schedule.at(i).begin(), contact_schedule.at(i).end(),
                false);
    } else if (ref_primitive_plan(i) == LAND_STANCE) {
      contact_schedule.at(i) = {true, true, true, true};
    }
  }
}

void LocalFootstepPlanner::cubicHermiteSpline(double pos_prev, double vel_prev,
                                              double pos_next, double vel_next,
                                              double phase, double duration,
                                              double& pos, double& vel,
                                              double& acc) {
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
    const std::vector<std::vector<bool>>& contact_schedule,
    const Eigen::MatrixXd& body_plan, const Eigen::MatrixXd& grf_plan,
    const Eigen::MatrixXd& ref_body_plan,
    const Eigen::VectorXd& foot_positions_current,
    const Eigen::VectorXd& foot_velocities_current,
    double first_element_duration,
    quad_msgs::msg::MultiFootState& past_footholds_msg,
    Eigen::MatrixXd& foot_positions, Eigen::MatrixXd& foot_velocities,
    Eigen::MatrixXd& foot_accelerations) {
  // Place new footholds at touchdown events.
  for (int j = 0; j < num_feet_; j++) {
    for (size_t i = 1; i < contact_schedule.size(); i++) {
      if (isNewContact(contact_schedule, i, j)) {
        Eigen::Vector3d foot_position, foot_position_grf, foot_position_nominal,
            hip_position_midstance, centrifugal, vel_tracking,
            foot_position_raibert;

        Eigen::Vector3d body_pos_midstance, body_rpy_midstance,
            body_vel_touchdown, ref_body_vel_touchdown, body_ang_vel_touchdown,
            ref_body_ang_vel_touchdown, grf_midstance;

        int end_of_stance = getNextLiftoffIndex(contact_schedule, i, j);

        // Extend body motion if the upcoming stance runs past the horizon.
        Eigen::MatrixXd body_plan_stance;
        if (isContact(contact_schedule, end_of_stance, j)) {
          end_of_stance =
              std::max(static_cast<int>(i) + int(period_ * duty_cycles_[j]),
                       end_of_stance);

          body_plan_stance = Eigen::MatrixXd(end_of_stance + 1, 12);
          body_plan_stance.topRows(horizon_length_) = body_plan;
          for (size_t k = horizon_length_; k < end_of_stance; k++) {
            body_plan_stance.row(k) = computeFutureBodyPlan(
                k - (horizon_length_ - 1), body_plan.row(horizon_length_ - 1));
          }
        } else {
          body_plan_stance = body_plan;
        }

        // Center the foothold under the hip path during the stance window.
        std::vector<Eigen::Vector2d> P, R;
        for (size_t k = i; k < end_of_stance; k++) {
          quadKD_->worldToNominalHipFKWorldFrame(
              j, body_plan_stance.row(k).segment(0, 3),
              body_plan_stance.row(k).segment(3, 3), hip_position_midstance);

          Eigen::Vector2d p;
          p << hip_position_midstance.x(), hip_position_midstance.y();

          bool duplicate = false;
          for (size_t l = 0; l < P.size(); l++) {
            if ((P.at(l) - p).norm() < 1e-3) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) {
            P.push_back(p);
          }
        }

        hip_position_midstance = welzlMinimumCircle(P, R);

        body_vel_touchdown = body_plan.block<1, 3>(i, 6);
        ref_body_vel_touchdown = ref_body_plan.block<1, 3>(i, 6);
        body_ang_vel_touchdown = body_plan.block<1, 3>(i, 9);
        ref_body_ang_vel_touchdown = ref_body_plan.block<1, 3>(i, 9);

        double body_height_touchdown =
            std::max(body_plan(i, 2) -
                         terrain_grid_.atPosition(
                             "z_inpainted", body_plan.row(i).segment<2>(0)),
                     0.0);
        // Dynamic offsets combine centrifugal compensation and capture point.
        centrifugal = body_height_touchdown / 9.81 *
                      body_vel_touchdown.cross(ref_body_ang_vel_touchdown);
        vel_tracking = std::sqrt(body_height_touchdown / 9.81) *
                       (body_vel_touchdown - ref_body_vel_touchdown);

        // foot_position_grf =
        //     terrain_.projectToMap(hip_position_midstance, -1.0 *
        //     grf_midstance);

        foot_position_raibert =
            hip_position_midstance + centrifugal + vel_tracking;
        foot_position_nominal = foot_position_raibert;
        grid_map::Position foot_position_grid_map = {foot_position_nominal.x(),
                                                     foot_position_nominal.y()};

        if (!terrain_grid_.isInside(foot_position_grid_map)) {
          RCLCPP_WARN(node_->get_logger(),
                      "Foot position is outside the map. Steer the robot in "
                      "another direction");
          continue;
        }
        // Raise foothold by toe radius so the toe surface touches terrain.
        foot_position_nominal.z() =
            terrain_grid_.atPosition(
                "z_inpainted",
                terrain_grid_.getClosestPositionInMap(foot_position_grid_map),
                grid_map::InterpolationMethods::INTER_NEAREST) +
            toe_radius_;

        Eigen::Vector3d foot_position_previous =
            foot_positions.block<1, 3>(i, 3 * j);
        foot_position = getNearestValidFoothold(foot_position_nominal,
                                                foot_position_previous);

        foot_positions.block<1, 3>(i, 3 * j) = foot_position;

      } else {
        // Non-touchdown samples inherit the previous foothold.
        foot_positions.block<1, 3>(i, 3 * j) =
            getFootData(foot_positions, i - 1, j);
      }
    }
  }

  // Interpolate swing trajectories between past and planned footholds.
  for (int j = 0; j < num_feet_; j++) {
    quad_msgs::msg::FootState most_recent_foothold_msg =
        past_footholds_msg.feet[j];

    int i_liftoff = most_recent_foothold_msg.traj_index - current_plan_index;
    int i_touchdown = getNextContactIndex(contact_schedule, 0, j);
    double swing_duration = i_touchdown - i_liftoff;

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

    int mid_air = std::round(i_liftoff + swing_duration / 2);

    Eigen::VectorXd body_plan_mid_air;
    double swing_apex;

    if (mid_air < 0) {
      // Reuse the stored apex when the swing started before this horizon.
      swing_apex = most_recent_foothold_msg.velocity.z;
    } else {
      body_plan_mid_air = body_plan.row(mid_air);

      swing_apex = computeSwingApex(
          j, body_plan_mid_air, foot_position_prev_nominal, foot_position_next);

      // Store apex in velocity.z; this message field is otherwise unused here.
      past_footholds_msg.feet[j].velocity.z = swing_apex;
    }

    for (int i = 0; i < contact_schedule.size(); i++) {
      quad_msgs::msg::FootState foot_state_msg;
      foot_state_msg.traj_index = current_plan_index + i;

      Eigen::Vector3d foot_position;
      Eigen::Vector3d foot_velocity;
      Eigen::Vector3d foot_acceleration;

      if (!isContact(contact_schedule, i, j) ||
          (i != 0 && isNewContact(contact_schedule, i, j))) {
        // New contact still uses swing acceleration for inverse dynamics.
        if (isNewLiftoff(contact_schedule, i, j)) {
          i_liftoff = i;
          foot_position_prev = getFootData(foot_positions, i_liftoff, j);
          foot_position_prev_nominal =
              getFootData(foot_positions, i_liftoff, j);
          foot_velocity_prev = Eigen::Vector3d::Zero();

          i_touchdown = getNextContactIndex(contact_schedule, i, j);

          // Predict touchdown heuristically when it falls beyond this horizon.
          if (!isContact(contact_schedule, i_touchdown, j)) {
            int stance_duration = period_ * (duty_cycles_[j]);
            swing_duration = period_ - stance_duration;

            Eigen::VectorXd body_plan_midstance = computeFutureBodyPlan(
                (i_liftoff + swing_duration + 0.5 * stance_duration) -
                    (horizon_length_ - 1),
                body_plan.row(horizon_length_ - 1));
            Eigen::Vector3d body_pos_midstance =
                body_plan_midstance.segment(0, 3);
            Eigen::Vector3d body_rpy_midstance =
                body_plan_midstance.segment(3, 3);
            quadKD_->worldToNominalHipFKWorldFrame(
                j, body_pos_midstance, body_rpy_midstance, foot_position_next);

            grid_map::Position foot_position_next_grid_map =
                foot_position_next.head(2);
            if (!terrain_grid_.isInside(foot_position_next_grid_map)) {
              RCLCPP_WARN(
                  node_->get_logger(),
                  "computeFootPlan prediction receives a position out of "
                  "range, pick the previous position in map!");
              foot_position_next_grid_map = foot_position_prev.head(2);
              foot_position_next = foot_position_prev.head(2);
            }
            foot_position_next.z() =
                terrain_grid_.atPosition(
                    "z_inpainted", foot_position_next_grid_map,
                    grid_map::InterpolationMethods::INTER_NEAREST) +
                toe_radius_;
          } else {
            foot_position_next = getFootData(foot_positions, i_touchdown, j);
            swing_duration = i_touchdown - i_liftoff;
          }

          mid_air = std::round(i_liftoff + swing_duration / 2);
          if (mid_air > horizon_length_ - 1) {
            body_plan_mid_air = computeFutureBodyPlan(
                (i_liftoff + swing_duration / 2) - (horizon_length_ - 1),
                body_plan.row(horizon_length_ - 1));
          } else {
            body_plan_mid_air = body_plan.row(mid_air);
          }

          swing_apex =
              computeSwingApex(j, body_plan_mid_air, foot_position_prev_nominal,
                               foot_position_next);
        }

        // First sample may be partway through a timestep; adjust swing phase.
        double swing_idx =
            (i == 0) ? (i - i_liftoff + (dt_ - first_element_duration) / dt_)
                     : (i - i_liftoff);
        double swing_idx_current =
            0 - i_liftoff + (dt_ - first_element_duration) / dt_;

        double interp_phase;
        double interp_duration;

        interp_phase = swing_idx / swing_duration;
        interp_duration = swing_duration * dt_;

        // Interpolate horizontal swing with endpoint velocities fixed.
        cubicHermiteSpline(foot_position_prev.x(), foot_velocity_prev.x(),
                           foot_position_next.x(), foot_velocity_next.x(),
                           interp_phase, interp_duration, foot_position.x(),
                           foot_velocity.x(), foot_acceleration.x());
        cubicHermiteSpline(foot_position_prev.y(), foot_velocity_prev.y(),
                           foot_position_next.y(), foot_velocity_next.y(),
                           interp_phase, interp_duration, foot_position.y(),
                           foot_velocity.y(), foot_acceleration.y());

        // Split vertical swing into up/down splines through the apex.
        interp_phase = (2 * swing_idx >= swing_duration)
                           ? (2 * swing_idx / swing_duration - 1)
                           : (2 * swing_idx / swing_duration);
        interp_duration = swing_duration * dt_ / 2;

        if (swing_idx / swing_duration < 0.5) {
          cubicHermiteSpline(foot_position_prev.z(), foot_velocity_prev.z(),
                             swing_apex, 0, interp_phase, interp_duration,
                             foot_position.z(), foot_velocity.z(),
                             foot_acceleration.z());
        } else {
          cubicHermiteSpline(swing_apex, 0, foot_position_next.z(),
                             foot_velocity_next.z(), interp_phase,
                             interp_duration, foot_position.z(),
                             foot_velocity.z(), foot_acceleration.z());
        }

        foot_state_msg.contact = false;
      }

      if (isContact(contact_schedule, i, j)) {
        foot_position = getFootData(foot_positions, i, j);
        foot_velocity = Eigen::VectorXd::Zero(3);
        if (!(i != 0 && isNewContact(contact_schedule, i, j))) {
          // Preserve touchdown acceleration only at the contact transition.
          foot_acceleration = Eigen::VectorXd::Zero(3);
        }

        foot_state_msg.contact = true;
      }

      quad_utils::eigenToFootStateMsg(foot_position, foot_velocity,
                                      foot_acceleration, foot_state_msg);
      foot_positions.block<1, 3>(i, 3 * j) = foot_position;
      foot_velocities.block<1, 3>(i, 3 * j) = foot_velocity;
      foot_accelerations.block<1, 3>(i, 3 * j) = foot_acceleration;

      // Cache liftoff state for the next horizon's swing interpolation.
      if (i < period_ * duty_cycles_[j] &&
          isNewLiftoff(contact_schedule, i, j)) {
        past_footholds_msg.feet[j] = foot_state_msg;
      }
    }
  }
}

void LocalFootstepPlanner::loadFootPlanMsgs(
    const std::vector<std::vector<bool>>& contact_schedule,
    int current_plan_index, double first_element_duration,
    const Eigen::MatrixXd& foot_positions,
    const Eigen::MatrixXd& foot_velocities,
    const Eigen::MatrixXd& foot_accelerations,
    quad_msgs::msg::MultiFootPlanDiscrete& future_footholds_msg,
    quad_msgs::msg::MultiFootPlanContinuous& foot_plan_continuous_msg) {
  foot_plan_continuous_msg.states.resize(contact_schedule.size());
  future_footholds_msg.feet.resize(num_feet_);

  // Build continuous trajectory samples and discrete touchdown messages.
  for (int j = 0; j < num_feet_; j++) {
    future_footholds_msg.feet[j].header = future_footholds_msg.header;

    for (int i = 0; i < contact_schedule.size(); i++) {
      // Stamp each horizon sample once, independent of foot index.
      if (j == 0) {
        foot_plan_continuous_msg.states[i].header =
            foot_plan_continuous_msg.header;
        if (i == 0) {
          foot_plan_continuous_msg.states[i].header.stamp =
              foot_plan_continuous_msg.header.stamp;
        } else {
          foot_plan_continuous_msg.states[i].header.stamp =
              foot_plan_continuous_msg.header.stamp +
              rclcpp::Duration::from_seconds(first_element_duration) +
              rclcpp::Duration::from_seconds((i - 1) * dt_);
        }
        foot_plan_continuous_msg.states[i].traj_index = current_plan_index + i;
      }

      quad_msgs::msg::FootState foot_state_msg;
      foot_state_msg.header = foot_plan_continuous_msg.header;
      foot_state_msg.traj_index = foot_plan_continuous_msg.states[i].traj_index;
      quad_utils::eigenToFootStateMsg(foot_positions.block<1, 3>(i, 3 * j),
                                      foot_velocities.block<1, 3>(i, 3 * j),
                                      foot_accelerations.block<1, 3>(i, 3 * j),
                                      foot_state_msg);
      foot_state_msg.contact = isContact(contact_schedule, i, j);

      if (isNewContact(contact_schedule, i, j)) {
        future_footholds_msg.feet[j].footholds.push_back(foot_state_msg);
      }

      foot_plan_continuous_msg.states[i].feet.push_back(foot_state_msg);
    }
  }
}

Eigen::Vector3d LocalFootstepPlanner::getNearestValidFoothold(
    const Eigen::Vector3d& foot_position,
    const Eigen::Vector3d& foot_position_prev_solve) const {
  Eigen::Vector3d foot_position_best = foot_position;
  grid_map::Position pos_center, pos_center_aligned, offset, pos_valid;

  // Preserve sub-cell offset while scanning nearby grid cells.
  pos_center = foot_position.head<2>();
  grid_map::Index idx;
  terrain_grid_.getIndex(pos_center, idx);
  terrain_grid_.getPosition(idx, pos_center_aligned);
  offset = pos_center - pos_center_aligned;
  double best_kin_cost = std::numeric_limits<double>::max();

  // Spiral outward from the nominal foothold and keep the best valid cell.
  for (grid_map::SpiralIterator iterator(terrain_grid_, pos_center_aligned,
                                         foothold_search_radius_);
       !iterator.isPastEnd(); ++iterator) {
    terrain_grid_.getPosition(*iterator, pos_valid);
    pos_valid += offset;

    if (!terrain_grid_.isInside(pos_valid)) {
      continue;
    }

    double traversability = terrain_grid_.atPosition(obj_fun_layer_, pos_valid);
    double kin_cost =
        (pos_valid - foot_position.head<2>()).norm() +
        0.5 * (pos_valid - foot_position_prev_solve.head<2>()).norm();

    if (traversability > foothold_obj_threshold_ &&
        (kin_cost < best_kin_cost)) {
      foot_position_best.head<2>() = pos_valid;
      best_kin_cost = kin_cost;
    }
  }

  if (best_kin_cost == std::numeric_limits<double>::max()) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(),
        static_cast<rcutils_duration_value_t>(1e9),
        "No valid foothold found in radius of nominal, returning nominal");
  }
  foot_position_best.z() =
      terrain_grid_.atPosition("z_inpainted", foot_position_best.head<2>(),
                               grid_map::InterpolationMethods::INTER_LINEAR) +
      toe_radius_;
  return foot_position_best;
}

Eigen::Vector3d LocalFootstepPlanner::welzlMinimumCircle(
    std::vector<Eigen::Vector2d> P, std::vector<Eigen::Vector2d> R) {
  if (R.size() == 3) {
    // Base case: circle through three boundary points.
    Eigen::Vector3d D;
    D << (0.5000 * (R.at(0).x() * R.at(0).x() * R.at(1).y() -
                    R.at(0).x() * R.at(0).x() * R.at(2).y() -
                    R.at(1).x() * R.at(1).x() * R.at(0).y() +
                    R.at(1).x() * R.at(1).x() * R.at(2).y() +
                    R.at(2).x() * R.at(2).x() * R.at(0).y() -
                    R.at(2).x() * R.at(2).x() * R.at(1).y() +
                    R.at(0).y() * R.at(0).y() * R.at(1).y() -
                    R.at(0).y() * R.at(0).y() * R.at(2).y() -
                    R.at(0).y() * R.at(1).y() * R.at(1).y() +
                    R.at(0).y() * R.at(2).y() * R.at(2).y() +
                    R.at(1).y() * R.at(1).y() * R.at(2).y() -
                    R.at(1).y() * R.at(2).y() * R.at(2).y())) /
             (R.at(0).x() * R.at(1).y() - R.at(1).x() * R.at(0).y() -
              R.at(0).x() * R.at(2).y() + R.at(2).x() * R.at(0).y() +
              R.at(1).x() * R.at(2).y() - R.at(2).x() * R.at(1).y()),
        (0.5000 * (-R.at(0).x() * R.at(0).x() * R.at(1).x() +
                   R.at(0).x() * R.at(0).x() * R.at(2).x() +
                   R.at(0).x() * R.at(1).x() * R.at(1).x() -
                   R.at(0).x() * R.at(2).x() * R.at(2).x() +
                   R.at(0).x() * R.at(1).y() * R.at(1).y() -
                   R.at(0).x() * R.at(2).y() * R.at(2).y() -
                   R.at(1).x() * R.at(1).x() * R.at(2).x() +
                   R.at(1).x() * R.at(2).x() * R.at(2).x() -
                   R.at(1).x() * R.at(0).y() * R.at(0).y() +
                   R.at(1).x() * R.at(2).y() * R.at(2).y() +
                   R.at(2).x() * R.at(0).y() * R.at(0).y() -
                   R.at(2).x() * R.at(1).y() * R.at(1).y())) /
            (R.at(0).x() * R.at(1).y() - R.at(1).x() * R.at(0).y() -
             R.at(0).x() * R.at(2).y() + R.at(2).x() * R.at(0).y() +
             R.at(1).x() * R.at(2).y() - R.at(2).x() * R.at(1).y()),
        (0.5000 *
         std::sqrt((R.at(0).x() * R.at(0).x() - 2 * R.at(0).x() * R.at(1).x() +
                    R.at(1).x() * R.at(1).x() + R.at(0).y() * R.at(0).y() -
                    2 * R.at(0).y() * R.at(1).y() + R.at(1).y() * R.at(1).y()) *
                   (R.at(0).x() * R.at(0).x() - 2 * R.at(0).x() * R.at(2).x() +
                    R.at(2).x() * R.at(2).x() + R.at(0).y() * R.at(0).y() -
                    2 * R.at(0).y() * R.at(2).y() + R.at(2).y() * R.at(2).y()) *
                   (R.at(1).x() * R.at(1).x() - 2 * R.at(1).x() * R.at(2).x() +
                    R.at(2).x() * R.at(2).x() + R.at(1).y() * R.at(1).y() -
                    2 * R.at(1).y() * R.at(2).y() +
                    R.at(2).y() * R.at(2).y()))) /
            std::abs(R.at(0).x() * R.at(1).y() - R.at(1).x() * R.at(0).y() -
                     R.at(0).x() * R.at(2).y() + R.at(2).x() * R.at(0).y() +
                     R.at(1).x() * R.at(2).y() - R.at(2).x() * R.at(1).y());

    return D;
  }

  if (P.empty()) {
    // Base cases for zero, one, or two boundary points.
    Eigen::Vector3d D;
    switch (R.size()) {
      case 0:
        D << 0, 0, 0;
        break;
      case 1:
        D << R.at(0).x(), R.at(0).y(), 0;
        break;
      case 2:
        D << (R.at(0).x() + R.at(1).x()) / 2, (R.at(0).y() + R.at(1).y()) / 2,
            (R.at(0) - R.at(1)).norm() / 2;
        break;

      default:
        D << 0, 0, 0;
        break;
    }

    return D;
  }

  Eigen::Vector2d p = P.back();
  P.pop_back();
  Eigen::Vector3d D = welzlMinimumCircle(P, R);
  if ((p - D.segment(0, 2)).norm() < D.z()) {
    return D;
  }

  R.push_back(p);
  return welzlMinimumCircle(P, R);
}

double LocalFootstepPlanner::computeSwingApex(
    int leg_idx, const Eigen::VectorXd& body_plan,
    const Eigen::Vector3d& foot_position_prev,
    const Eigen::Vector3d& foot_position_next) {
  Eigen::Matrix4d g_world_legbase;
  quadKD_->worldToLegbaseFKWorldFrame(leg_idx, body_plan.segment(0, 3),
                                      body_plan.segment(3, 3), g_world_legbase);
  double hip_height = g_world_legbase(2, 3);
  double max_extension = 0.35;

  // Keep swing high enough for terrain, but below hip clearance limits.
  double swing_apex =
      std::min(ground_clearance_ - toe_radius_ +
                   std::max(foot_position_prev.z(), foot_position_next.z()),
               hip_height - hip_clearance_);
  swing_apex = std::max(swing_apex, hip_height - max_extension);

  return swing_apex;
}
