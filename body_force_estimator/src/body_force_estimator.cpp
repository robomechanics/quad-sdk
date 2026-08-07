#include "body_force_estimator/body_force_estimator.hpp"

#include "body_force_estimator/body_force_estimator_dynamics.hpp"

using namespace force_estimation_dynamics;

int joint_inds[12] = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

// Effective toe force estimate
double f_toe_MO[12];
// Planned liftoff time
double t_up[4];

BodyForceEstimator::BodyForceEstimator(rclcpp::Node::SharedPtr node)
    : node_(node) {
  // Load rosparams from parameter server
  std::string robot_state_topic, body_force_topic, toe_force_topic,
      local_plan_topic;
  quad_utils::loadROSParam(node_, "topics.state.ground_truth",
                           robot_state_topic);
  quad_utils::loadROSParam(node_, "topics.local_plan", local_plan_topic);
  quad_utils::loadROSParam(node_, "topics.body_force.joint_torques",
                           body_force_topic);
  quad_utils::loadROSParam(node_, "topics.body_force.toe_forces",
                           toe_force_topic);
  quad_utils::loadROSParamDefault(node_, "body_force_estimator.update_rate",
                                  update_rate_, 250.0);
  quad_utils::loadROSParamDefault(node_, "body_force_estimator.K_O", K_O_,
                                  50.0);
  quad_utils::loadROSParamDefault(node_, "body_force_estimator.cancel_friction",
                                  cancel_friction_, 1);

  // Select which robot's dynamics implementation the runtime pointers point at.
  // Recognized values: "spirit" (default) and "go2". Anything else falls back
  // to spirit. See body_force_estimator_dynamics.hpp for the dispatch details.
  std::string robot_type;
  quad_utils::loadROSParamDefault(node_, "robot_type", robot_type,
                                  std::string("spirit"));
  force_estimation_dynamics::loadRobot(robot_type);
  RCLCPP_INFO(node_->get_logger(),
              "body_force_estimator: robot_type='%s' → using %s dynamics",
              robot_type.c_str(),
              (robot_type == "go2") ? "Go2" : "Spirit40");

  // Optional per-launch override of effort_sign[] — needed to switch between
  // Gazebo (whose estimator_plugin.cpp hardcodes -torque_msg.y() on the hip
  // channel, so Go2 needs effort_sign={1,-1,1} to undo) and real Unitree
  // hardware (unitree_interface passes tau_est() through directly, so
  // Go2 needs effort_sign={1,1,1}). If the yaml array size != 3, the impl
  // default (Gazebo-safe {1,-1,1} for Go2, {1,1,1} for Spirit) stays.
  std::vector<double> effort_sign_default = {
      force_estimation_dynamics::effort_sign[0],
      force_estimation_dynamics::effort_sign[1],
      force_estimation_dynamics::effort_sign[2]};
  std::vector<double> effort_sign_override;
  quad_utils::loadROSParamDefault(node_, "body_force_estimator.effort_sign",
                                  effort_sign_override, effort_sign_default);
  if (effort_sign_override.size() == 3) {
    force_estimation_dynamics::effort_sign[0] = effort_sign_override[0];
    force_estimation_dynamics::effort_sign[1] = effort_sign_override[1];
    force_estimation_dynamics::effort_sign[2] = effort_sign_override[2];
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "body_force_estimator.effort_sign has %zu elements, expected "
                "3 (abad,hip,knee). Falling back to impl default.",
                effort_sign_override.size());
  }
  RCLCPP_INFO(node_->get_logger(),
              "body_force_estimator: active effort_sign = [%+.1f, %+.1f, %+.1f]",
              force_estimation_dynamics::effort_sign[0],
              force_estimation_dynamics::effort_sign[1],
              force_estimation_dynamics::effort_sign[2]);

  // Setup pubs and subs
  robot_state_sub_ = node_->create_subscription<quad_msgs::msg::RobotState>(
      robot_state_topic, rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&BodyForceEstimator::robotStateCallback, this,
                std::placeholders::_1));
  local_plan_sub_ = node_->create_subscription<quad_msgs::msg::RobotPlan>(
      local_plan_topic, 10,
      std::bind(&BodyForceEstimator::localPlanCallback, this,
                std::placeholders::_1));
  body_force_pub_ = node_->create_publisher<quad_msgs::msg::BodyForceEstimate>(
      body_force_topic, 10);
  toe_force_pub_ =
      node_->create_publisher<quad_msgs::msg::GRFArray>(toe_force_topic, 10);
}

void BodyForceEstimator::robotStateCallback(
    const quad_msgs::msg::RobotState::SharedPtr msg) {
  last_state_msg_ = msg;
}

void BodyForceEstimator::localPlanCallback(
    const quad_msgs::msg::RobotPlan::SharedPtr msg) {
  last_local_plan_msg_ = msg;
}

void BodyForceEstimator::update() {
  Eigen::Matrix3d M;
  Eigen::Vector3d beta;
  Eigen::Matrix3d J_MO;  // Jacobian

  Eigen::Vector3d q;       // joint positions
  Eigen::Vector3d qd;      // joint velocities
  Eigen::Vector3d tau;     // joint actuator torques
  Eigen::Vector3d re;      // momentum observer external torque estimates
  Eigen::Vector3d pe;      // momentum observer momentum estimate
  Eigen::Vector3d fe_toe;  // effective toe forces

  // Momentum observer gain (todo: this should be precomputed once)
  Eigen::Matrix3d K_O;
  K_O << K_O_, 0, 0, 0, K_O_, 0, 0, 0, K_O_;

  // Wire→URDF preprocessing pulled from the loaded robot dynamics
  // (joint_scale / joint_offset). See body_force_estimator_dynamics.hpp.
  // Spirit40 uses {1,-1,1}/{0,0,0} — matches the previous hardcoded
  // joint_dirs exactly. Go2 uses {1,-1,1}/{0,pi/2,pi} to add the origin
  // offsets that the wire encoders carry (see quad_utils/config/go2.yaml).

  if (last_state_msg_ == NULL || last_local_plan_msg_ == NULL) {
    return;
  }

  // Interpolate the local plan to get the reference state
  quad_msgs::msg::RobotState ref_state_msg;
  rclcpp::Time t_first_state(last_local_plan_msg_->states.front().header.stamp);
  double t_now =
      (node_->now() - rclcpp::Time(last_local_plan_msg_->state_timestamp))
          .seconds();  // Use time of state - RECOMMENDED
  if ((t_now <
       (rclcpp::Time(last_local_plan_msg_->states.front().header.stamp) -
        t_first_state)
           .seconds()) ||
      (t_now > (rclcpp::Time(last_local_plan_msg_->states.back().header.stamp) -
                t_first_state)
                   .seconds())) {
    // Plan timestamps don't bracket the current time (commonly: the
    // local_planner has not published a fresh LocalPlan recently). The
    // interpolation loop below will leave ref_state_msg default-
    // constructed, and the per-foot loop further down indexes
    // ref_state_msg.feet.feet[i] — which crashes with SIGSEGV when the
    // vector is empty. Bail out instead and wait for the next callback
    // to bring a usable plan.
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "ID node couldn't find the correct ref state!");
    return;
  }
  for (size_t i = 0; i < last_local_plan_msg_->states.size() - 1; i++) {
    if ((t_now >= (rclcpp::Time(last_local_plan_msg_->states[i].header.stamp) -
                   t_first_state)
                      .seconds()) &&
        (t_now <
         (rclcpp::Time(last_local_plan_msg_->states[i + 1].header.stamp) -
          t_first_state)
             .seconds())) {
      double t_interp =
          (t_now - (rclcpp::Time(last_local_plan_msg_->states[i].header.stamp) -
                    t_first_state)
                       .seconds()) /
          (rclcpp::Time(last_local_plan_msg_->states[i + 1].header.stamp)
               .seconds() -
           rclcpp::Time(last_local_plan_msg_->states[i].header.stamp)
               .seconds());

      // Linearly interpolate between states
      quad_utils::interpRobotState(last_local_plan_msg_->states[i],
                                   last_local_plan_msg_->states[i + 1],
                                   t_interp, ref_state_msg);
      continue;
    }
  }

  // Defense in depth: even with the early-return above, refuse to deref a
  // ref_state_msg whose feet vector wasn't populated (eg. an upstream that
  // publishes a malformed RobotState).
  if (ref_state_msg.feet.feet.size() < 4) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "ref_state_msg.feet.feet has %zu entries, expected 4; skipping update",
        ref_state_msg.feet.feet.size());
    return;
  }

  if (past_feet_state_.feet.empty()) {
    past_feet_state_ = ref_state_msg.feet;
  }

  for (int i = 0; i < 4; i++) {
    // Loop over four legs: FL, BL, FR, BR

    // Zero the torque estimates when the foot first lifts off the ground
    if (ref_state_msg.feet.feet[i].contact) {
      // Record the time that the foot was last in stance
      t_up[i] = (node_->now()).seconds();
    }
    if (!ref_state_msg.feet.feet[i].contact &&
        ((node_->now()).seconds() - t_up[i] < 0.03)) {
      // on liftoff reset momentum observer state to 0s
      for (int j = 0; j < 3; j++) {
        p_hat[3 * i + j] = 0;
        r_mom[3 * i + j] = 0;
      }
    } else {
      // Compute joint torque estimates with momentum observer

      for (int j = 0; j < 3; j++) {
        // read joint data from message and convert wire→URDF frame
        int ind = 3 * i + j;
        q[j]  = joint_scale[j] * last_state_msg_->joints.position[ind]
              + joint_offset[j];
        qd[j] = joint_scale[j] * last_state_msg_->joints.velocity[ind];
        tau[j] = MO_ktau[j] * joint_scale[j] * effort_sign[j]
               * last_state_msg_->joints.effort[ind];

        if (cancel_friction_) {
          // Compensate for friction with estimated dry and viscous parameters
          tau[j] += (qd[j] > 0 ? 1 : -1) * MO_fric[j] + qd[j] * MO_damp[j];
        }

        // read this leg's estimates
        re[j] = r_mom[3 * i + j];
        pe[j] = p_hat[3 * i + j];
      }
      // Compute dynamics matrices and vectors
      int RL = i < 2 ? -1 : 1;
      f_M(q, RL, M);
      f_beta(q, qd, RL, beta);
      f_J_MO(q, RL, J_MO);

      // Momentum observer update
      Eigen::Vector3d p = M * qd;
      Eigen::Vector3d pd_hat = tau - beta + re;
      p = p - pe;
      re = K_O * p;
      pe = pe + pd_hat / update_rate_;

      // Effective toe forces
      fe_toe = J_MO.transpose().colPivHouseholderQr().solve(re);

      for (int j = 0; j < 3; j++) {
        r_mom[3 * i + j] = re[j];
        p_hat[3 * i + j] = pe[j];
        f_toe_MO[3 * i + j] = fe_toe[j];
      }
    }
  }

  past_feet_state_ = ref_state_msg.feet;
}

void BodyForceEstimator::publishBodyForce() {
  quad_msgs::msg::BodyForceEstimate msg;
  quad_msgs::msg::GRFArray msg_toe;

  for (int i = 0; i < 4; i++) {
    geometry_msgs::msg::Vector3 ft;

    ft.x = f_toe_MO[3 * i + 0];
    ft.y = f_toe_MO[3 * i + 1];
    ft.z = f_toe_MO[3 * i + 2];

    msg_toe.vectors.push_back(ft);
  }
  for (int i = 0; i < 12; i++) {
    msg.joint_torques.push_back(r_mom[i]);
  }

  msg.header.stamp = node_->now();
  msg_toe.header.stamp = node_->now();

  body_force_pub_->publish(msg);
  toe_force_pub_->publish(msg_toe);
}

void BodyForceEstimator::spin() {
  rclcpp::Rate r(update_rate_);
  while (rclcpp::ok()) {
    // Collect new messages on subscriber topics

    // Compute new estimate
    this->update();

    // Publish new estimate
    publishBodyForce();

    rclcpp::spin_some(node_);
    // Enforce update rate
    r.sleep();
  }
}
