#include "body_force_estimator/body_force_estimator.h"

#include "body_force_estimator/body_force_estimator_dynamics.h"

using namespace force_estimation_dynamics;

int joint_inds[12] = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};

// Effective toe force estimate
double f_toe_MO[12];
// Planned liftoff time
double t_up[4];

BodyForceEstimator::BodyForceEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string robot_state_topic, body_force_topic, toe_force_topic,
      local_plan_topic;
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/body_force/joint_torques",
                           body_force_topic);
  quad_utils::loadROSParam(nh_, "topics/body_force/toe_forces",
                           toe_force_topic);
  nh.param<double>(
      "/body_force_estimator/update_rate", update_rate_,
      250);  // add a param for your package instead of using the estimator one
  nh.param<double>("/body_force_estimator/K_O", K_O_, 50);
  nh.param<int>("/body_force_estimator/cancel_friction", cancel_friction_, 1);

  // Setup pubs and subs
  robot_state_sub_ = nh_.subscribe(
      robot_state_topic, 1, &BodyForceEstimator::robotStateCallback, this,
      ros::TransportHints().tcpNoDelay(true));
  local_plan_sub_ = nh_.subscribe(local_plan_topic, 1,
                                  &BodyForceEstimator::localPlanCallback, this);
  body_force_pub_ =
      nh_.advertise<quad_msgs::BodyForceEstimate>(body_force_topic, 1);
  toe_force_pub_ = nh_.advertise<quad_msgs::GRFArray>(toe_force_topic, 1);
}

void BodyForceEstimator::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_state_msg_ = msg;
}

void BodyForceEstimator::localPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr& msg) {
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

  // Joint directions (todo: make this a parameter or read the URDF)
  int joint_dirs[3] = {1, -1, 1};

  if (last_state_msg_ == NULL || last_local_plan_msg_ == NULL) {
    return;
  }

  // Interpolate the local plan to get the reference state
  quad_msgs::RobotState ref_state_msg;
  ros::Time t_first_state = last_local_plan_msg_->states.front().header.stamp;
  double t_now = (ros::Time::now() - last_local_plan_msg_->state_timestamp)
                     .toSec();  // Use time of state - RECOMMENDED
  if ((t_now <
       (last_local_plan_msg_->states.front().header.stamp - t_first_state)
           .toSec()) ||
      (t_now >
       (last_local_plan_msg_->states.back().header.stamp - t_first_state)
           .toSec())) {
    ROS_ERROR("ID node couldn't find the correct ref state!");
    std::cout << "t_now " << t_now << ", plan front "
              << (last_local_plan_msg_->states.front().header.stamp -
                  t_first_state)
                     .toSec()
              << ", plan back "
              << (last_local_plan_msg_->states.back().header.stamp -
                  t_first_state)
                     .toSec()
              << std::endl;
  }
  for (int i = 0; i < last_local_plan_msg_->states.size() - 1; i++) {
    if ((t_now >= (last_local_plan_msg_->states[i].header.stamp - t_first_state)
                      .toSec()) &&
        (t_now <
         (last_local_plan_msg_->states[i + 1].header.stamp - t_first_state)
             .toSec())) {
      double t_interp =
          (t_now -
           (last_local_plan_msg_->states[i].header.stamp - t_first_state)
               .toSec()) /
          (last_local_plan_msg_->states[i + 1].header.stamp.toSec() -
           last_local_plan_msg_->states[i].header.stamp.toSec());

      // Linearly interpolate between states
      quad_utils::interpRobotState(last_local_plan_msg_->states[i],
                                   last_local_plan_msg_->states[i + 1],
                                   t_interp, ref_state_msg);
      continue;
    }
  }

  if (past_feet_state_.feet.empty()) {
    past_feet_state_ = ref_state_msg.feet;
  }

  for (int i = 0; i < 4; i++) {
    // Loop over four legs: FL, BL, FR, BR

    //*
    if (ref_state_msg.feet.feet[i].contact) {
      t_up[i] = (ros::Time::now()).toSec();
    }
    if (!ref_state_msg.feet.feet[i].contact &&
        ((ros::Time::now()).toSec() - t_up[i] < 0.03)) {
      // on liftoff reset momentum observer state to 0s
      for (int j = 0; j < 3; j++) {
        p_hat[3 * i + j] = 0;
        r_mom[3 * i + j] = 0;
      }
    } else {
      //*/
      // Compute joint torque estimates with momentum observer

      for (int j = 0; j < 3; j++) {
        // read joint data from message
        int ind = 3 * i + j;
        q[j] = joint_dirs[j] * last_state_msg_->joints.position[ind];
        qd[j] = joint_dirs[j] * last_state_msg_->joints.velocity[ind];
        tau[j] =
            MO_ktau[j] * joint_dirs[j] * last_state_msg_->joints.effort[ind];

        if (cancel_friction_) {
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
  // ROS_INFO("In BodyForce");
  quad_msgs::BodyForceEstimate msg;
  quad_msgs::GRFArray msg_toe;

  for (int i = 0; i < 4; i++) {
    geometry_msgs::Vector3 ft;

    ft.x = f_toe_MO[3 * i + 0];
    ft.y = f_toe_MO[3 * i + 1];
    ft.z = f_toe_MO[3 * i + 2];

    msg_toe.vectors.push_back(ft);
  }
  for (int i = 0; i < 12; i++) {
    msg.joint_torques.push_back(r_mom[i]);
  }

  msg.header.stamp = ros::Time::now();
  msg_toe.header.stamp = ros::Time::now();

  body_force_pub_.publish(msg);
  toe_force_pub_.publish(msg_toe);
}

void BodyForceEstimator::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics

    // Compute new estimate
    this->update();

    // Publish new estimate
    publishBodyForce();

    ros::spinOnce();
    // Enforce update rate
    r.sleep();
  }
}
