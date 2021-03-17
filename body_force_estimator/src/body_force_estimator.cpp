#include "body_force_estimator/body_force_estimator.h"
#include "body_force_estimator/body_force_estimator_dynamics.h"

using namespace force_estimation_dynamics;

// Temporary
#if USE_SIM == 1
int joint_inds [12] = {10, 0, 1, 11, 4, 5, 2, 6, 7, 3, 8, 9};
#elif USE_SIM == 2
int joint_inds [12] = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};
#endif

// Effective toe force estimate
double f_toe_MO[12];

BodyForceEstimator::BodyForceEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string robot_state_topic, body_force_topic;
  #if USE_SIM == 2
  nh.param<std::string>("topics/joint_encoder", robot_state_topic, "/joint_encoder");
  #else
  nh.param<std::string>("topics/state/ground_truth", robot_state_topic, "/state/ground_truth");
  #endif
  nh.param<std::string>("topics/body_force", body_force_topic, "/body_force");
  nh.param<double>("body_force_estimator/update_rate", update_rate_, 100); // add a param for your package instead of using the estimator one
  nh.param<double>("body_force_estimator/K_O", K_O_, 20);

  // Setup pubs and subs
  #if USE_SIM == 1
  robot_state_sub_ = nh_.subscribe("/spirit/joint_states",1,&BodyForceEstimator::robotStateCallback, this);
  #elif USE_SIM == 2
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&BodyForceEstimator::robotStateCallback, this);
  #else
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&BodyForceEstimator::robotStateCallback, this);
  #endif
  body_force_pub_ = nh_.advertise<spirit_msgs::BodyForceEstimate>(body_force_topic,1);
}

#if USE_SIM > 0
void BodyForceEstimator::robotStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
#else
void BodyForceEstimator::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
#endif
  // ROS_INFO("In robotStateCallback");
  last_state_msg_ = msg;
}

void BodyForceEstimator::update() {
  Eigen::Matrix3d M;
  Eigen::Vector3d beta;
  Eigen::Matrix3d J_MO; // Jacobian

  Eigen::Vector3d q; // joint positions
  Eigen::Vector3d qd; // joint velocities
  Eigen::Vector3d tau; // joint actuator torques
  Eigen::Vector3d re; // momentum observer external torque estimates
  Eigen::Vector3d pe; // momentum observer momentum estimate
  Eigen::Vector3d fe_toe; // effective toe forces

  // Momentum observer gain (todo: this should be precomputed once)
  Eigen::Matrix3d K_O;
  K_O << K_O_, 0, 0,
         0, K_O_, 0,
         0, 0, K_O_;

  // Joint directions (todo: make this a parameter or read the URDF)
  int joint_dirs [3] = {1, -1, 1};

  if (last_state_msg_ == NULL) {
    return;
  }

  for (int i = 0; i < 4; i++) {
    // Loop over four legs: FL, BL, FR, BR
    for (int j = 0; j < 3; j++) {
      // read joint data from message
      #if USE_SIM > 0
      int ind = joint_inds[3*i+j];
      q[j] = joint_dirs[j]*last_state_msg_->position[ind];
      qd[j] = joint_dirs[j]*last_state_msg_->velocity[ind];
      tau[j] = MO_ktau[j]*joint_dirs[j]*last_state_msg_->effort[ind];
      #else
      int ind = 3*i+j;
      q[j] = joint_dirs[j]*last_state_msg_->joints.position[ind];
      qd[j] = joint_dirs[j]*last_state_msg_->joints.velocity[ind];
      tau[j] = MO_ktau[j]*joint_dirs[j]*last_state_msg_->joints.effort[ind];
      #endif

      tau[j] += (qd[j] > 0 ? 1 : -1) * MO_fric[j] + qd[j] * MO_damp[j];

      // read this leg's estimates
      re[j] = r_mom[3*i+j];
      pe[j] = p_hat[3*i+j];
    }
    // Compute dynamics matrices and vectors
    int RL = i < 2 ? -1 : 1;
    f_M(q, RL, M);
    f_beta(q, qd, RL, beta);
    f_J_MO(q, RL, J_MO);

    // Momentum observer update
    Eigen::Vector3d p = M*qd;
    Eigen::Vector3d pd_hat = tau - beta + re;
    p = p - pe;
    re = K_O*p;
    pe = pe + pd_hat/update_rate_;

    // Effective toe forces
    fe_toe = J_MO.transpose().colPivHouseholderQr().solve(re);

    for (int j = 0; j < 3; j++) {
      r_mom[3*i+j] = re[j];
      p_hat[3*i+j] = pe[j];
      f_toe_MO[3*i+j] = fe_toe[j];
    }
  }
}

void BodyForceEstimator::publishBodyForce() {
  // ROS_INFO("In BodyForce");
  spirit_msgs::BodyForceEstimate msg;

  for (int i = 0; i < 4; i++) {
    geometry_msgs::Wrench w;
    w.torque.x = r_mom[3*i+0];
    w.torque.y = r_mom[3*i+1];
    w.torque.z = r_mom[3*i+2];

    w.force.x = f_toe_MO[3*i+0];
    w.force.y = f_toe_MO[3*i+1];
    w.force.z = f_toe_MO[3*i+2];
    /*
    #if USE_SIM > 0
    if (last_state_msg_ != NULL) {
      w.force.x = last_state_msg_->position[joint_inds[3*i+0]];
      w.force.y = last_state_msg_->position[joint_inds[3*i+1]];
      w.force.z = last_state_msg_->position[joint_inds[3*i+2]];
    }
    #endif
    */
    msg.body_wrenches.push_back(w);
  }

  msg.header.stamp = ros::Time::now();

  body_force_pub_.publish(msg);
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
