#include "body_force_estimator/body_force_estimator.h"
#include "body_force_estimator/body_force_estimator_dynamics.h"

using namespace force_estimation_dynamics;

BodyForceEstimator::BodyForceEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string robot_state_topic, body_force_topic;
  nh.param<std::string>("topics/state/ground_truth", robot_state_topic, "/state/ground_truth");
  //nh.param<std::string>("topics/joint_encoder", robot_state_topic, "/joint_encoder");
  nh.param<std::string>("topics/body_force", body_force_topic, "/body_force");
  nh.param<double>("body_force_estimator/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one

  // Setup pubs and subs
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&BodyForceEstimator::robotStateCallback, this);
  body_force_pub_ = nh_.advertise<spirit_msgs::BodyForceEstimate>(body_force_topic,1);
}

void BodyForceEstimator::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_state_msg_ = msg;
}

void BodyForceEstimator::update() {
  Eigen::Matrix3d M;
  Eigen::Vector2d beta;

  for (int i = 0; i < 4; i++) {
    // Loop over four legs
    /*
    double q[3] = {last_state_msg_.joints.position[7+i],
      last_state_msg_.joints.position[2*i+0],
      last_state_msg_.joints.position[2*i+1]};
    double qd[3]= {last_state_msg_.joints.velocity[7+i],
      last_state_msg_.joints.velocity[2*i+0],
      last_state_msg_.joints.velocity[2*i+1]};
    double tau[3]= {last_state_msg_.joints.effort[7+i],
      last_state_msg_.joints.effort[2*i+0],
      last_state_msg_.joints.effort[2*i+1]};
    */
    // TODO: working here
  }
}

void BodyForceEstimator::publishBodyForce() {
  // ROS_INFO("In BodyForce");
  spirit_msgs::BodyForceEstimate msg;
  body_force_pub_.publish(msg);
}

void BodyForceEstimator::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics

    ros::spinOnce();
    // Enforce update rate
    r.sleep();
  }
}
