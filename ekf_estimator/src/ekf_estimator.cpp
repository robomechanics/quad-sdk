#include "ekf_estimator/ekf_estimator.h"

EKFEstimator::EKFEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic, state_estimate_topic;
  nh.param<std::string>("topics/joint_encoder", joint_encoder_topic, "/joint_encoder");
  nh.param<std::string>("topics/imu", imu_topic, "/imu");
  nh.param<std::string>("topics/state_estimate", state_estimate_topic, "/state_estimate");
  nh.param<std::string>("topics/contact_detection", contact_topic, "/contact_detection");
  nh.param<double>("ekf_estimator/update_rate", update_rate_, 200);

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&EKFEstimator::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&EKFEstimator::imuCallback, this);
  contact_sub_ = nh_.subscribe(contact_topic,1,&EKFEstimator::contactCallback, this);
  state_estimate_pub_ = nh_.advertise<spirit_msgs::StateEstimate>(state_estimate_topic,1);
}

void EKFEstimator::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void EKFEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

void EKFEstimator::contactCallback(const spirit_msgs::ContactDetection::ConstPtr& msg) {
  last_contact_msg_ = msg;
}

spirit_msgs::StateEstimate EKFEstimator::updateStep() {
  spirit_msgs::StateEstimate new_state_est;

  if (last_imu_msg_ != NULL)
  {
    new_state_est.body.pose.pose.orientation = (*last_imu_msg_).orientation;
  } else {
    geometry_msgs::Quaternion quat;
    quat.x = 0;
    quat.y = 0;
    quat.z = 0;
    quat.w = 1;
    new_state_est.body.pose.pose.orientation = quat;
  }
  if (last_joint_state_msg_ != NULL)
  {
    new_state_est.joints = *last_joint_state_msg_;
  }

  new_state_est.header.stamp = ros::Time::now();
  return new_state_est;
}

void EKFEstimator::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Compute new state estimate
    spirit_msgs::StateEstimate new_state_est = this->updateStep();

    // Publish new state estimate
    state_estimate_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}