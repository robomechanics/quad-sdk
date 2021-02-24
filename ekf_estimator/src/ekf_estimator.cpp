#include "ekf_estimator/ekf_estimator.h"

EKFEstimator::EKFEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic, state_estimate_topic;
  nh.param<std::string>("topics/joint_encoder", joint_encoder_topic, "/joint_encoder");
  nh.param<std::string>("topics/imu", imu_topic, "/imu");
  nh.param<std::string>("topics/state/estimate", state_estimate_topic, "/state/estimate");
  nh.param<std::string>("topics/contact_mode", contact_topic, "/contact_mode");
  nh.param<double>("ekf_estimator/update_rate", update_rate_, 200);
  nh.param<double>("ekf_estimator/joint_state_max_time",joint_state_msg_time_diff_max_,20);
  
  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&EKFEstimator::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&EKFEstimator::imuCallback, this);
  contact_sub_ = nh_.subscribe(contact_topic,1,&EKFEstimator::contactCallback, this);
  state_estimate_pub_ = nh_.advertise<spirit_msgs::RobotState>(state_estimate_topic,1);
}

void EKFEstimator::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	last_joint_state_msg_ = msg;
}

void EKFEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

void EKFEstimator::contactCallback(const spirit_msgs::ContactMode::ConstPtr& msg) {
  last_contact_msg_ = msg;
}

spirit_msgs::RobotState EKFEstimator::updateStep() {
  // Record start time of function, used in verifying messages are not out of date
  // and in timing function
  ros::Time start_time = ros::Time::now();

  // Create skeleton message to send out
  spirit_msgs::RobotState new_state_est;

  // Record whether we have good imu and joint state data
  bool good_imu = false;
  bool good_joint_state = false;

  // Collect info from last imu message
  if (last_imu_msg_ != NULL)
  {
    new_state_est.body.pose.pose.orientation = (*last_imu_msg_).orientation;
    good_imu = true;
  }
  else {
    geometry_msgs::Quaternion quat;
    quat.x = 0;
    quat.y = 0;
    quat.z = 0;
    quat.w = 1;
    new_state_est.body.pose.pose.orientation = quat;
  }
  // Collect info from last joint state message, making sure info is not out of date
  if (last_joint_state_msg_ != NULL)
  {
    new_state_est.joints = *last_joint_state_msg_;
    double joint_state_msg_time_diff = 0;
    if (joint_state_msg_time_diff > joint_state_msg_time_diff_max_)
    {
      // Don't use this info in EKF update!
      ROS_WARN("Haven't received a recent joint state message, skipping EKF measurement step");
    }
    else
    {
      good_joint_state = true;
    }
  }
  else {
    ROS_WARN_THROTTLE(0.5,"Still waiting for first joint state message");
    new_state_est.joints.header.stamp = ros::Time::now();
    new_state_est.joints.name = {"8", "0", "1", "9","2", "3", "10", "4","5", 
      "11", "6", "7"};
      new_state_est.joints.position = {0,0,0,0,0,0,0,0,0,0,0,0};
      new_state_est.joints.velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
      new_state_est.joints.effort = {0,0,0,0,0,0,0,0,0,0,0,0};
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
    spirit_msgs::RobotState new_state_est = this->updateStep();

    // Publish new state estimate
    state_estimate_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}
