#include "ground_truth_publisher/ground_truth_publisher.h"

GroundTruthPublisher::GroundTruthPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, mocap_topic, ground_truth_state_topic;
  nh.param<std::string>("topics/joint_encoder", joint_encoder_topic, "/joint_encoder");
  nh.param<std::string>("topics/imu", imu_topic, "/imu");
  nh.param<std::string>("topics/mocap", mocap_topic, "/mocap_data");
  nh.param<std::string>("topics/ground_truth_state", ground_truth_state_topic, "/ground_truth_state");
  nh.param<double>("ground_truth_publisher/update_rate", update_rate_, 200);

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&GroundTruthPublisher::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&GroundTruthPublisher::imuCallback, this);
  mocap_sub_ = nh_.subscribe(mocap_topic,1,&GroundTruthPublisher::mocapCallback, this);
  state_pub_ = nh_.advertise<spirit_msgs::StateEstimate>(ground_truth_state_topic,1);
}

void GroundTruthPublisher::mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  last_mocap_msg_ = msg;
}

void GroundTruthPublisher::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void GroundTruthPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

spirit_msgs::StateEstimate GroundTruthPublisher::updateStep() {
  spirit_msgs::StateEstimate new_state_est;

  if (last_imu_msg_ != NULL)
  {
    new_state_est.body.pose.pose.orientation = (*last_imu_msg_).orientation;
    new_state_est.body.twist.twist.angular = (*last_imu_msg_).angular_velocity;

  }
  if (last_joint_state_msg_ != NULL)
  {
    new_state_est.joints = *last_joint_state_msg_;
  }
  if (last_mocap_msg_ != NULL)
  {
    new_state_est.body.pose.pose.position = (*last_mocap_msg_).position;
  }

  new_state_est.header.stamp = ros::Time::now();
  return new_state_est;
}

void GroundTruthPublisher::spin() {
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
