#include "spirit_utils/ground_truth_publisher.h"

GroundTruthPublisher::GroundTruthPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, vel_topic, mocap_topic, ground_truth_state_topic;

  spirit_utils::loadROSParam(nh_,"topics/joint_encoder",joint_encoder_topic);
  spirit_utils::loadROSParam(nh_,"topics/imu",imu_topic);
  spirit_utils::loadROSParam(nh_,"topics/vel",vel_topic);
  spirit_utils::loadROSParam(nh_,"topics/mocap",mocap_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",ground_truth_state_topic);
  spirit_utils::loadROSParam(nh_,"ground_truth_publisher/velocity_smoothing_weight",alpha_);
  spirit_utils::loadROSParam(nh_,"ground_truth_publisher/mocap_rate",mocap_rate_);

  // Assume zero initial velocity
  mocap_vel_estimate_.x = 0;
  mocap_vel_estimate_.y = 0;
  mocap_vel_estimate_.z = 0;

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&GroundTruthPublisher::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&GroundTruthPublisher::imuCallback, this);
  vel_sub_ = nh_.subscribe(vel_topic,1,&GroundTruthPublisher::velCallback, this);
  mocap_sub_ = nh_.subscribe(mocap_topic,1,&GroundTruthPublisher::mocapCallback, this);
  ground_truth_state_pub_ = nh_.advertise<spirit_msgs::RobotState>(ground_truth_state_topic,1);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();
}

void GroundTruthPublisher::mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  if (last_mocap_msg_ != NULL)
  {
    // Collect change in position for velocity update
    double xDiff = msg->pose.position.x - last_mocap_msg_->pose.position.x;
    double yDiff = msg->pose.position.y - last_mocap_msg_->pose.position.y;
    double zDiff = msg->pose.position.z - last_mocap_msg_->pose.position.z;

    // Filtered velocity estimate assuming motion capture frame rate is constant at mocap_rate_
    // in order to avoid variable network and ROS latency that appears in the message time stamp
    mocap_vel_estimate_.x = alpha_*mocap_vel_estimate_.x + (1-alpha_)*xDiff*mocap_rate_;
    mocap_vel_estimate_.y = alpha_*mocap_vel_estimate_.y + (1-alpha_)*yDiff*mocap_rate_;
    mocap_vel_estimate_.z = alpha_*mocap_vel_estimate_.z + (1-alpha_)*zDiff*mocap_rate_;
  }

  // Update our cached mocap position
  last_mocap_msg_ = msg;
}

void GroundTruthPublisher::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void GroundTruthPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

void GroundTruthPublisher::velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  last_vel_msg_ = msg;
}

bool GroundTruthPublisher::updateStep(spirit_msgs::RobotState &new_state_est) {

  bool fully_populated = true;

  if (last_vel_msg_ != NULL)
  {
    // new_state_est.body.pose.orientation = last_imu_msg_->orientation;
    new_state_est.body.twist.angular.y = last_vel_msg_->twist.angular.x;
    new_state_est.body.twist.angular.x = last_vel_msg_->twist.angular.y;
    new_state_est.body.twist.angular.z = last_vel_msg_->twist.angular.z;
  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No imu in /state/ground_truth");
  }
  if (last_mocap_msg_ != NULL)
  {
    new_state_est.body.pose.orientation = last_mocap_msg_->pose.orientation;
    new_state_est.body.pose.position = last_mocap_msg_->pose.position;
    new_state_est.body.twist.linear = mocap_vel_estimate_;
  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No body pose (mocap) in /state/ground_truth");
  }
  if (last_joint_state_msg_ != NULL)
  {
    new_state_est.joints = *last_joint_state_msg_;
    if (last_vel_msg_ != NULL)
    {
      spirit_utils::fkRobotState(*kinematics_, new_state_est.body,new_state_est.joints, new_state_est.feet);
    }
  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No joints or feet in /state/ground_truth");
  }

  new_state_est.header.stamp = ros::Time::now();
  return fully_populated;
}

void GroundTruthPublisher::spin() {
  ros::Rate r(mocap_rate_);

  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Compute new state estimate
    spirit_msgs::RobotState new_state_est;
    bool fully_populated = this->updateStep(new_state_est);

    // Publish new state estimate if fully populated
    if (fully_populated) {
      ground_truth_state_pub_.publish(new_state_est);
    }
    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}
