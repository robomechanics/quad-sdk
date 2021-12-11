#include "quad_utils/ground_truth_publisher.h"

GroundTruthPublisher::GroundTruthPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, vel_topic, mocap_topic, ground_truth_state_topic, ground_truth_state_body_frame_topic;

  quad_utils::loadROSParam(nh_,"topics/joint_encoder",joint_encoder_topic);
  quad_utils::loadROSParam(nh_,"topics/imu",imu_topic);
  quad_utils::loadROSParam(nh_,"topics/vel",vel_topic);
  quad_utils::loadROSParam(nh_,"topics/mocap",mocap_topic);
  quad_utils::loadROSParam(nh_,"topics/state/ground_truth",ground_truth_state_topic);
  quad_utils::loadROSParam(nh_,"topics/state/ground_truth_body_frame",ground_truth_state_body_frame_topic);
  quad_utils::loadROSParam(nh_,"ground_truth_publisher/velocity_smoothing_weight",alpha_);
  quad_utils::loadROSParam(nh_,"ground_truth_publisher/mocap_dropout_threshold",mocap_dropout_threshold_);
  quad_utils::loadROSParam(nh_,"ground_truth_publisher/mocap_rate",mocap_rate_);
  quad_utils::loadROSParam(nh_,"ground_truth_publisher/update_rate",update_rate_);

  // Assume zero initial velocity
  mocap_vel_estimate_.setZero();

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&GroundTruthPublisher::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&GroundTruthPublisher::imuCallback, this);
  vel_sub_ = nh_.subscribe(vel_topic,1,&GroundTruthPublisher::velCallback, this);
  mocap_sub_ = nh_.subscribe(mocap_topic,1000,&GroundTruthPublisher::mocapCallback, this, ros::TransportHints().tcpNoDelay(true));
  ground_truth_state_pub_ = nh_.advertise<quad_msgs::RobotState>(ground_truth_state_topic,1);
  ground_truth_state_body_frame_pub_ = nh_.advertise<quad_msgs::RobotState>(ground_truth_state_body_frame_topic,1);

  // Convert kinematics
  quadKD_ = std::make_shared<quad_utils::QuadKD>();

  // Initialize vectors for velocity history
  vel_hist_.resize(3);
  for (int i = 0; i < vel_hist_.size(); i++) {
    vel_hist_[i].resize(median_filter_window_);
  }

  joints_order_ = {8, 0, 1, 9, 2, 3, 10, 4, 5, 11, 6, 7};
}

void GroundTruthPublisher::mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  if (last_mocap_msg_ != NULL)
  {
    // Collect change in position for velocity update
    Eigen::Vector3d pos_new, pos_old;
    quad_utils::pointMsgToEigen(msg->pose.position, pos_new);
    quad_utils::pointMsgToEigen(last_mocap_msg_->pose.position, pos_old);

    // Record time diff between messages
    double t_diff_mocap_msg = (msg->header.stamp - last_mocap_msg_->header.stamp).toSec();

    // Use new measurement
    if (abs(t_diff_mocap_msg - 1.0/mocap_rate_) < mocap_dropout_threshold_) {
      
      // Declare vectors for vel measurement and estimate
      Eigen::Vector3d vel_new_measured, vel_new_est;
      vel_new_measured = (pos_new - pos_old)*mocap_rate_;

      // Filtered velocity estimate assuming motion capture frame rate is constant at mocap_rate_
      // in order to avoid variable network and ROS latency that appears in the message time stamp
      mocap_vel_estimate_ = alpha_*mocap_vel_estimate_ + (1-alpha_)*vel_new_measured;
    } else {
      ROS_WARN("Mocap time diff exceeds max dropout threshold, using latest value");
      mocap_vel_estimate_ = (pos_new - pos_old)/t_diff_mocap_msg;
    }
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

bool GroundTruthPublisher::updateStep(quad_msgs::RobotState &new_state_est) {

  bool fully_populated = true;

  if (last_vel_msg_ != NULL)
  {
    // new_state_est.body.pose.pose.orientation = last_imu_msg_->orientation;
    new_state_est.body.twist.angular = last_vel_msg_->twist.angular;
    new_state_est.body.twist.linear = last_vel_msg_->twist.linear;
  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No imu in /state/ground_truth");
  }
  if (last_mocap_msg_ != NULL)
  {
    new_state_est.body.pose.orientation = last_mocap_msg_->pose.orientation;
    new_state_est.body.pose.position = last_mocap_msg_->pose.position;
    quad_utils::Eigen3ToVector3Msg(mocap_vel_estimate_,new_state_est.body.twist.linear);
  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No body pose (mocap) in /state/ground_truth");

    new_state_est.body.pose.orientation.x = 0;
    new_state_est.body.pose.orientation.y = 0;
    new_state_est.body.pose.orientation.z = 0;
    new_state_est.body.pose.orientation.w = 1;
    new_state_est.body.pose.position.x = 0;
    new_state_est.body.pose.position.y = 0;
    new_state_est.body.pose.position.z = 0;
  }
  if (last_joint_state_msg_ != NULL)
  {

    new_state_est.joints.name.resize(joints_order_.size());
    new_state_est.joints.position.resize(joints_order_.size());
    new_state_est.joints.velocity.resize(joints_order_.size());
    new_state_est.joints.effort.resize(joints_order_.size());

    for (size_t i = 0; i < joints_order_.size(); i++)
    {
      new_state_est.joints.name.at(i) = last_joint_state_msg_->name.at(joints_order_.at(i));
      new_state_est.joints.position.at(i) = last_joint_state_msg_->position.at(joints_order_.at(i));
      new_state_est.joints.velocity.at(i) = last_joint_state_msg_->velocity.at(joints_order_.at(i));
      new_state_est.joints.effort.at(i) = last_joint_state_msg_->effort.at(joints_order_.at(i));
    }

    if (last_vel_msg_ != NULL)
    {
      quad_utils::fkRobotState(*quadKD_, new_state_est.body,new_state_est.joints, new_state_est.feet);
    }
  } else {
    fully_populated = false;
    ROS_WARN_THROTTLE(1, "No joints or feet in /state/ground_truth");
  }

  quad_utils::updateStateHeaders(new_state_est, ros::Time::now(), "map", 0);
  return fully_populated;
}

void GroundTruthPublisher::spin() {
  ros::Rate r(update_rate_);

  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Compute new state estimate
    quad_msgs::RobotState new_state_est;
    bool fully_populated = this->updateStep(new_state_est);

    // Publish new state estimate if fully populated
    if (fully_populated) {
      ground_truth_state_pub_.publish(new_state_est);
    }
    ground_truth_state_body_frame_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}
