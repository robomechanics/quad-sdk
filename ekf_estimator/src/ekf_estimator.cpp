#include "ekf_estimator/ekf_estimator.h"


const int numofStates = 6;

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

  // load ground_truth state rosparams and setup subs
  std::string state_ground_truth_topic;
  nh.param<std::string>("topic/state/ground_truth", state_ground_truth_topic, "/state/ground_truth");
  state_ground_truth_pub_ = nh_.subscribe(state_ground_truth_topic,1,&EKFEstimator::groundtruthCallback,this);
  
  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&EKFEstimator::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&EKFEstimator::imuCallback, this);
  contact_sub_ = nh_.subscribe(contact_topic,1,&EKFEstimator::contactCallback, this);
  state_estimate_pub_ = nh_.advertise<quad_msgs::RobotState>(state_estimate_topic,1);
}

void EKFEstimator::groundtruthCallback(const quad_msgs::RobotState::ConstPtr& msg) {
  last_state_msg_ = msg;
}

void EKFEstimator::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	last_joint_state_msg_ = msg;
}

void EKFEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

void EKFEstimator::contactCallback(const quad_msgs::ContactMode::ConstPtr& msg) {
  last_contact_msg_ = msg;
}

quad_msgs::RobotState EKFEstimator::updateStep() {
  // Record start time of function, used in verifying messages are not out of date
  // and in timing function
  ros::Time start_time = ros::Time::now();

  // Create skeleton message to send out
  quad_msgs::RobotState new_state_est;
  
  // Record whether we have good imu and joint state data
  bool good_imu = false;
  bool good_joint_state = false;

  double dt = (start_time - last_time).toSec();
  std::cout << dt << std::endl;
  last_time = start_time;

  
   // Initial linear position body (0:2)
  Eigen::Matrix<double, 3, 1> r0 = Eigen::MatrixXd::Zero(3, 1);
  // Initial linear velocity body (3:5)
  Eigen::Matrix<double, 3, 1> v0 = Eigen::MatrixXd::Zero(3, 1);
  Eigen::MatrixXd x0(r0.rows() + v0.rows(), r0.cols());
  x = x0;
  // Collect position and velocity info from state vector
  Eigen::VectorXd r = last_x.block<3, 1>(0, 0);
  Eigen::VectorXd v = last_x.block<3, 1>(3, 0);

  g = Eigen::VectorXd(3,1);
  g << 0.0,
    0.0,
    9.8;
  
  Eigen::VectorXd a = Eigen::VectorXd(3,1);
  a << (*last_imu_msg_).linear_acceleration.x,
    (*last_imu_msg_).linear_acceleration.y,
    (*last_imu_msg_).linear_acceleration.z;
  
  a = a - g;

  // Zero order hold to predict next state
  // Hold rotation, foothold placements constant
  x.block<3, 1>(0, 0) = r + dt * v + dt * dt * 0.5 * a;
  x.block<3, 1>(3, 0) = v + dt * a;

  last_x = x;


  //publish new message
  new_state_est.header.stamp = ros::Time::now();

  geometry_msgs::Quaternion quat;
  quat.x = 0;
  quat.y = 0;
  quat.z = 0;
  quat.w = 1;

  new_state_est.body.header.stamp = ros::Time::now();
  new_state_est.body.pose.orientation = quat;
  new_state_est.body.pose.position.x = new_state_est.body.pose.position.x + 1;
  

  new_state_est.joints.position = (*last_joint_state_msg_).position;
  new_state_est.joints.header.stamp = ros::Time::now();
 
  
  return new_state_est;
}

void EKFEstimator::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Compute new state estimate
    quad_msgs::RobotState new_state_est = this->updateStep();

    // Publish new state estimate
    state_estimate_pub_.publish(new_state_est);
    
    // std::cout<<"state published" << std::endl;


    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;
    

    // Enforce update rate
    r.sleep();
  }
}
