#include "ekf_estimator/ekf_estimator.h"


const int numofStates = 6;
const int num_feet_ = 4;
EKFEstimator::EKFEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic, state_estimate_topic;
  nh.param<std::string>("topics/mcu/state/jointURDF", joint_encoder_topic, "/mcu/state/jointURDF");
  nh.param<std::string>("topics/mcu/state/imu", imu_topic, "/mcu/state/imu");
  nh.param<std::string>("topics/state/estimate", state_estimate_topic, "/state/estimate");
  nh.param<std::string>("topics/contact_mode", contact_topic, "/contact_mode");
  nh.param<double>("ekf_estimator/update_rate", update_rate_, 200);
  nh.param<double>("ekf_estimator/joint_state_max_time",joint_state_msg_time_diff_max_,20);

  // Load IMU bias
  nh.getParam("/ekf_estimator/bias_x", bias_x_);
  nh.getParam("/ekf_estimator/bias_y", bias_y_);
  nh.getParam("/ekf_estimator/bias_z", bias_z_);
  // load ground_truth state rosparams and setup subs
  std::string state_ground_truth_topic;
  nh.param<std::string>("topic/state/ground_truth", state_ground_truth_topic, "/state/ground_truth");
  state_ground_truth_pub_ = nh_.subscribe(state_ground_truth_topic,1,&EKFEstimator::groundtruthCallback,this);

  
  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&EKFEstimator::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&EKFEstimator::imuCallback, this);
  contact_sub_ = nh_.subscribe(contact_topic,1,&EKFEstimator::contactCallback, this);
  state_estimate_pub_ = nh_.advertise<quad_msgs::RobotState>(state_estimate_topic,1);

  // QuadKD class
  // quadKD_ = std::make_shared<quad_utils::QuadKD>();
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
  bool good_ground_truth_state = false;

  //calculate dt
  double dt = (start_time - last_time).toSec();
  last_time = start_time;
  

  /// Collect Data

  // IMU reading linear xyz (0-2) angular (3-5)
  Eigen::VectorXd imu_data = Eigen::VectorXd::Zero(6);
  // IMU orientation
  geometry_msgs::Quaternion imu_ori;
  imu_ori.x = 0;
  imu_ori.y = 0;
  imu_ori.z = 0;
  imu_ori.w = 1;
  if (last_imu_msg_ != NULL){
    good_imu = true;
    imu_data << (*last_imu_msg_).linear_acceleration.x,
      (*last_imu_msg_).linear_acceleration.y,
      (*last_imu_msg_).linear_acceleration.z,
      (*last_imu_msg_).angular_velocity.x,
      (*last_imu_msg_).angular_velocity.y,
      (*last_imu_msg_).angular_velocity.z;
    imu_ori = (*last_imu_msg_).orientation;
  }
  
  // Joint data reading 3 joints * 4 legs
  std::vector<double> joints_data(12, 0);
  if (last_joint_state_msg_ != NULL)
  {
    good_joint_state = true;
    for (int i = 0; i < 12; i++){
      joints_data[i] = (*last_joint_state_msg_).position[i];
    } 
  }
  else
  {
    good_joint_state = false;
    joints_data = {0.767431, 1.591838, 0.804742, 1.612967, 0.721895, 1.562485, 0.729919, 1.514980, -0.012140, -0.024205, 0.050132, 0.058434};
  }
  
  

  // Collect position and velocity info from previous state vector
  Eigen::VectorXd r = last_x.block<3, 1>(0, 0);
  Eigen::VectorXd v = last_x.block<3, 1>(3, 0);

  // imu bias measured 
  imu_bias = Eigen::VectorXd(3,1);
  imu_bias << bias_x_,
    bias_y_,
    bias_z_;
  
  // calculate acceleration set z acceleration to 0 for now
  Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
  a = imu_data.head(3) - imu_bias;
  a[2] = 0.0;


  /// update state x and last_x
  // Zero order hold to predict next state
  // Hold rotation, foothold placements constant
  x.block<3, 1>(0, 0) = r + dt * v + dt * dt * 0.5 * a;
  x.block<3, 1>(3, 0) = v + dt * a;

  last_x = x;
 

  // Update when I have good data, otherwise stay at the origin  
  if (last_state_msg_ != NULL){
    good_ground_truth_state = true;
    
  }else{
    good_ground_truth_state = false;
    x = Eigen::VectorXd::Zero(6);
    last_x = Eigen::VectorXd::Zero(6); 
    a = Eigen::VectorXd::Zero(3);
    x << - 1.457778,
      1.004244,
      0.308681,
      0,
      0,
      0;
    last_x = x;
  }

  /// publish new message
  new_state_est.header.stamp = ros::Time::now();
  
  // body
  new_state_est.body.header.stamp = ros::Time::now();
  new_state_est.body.pose.orientation.x = imu_ori.x;
  new_state_est.body.pose.orientation.y = imu_ori.y;
  new_state_est.body.pose.orientation.z = imu_ori.z;
  new_state_est.body.pose.orientation.w = imu_ori.w;
  new_state_est.body.pose.position.x = x[0];
  new_state_est.body.pose.position.y = x[1];
  new_state_est.body.pose.position.z = x[2];

  // joint
  new_state_est.joints.header.stamp = ros::Time::now();
  new_state_est.joints.name = {"0", "1", "2","3", "4", "5", "6","7",
  "8", "9", "10", "11"};
  new_state_est.joints.position = joints_data;
  new_state_est.joints.velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
  new_state_est.joints.effort = {0,0,0,0,0,0,0,0,0,0,0,0};


  return new_state_est;
}

void EKFEstimator::spin() {
  ros::Rate r(update_rate_);
  
  // Initial linear position body (0:2)
  // Initial linear velocity body (3:5) 
  Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(6, 1);
  x0 << - 1.457778,
      1.004244,
      0.308681,
      0,
      0,
      0;
  x = x0;
  last_x = x;

  // std::cout << "estimate state x" << x << std::endl;
  std::cout << "bias x is" << bias_x_ << std::endl;
  std::cout << "bias y is" << bias_y_ << std::endl;
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Compute new state estimate
    quad_msgs::RobotState new_state_est = this->updateStep();

    // Publish new state estimate
    state_estimate_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;
    

    // Enforce update rate
    r.sleep();
  }
}
