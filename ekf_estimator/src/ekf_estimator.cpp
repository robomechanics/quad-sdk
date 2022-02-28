#include "ekf_estimator/ekf_estimator.h"

EKFEstimator::EKFEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic,
      state_estimate_topic;
  nh.param<std::string>("topics/mcu/state/jointURDF", joint_encoder_topic,
                        "/mcu/state/jointURDF");
  nh.param<std::string>("topics/mcu/state/imu", imu_topic, "/mcu/state/imu");
  nh.param<std::string>("topics/state/estimate", state_estimate_topic,
                        "/state/estimate");
  nh.param<std::string>("topics/contact_mode", contact_topic, "/contact_mode");
  nh.param<double>("ekf_estimator/update_rate", update_rate_, 200);
  nh.param<double>("ekf_estimator/joint_state_max_time",
                   joint_state_msg_time_diff_max_, 20);

  // // Load IMU bias
  // nh.getParam("/ekf_estimator/bias_x", bias_x_);
  // nh.getParam("/ekf_estimator/bias_y", bias_y_);
  // nh.getParam("/ekf_estimator/bias_z", bias_z_);

  // load ground_truth state rosparams and setup subs
  std::string state_ground_truth_topic;
  nh.param<std::string>("topic/state/ground_truth", state_ground_truth_topic,
                        "/state/ground_truth");
  state_ground_truth_sub_ = nh_.subscribe(
      state_ground_truth_topic, 1, &EKFEstimator::groundtruthCallback, this);

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic, 1,
                                     &EKFEstimator::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 1, &EKFEstimator::imuCallback, this);
  contact_sub_ =
      nh_.subscribe(contact_topic, 1, &EKFEstimator::contactCallback, this);
  state_estimate_pub_ =
      nh_.advertise<quad_msgs::RobotState>(state_estimate_topic, 1);

  // QuadKD class
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

void EKFEstimator::groundtruthCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  last_state_msg_ = msg;
}

void EKFEstimator::jointEncoderCallback(
    const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void EKFEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

void EKFEstimator::contactCallback(
    const quad_msgs::ContactMode::ConstPtr& msg) {
  last_contact_msg_ = msg;
}

quad_msgs::RobotState EKFEstimator::updateStep() {
  // Record start time of function, used in verifying messages are not out of
  // date and in timing function
  ros::Time start_time = ros::Time::now();

  // Create skeleton message to send out
  quad_msgs::RobotState new_state_est;

  // Record whether we have good imu and joint state data
  bool good_imu = false;
  bool good_joint_state = false;
  bool good_ground_truth_state = false;

  // calculate dt
  double dt = (start_time - last_time).toSec();
  last_time = start_time;

  /// Collect and Process Data
  // IMU reading linear acceleration
  Eigen::VectorXd fk = Eigen::VectorXd::Zero(3);
  // IMU reading angular acceleration
  Eigen::VectorXd wk = Eigen::VectorXd::Zero(3);
  // IMU orientation (w, x, y, z)
  Eigen::Quaterniond qk(1, 0, 0, 0);
  // if there is good imu data: read data from bag file
  if (last_imu_msg_ != NULL) {
    good_imu = true;
    fk << (*last_imu_msg_).linear_acceleration.x,
        (*last_imu_msg_).linear_acceleration.y,
        (*last_imu_msg_).linear_acceleration.z;
    wk << (*last_imu_msg_).angular_velocity.x,
        (*last_imu_msg_).angular_velocity.y,
        (*last_imu_msg_).angular_velocity.z;

    qk.w() = (*last_imu_msg_).orientation.w;
    qk.x() = (*last_imu_msg_).orientation.x;
    qk.y() = (*last_imu_msg_).orientation.y;
    qk.z() = (*last_imu_msg_).orientation.z;
    qk.normalize();
  }

  // Joint data reading 3 joints * 4 legs
  std::vector<double> jk(12, 0);
  if (last_joint_state_msg_ != NULL) {
    good_joint_state = true;
    for (int i = 0; i < 12; i++) {
      jk[i] = (*last_joint_state_msg_).position[i];
    }
  } else {
    good_joint_state = false;
    // nominal joint encoder values
    jk = {0.767431, 1.591838, 0.804742,  1.612967,  0.721895, 1.562485,
          0.729919, 1.514980, -0.012140, -0.024205, 0.050132, 0.058434};
  }

  // calculate rotational matrix from world frame to body frame
  Eigen::Matrix3d C = qk.toRotationMatrix();

  // Collect states info from previous state vector
  Eigen::VectorXd r = last_X.block<3, 1>(0, 0);
  Eigen::VectorXd v = last_X.block<3, 1>(3, 0);
  Eigen::VectorXd q = last_X.block<4, 1>(6, 0);
  Eigen::VectorXd p = last_X.block<12, 1>(10, 0);
  Eigen::VectorXd bf = last_X.block<1, 1>(22, 0);
  Eigen::VectorXd bw = last_X.block<1, 1>(23, 0);

  // calculate linear acceleration set z acceleration to -9.8 for now
  // Set imu bias
  Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
  a = fk - bf;
  a[2] = -9.8;
  // calculate angular acceleration
  // Set imu bias
  Eigen::VectorXd w = Eigen::VectorXd::Zero(3);
  w = wk - bw;

  /// Prediction Step
  // Set predicted state estimate X_pre
  X_pre = Eigen::VectorXd::Zero(num_state);
  X_pre.block<3, 1>(0, 0) =
      r + dt * v + dt * dt * 0.5 * (C.transpose() * a + g);
  X_pre.block<3, 1>(3, 0) = v + dt * (C.transpose() * a + g);
  // quaternion updates
  Eigen::VectorXd wdt = dt * w;
  Eigen::VectorXd q_pre = this->quaternionDynamics(wdt, q);
  // std::cout << "this is qk+1" << q_pre << std::endl;
  X_pre.block<4, 1>(6, 0) = q_pre;
  X_pre.block<12, 1>(10, 0) = X.block<12, 1>(10, 0);
  X_pre.block<6, 1>(22, 0) = X.block<6, 1>(22, 0);

  // Linearized dynamics matrix
  F = Eigen::MatrixXd::Identity(num_cov, num_cov);
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  Eigen::MatrixXd fkskew = this->calcSkewsym(a);
  Eigen::MatrixXd r0 = this->calcRodrigues(dt, w, 0);
  Eigen::MatrixXd r1 = this->calcRodrigues(dt, w, 1);
  Eigen::MatrixXd r2 = this->calcRodrigues(dt, w, 2);
  Eigen::MatrixXd r3 = this->calcRodrigues(dt, w, 3);

  F.block<3, 3>(0, 6) = -(pow(dt, 2) / 2) * C.transpose() * fkskew;
  F.block<3, 3>(0, 21) = -(pow(dt, 2) / 2) * C.transpose();
  F.block<3, 3>(3, 6) = -dt * C.transpose() * fkskew;
  F.block<3, 3>(3, 21) = -dt * C.transpose();
  F.block<3, 3>(6, 6) = r0.transpose();
  F.block<3, 3>(0, 24) = -1 * r1.transpose();
  Eigen::MatrixXd Ft = F.transpose();

  // Discrete Process Noise Covariance Matrix
  Q = Eigen::MatrixXd::Zero(num_cov, num_cov);
  Q.block<3, 3>(0, 0) =
      (pow(dt, 3) / 3) * noise_acc + (pow(dt, 5) / 20) * bias_acc;
  Q.block<3, 3>(0, 3) =
      (pow(dt, 2) / 2) * noise_acc + (pow(dt, 4) / 8) * bias_acc;
  Q.block<3, 3>(0, 21) = -1 * (pow(dt, 3) / 6) * C.transpose() * bias_acc;
  Q.block<3, 3>(3, 0) =
      (pow(dt, 2) / 2) * noise_acc + (pow(dt, 4) / 8) * bias_acc;
  Q.block<3, 3>(3, 3) = dt * noise_acc + (pow(dt, 3) / 3) * bias_acc;
  Q.block<3, 3>(3, 21) = -1 * (pow(dt, 2) / 2) * C.transpose() * bias_acc;
  Q.block<3, 3>(6, 6) = dt * noise_gyro + r3 + r3.transpose() * bias_gyro;
  Q.block<3, 3>(6, 24) = -r2.transpose() * bias_gyro;
  // will change based on contact
  Q.block<3, 3>(9, 9) = dt * C.transpose() * noise_feet * C;
  Q.block<3, 3>(12, 12) = dt * C.transpose() * noise_feet * C;
  Q.block<3, 3>(15, 15) = dt * C.transpose() * noise_feet * C;
  Q.block<3, 3>(18, 18) = dt * C.transpose() * noise_feet * C;
  Q.block<3, 3>(21, 0) = -1 * (pow(dt, 3) / 6) * bias_acc * C;
  Q.block<3, 3>(21, 3) = -1 * (pow(dt, 2) / 2) * bias_acc * C;
  Q.block<3, 3>(21, 21) = dt * bias_acc;
  Q.block<3, 3>(24, 6) = -1 * bias_gyro * r2;
  Q.block<3, 3>(24, 24) = dt * bias_gyro;

  // Covariance update
  P_pre = F * P * Ft + Q;

  /// Update Step
  // Collect states info from predicted state vector
  Eigen::VectorXd r_pre = X_pre.block<3, 1>(0, 0);
  Eigen::VectorXd v_pre = X_pre.block<3, 1>(3, 0);
  Eigen::VectorXd p_pre = X_pre.block<12, 1>(10, 0);
  Eigen::VectorXd bf_pre = X_pre.block<1, 1>(22, 0);
  Eigen::VectorXd bw_pre = X_pre.block<1, 1>(23, 0);

  Eigen::Quaterniond quaternion_pre(q_pre[0], q_pre[1], q_pre[2], q_pre[3]);
  quaternion_pre.normalize();
  Eigen::Matrix3d C_pre = quaternion_pre.toRotationMatrix();

  // Measured feet positions in the body frame
  Eigen::VectorXd s = Eigen::MatrixXd::Zero(3 * num_feet, 1);
  for (int i = 0; i < num_feet; i++) {
    // foot index i:(0 = FL, 1 = BL, 2 = FR, 3 = BR)
    Eigen::Vector3d joint_state_i;
    joint_state_i << X_pre[10 + i * 3], X_pre[10 + i * 3 + 1],
        X_pre[10 + i * 3 + 2];
    Eigen::Vector3d toe_body_pos;
    quadKD_->bodyToFootFKBodyFrame(i, joint_state_i, toe_body_pos);
    s.segment(i * 3, 3) = toe_body_pos;
  }
  // std::cout << "foot positions" << s << std::endl;

  // measurement residual
  Eigen::VectorXd y = Eigen::VectorXd::Zero(num_measure);
  for (int i = 0; i < num_feet; i++) {
    y.segment(i * 3, 3) =
        s.segment(i * 3, 3) - (C_pre * (p_pre.segment(i * 3, 3) - r_pre));
  }

  // Measurement jacobian
  H = Eigen::MatrixXd::Zero(num_measure, num_cov);
  H.block<18, 3>(0, 0) = -C_pre.replicate(6, 1);
  for (int i = 0; i < num_feet; i++) {
    Eigen::VectorXd vtemp = C_pre * (p_pre.segment(i * 3, 3) - r_pre);
    H.block<3, 3>(i * 3, 6) = this->calcSkewsym(vtemp);
  }
  H.block<3, 3>(0, 9) = C_pre;
  H.block<3, 3>(3, 12) = C_pre;
  H.block<3, 3>(6, 15) = C_pre;
  H.block<3, 3>(9, 18) = C_pre;

  // Measurement Noise
  R = Eigen::MatrixXd::Zero(num_measure, num_measure);

  // update Covariance
  Eigen::MatrixXd S = H * P_pre * H.transpose() + R;
  Eigen::MatrixXd K = P_pre * H.transpose() * S.inverse();
  Eigen::VectorXd delta_X = K * y;

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_cov, num_cov);
  P = (I - K * H) * P_pre;

  // update state
  X.segment(0, 6) = X_pre.segment(0, 6) + delta_X.segment(0, 6);
  Eigen::VectorXd delta_q = delta_X.segment(6, 3);
  Eigen::VectorXd q_upd = this->quaternionDynamics(delta_q, q_pre);
  X.segment(4, 6) = q_upd;
  X.segment(10, num_feet * 3) =
      X_pre.segment(10, num_feet * 3) + delta_X.segment(9, num_feet * 3);
  X.segment(22, 6) = X_pre.segment(22, 6) + delta_X.segment(21, 6);

  // set current state value to previous state
  last_X = X;

  // Update when I have good data, otherwise stay at the origin
  if (last_state_msg_ != NULL) {
    good_ground_truth_state = true;
  } else {
    good_ground_truth_state = false;
    X = X0;
    last_X = X0;
    a = Eigen::VectorXd::Zero(3);
  }

  /// publish new message
  new_state_est.header.stamp = ros::Time::now();

  // body
  new_state_est.body.header.stamp = ros::Time::now();
  new_state_est.body.pose.orientation.x = qk.x();
  new_state_est.body.pose.orientation.y = qk.y();
  new_state_est.body.pose.orientation.z = qk.z();
  new_state_est.body.pose.orientation.w = qk.w();
  new_state_est.body.pose.position.x = X[0];
  new_state_est.body.pose.position.y = X[1];
  new_state_est.body.pose.position.z = X[2];

  // joint
  new_state_est.joints.header.stamp = ros::Time::now();
  new_state_est.joints.name = {"0", "1", "2", "3", "4",  "5",
                               "6", "7", "8", "9", "10", "11"};
  new_state_est.joints.position = jk;
  new_state_est.joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  new_state_est.joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  return new_state_est;
}

Eigen::VectorXd EKFEstimator::quaternionDynamics(const Eigen::VectorXd& wdt,
                                                 const Eigen::VectorXd& q2v) {
  Eigen::VectorXd output = Eigen::VectorXd::Zero(4);

  double v = wdt.norm();
  Eigen::Vector3d q_xyz;
  if (v == 0) {
    q_xyz = Eigen::VectorXd::Zero(3);
  } else {
    q_xyz = sin(v / 2) * (wdt / v);
  }
  double q_w = cos(v / 2);
  Eigen::Quaterniond q1(q_w, q_xyz[0], q_xyz[1], q_xyz[2]);
  Eigen::Quaterniond q2(q2v[0], q2v[1], q2v[2], q2v[3]);
  q1.normalize();
  q2.normalize();
  Eigen::Quaterniond q3;
  q3.setIdentity();
  q3.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
  q3.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());
  q3.normalize();
  output << q3.w(), q3.x(), q3.y(), q3.z();
  return output;
}

Eigen::MatrixXd EKFEstimator::calcSkewsym(const Eigen::VectorXd& w) {
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(3, 3);
  output << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return output;
}

Eigen::MatrixXd EKFEstimator::calcRodrigues(const double& dt,
                                            const Eigen::VectorXd& w,
                                            const int& i) {
  Eigen::MatrixXd output = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd wdt = dt * w;
  double ang = wdt.norm();
  Eigen::VectorXd axis;
  if (ang == 0) {
    axis = Eigen::VectorXd::Zero(3);
  } else {
    axis = wdt / ang;
  }
  Eigen::MatrixXd w_cap = calcSkewsym(axis);
  switch (i) {
    case 0:
      output = output + sin(ang) * w_cap + (1 - cos(ang)) * (w_cap * w_cap);
      break;

    case 1:
      output = output + (1 - cos(ang)) * (w_cap / ang) +
               (ang - sin(ang)) * (w_cap * w_cap) / ang;
      break;

    case 2:
      output = output + (ang - sin(ang)) * (w_cap / (ang * ang)) +
               (((cos(ang) - 1) + (pow(ang, 2) / 2)) / (ang * ang)) *
                   (w_cap * w_cap);
      break;

    case 3:
      output = output +
               (cos(ang) - 1 + (pow(ang, 2) / 2)) / (pow(ang, 3)) * w_cap +
               ((sin(ang) - ang + (pow(ang, 3) / 6)) / (pow(ang, 3)) *
                (w_cap * w_cap));
      break;

    default:
      break;
  }
  return output;
}

void EKFEstimator::setNoise() {
  noise_acc = 0.01 * Eigen::MatrixXd::Identity(3, 3);
  noise_gyro = 0.01 * Eigen::MatrixXd::Identity(3, 3);
  bias_acc = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  bias_gyro = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  noise_feet = 0.01 * Eigen::MatrixXd::Identity(3, 3);
  noise_fk = 0.01 * Eigen::MatrixXd::Identity(3, 3);
  noise_encoder = 0.01;
}

void EKFEstimator::spin() {
  ros::Rate r(update_rate_);

  // initial state
  X0 = Eigen::VectorXd::Zero(num_state);
  X0 << -1.457778, 1.004244, 0.308681, 0, 0, 0, 1, 0, 0, 0, 0.767431, 1.591838,
      0.804742, 1.612967, 0.721895, 1.562485, 0.729919, 1.514980, -0.012140,
      -0.024205, 0.050132, 0.058434, 0, 0, 0, 0, 0, 0;
  X = X0;
  last_X = X0;

  // initial state covariance matrix
  P0 = 0.01 * Eigen::MatrixXd::Identity(num_cov, num_cov);
  P = P0;

  // set noise
  this->setNoise();

  // set gravity
  g = Eigen::VectorXd::Zero(3);
  g[2] = 9.8;

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
