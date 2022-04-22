#include "ekf_estimator/ekf_estimator.h"

EKFEstimator::EKFEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic,
      state_estimate_topic;
  nh.param<std::string>("topics/state/joints", joint_encoder_topic,
                        "/state/joints");
  nh.param<std::string>("topics/state/imu", imu_topic, "/state/imu");
  nh.param<std::string>("topics/state/estimate", state_estimate_topic,
                        "/state/estimate");
  nh.param<std::string>("topics/contact_mode", contact_topic, "/contact_mode");
  nh.param<double>("ekf_estimator/update_rate", update_rate_, 200);
  nh.param<double>("ekf_estimator/joint_state_max_time",
                   joint_state_msg_time_diff_max_, 20);

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

void EKFEstimator::setX(Eigen::VectorXd Xin) {
  X = Xin;
  // std::cout << "this is X" << X << std::endl;
}

void EKFEstimator::setlast_X(Eigen::VectorXd Xin) { last_X = Xin; }

void EKFEstimator::setP(Eigen::MatrixXd Pin) { P = Pin; }

Eigen::VectorXd EKFEstimator::getX() { return X; }

Eigen::VectorXd EKFEstimator::getlastX() { return last_X; }

Eigen::VectorXd EKFEstimator::getX_pre() { return X_pre; }

quad_msgs::RobotState EKFEstimator::StepOnce() {
  // Record start time of function, used in verifying messages are not out of
  // date and in timing function
  ros::Time start_time = ros::Time::now();

  // Create skeleton message to send out
  quad_msgs::RobotState new_state_est;

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
  this->readIMU(last_imu_msg_, fk, wk, qk);

  // Joint data reading 3 joints * 4 legs
  Eigen::VectorXd jk = Eigen::VectorXd::Zero(12);
  this->readJointEncoder(last_joint_state_msg_, jk);
  std::vector<double> jkVector(jk.data(), jk.data() + jk.rows() * jk.cols());

  /// Prediction Step
  this->predict(dt, fk, wk, qk);

  // for testing prediction step
  // X = X_pre;
  // P = P_pre;
  // last_X = X;

  /// Update Step
  this->update(jk);

  // Update when I have good data, otherwise stay at the origin
  if (last_state_msg_ != NULL) {
    good_ground_truth_state = true;
  } else {
    good_ground_truth_state = false;
    X = X0;
    X_pre = X0;
    last_X = X0;
    P = P0;
    P_pre = P0;
  }

  /// publish new message
  new_state_est.header.stamp = ros::Time::now();

  // body
  new_state_est.body.header.stamp = ros::Time::now();
  new_state_est.body.pose.orientation.w = X[6];
  new_state_est.body.pose.orientation.x = X[7];
  new_state_est.body.pose.orientation.y = X[8];
  new_state_est.body.pose.orientation.z = X[9];

  new_state_est.body.pose.position.x = X[0];
  new_state_est.body.pose.position.y = X[1];
  new_state_est.body.pose.position.z = X[2];

  new_state_est.body.twist.linear.x = X[3];
  new_state_est.body.twist.linear.y = X[4];
  new_state_est.body.twist.linear.z = X[5];

  // joint
  new_state_est.joints.header.stamp = ros::Time::now();
  // '8', '0', '1', '9', '2', '3', '10', '4', '5', '11', '6', '7'
  new_state_est.joints.name = {"8",  "0", "1", "9",  "2", "3",
                               "10", "4", "5", "11", "6", "7"};
  new_state_est.joints.position = jkVector;
  new_state_est.joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  new_state_est.joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // feet

  return new_state_est;
}

void EKFEstimator::predict(const double& dt, const Eigen::VectorXd& fk,
                           const Eigen::VectorXd& wk,
                           const Eigen::Quaterniond& qk) {
  // calculate rotational matrix from world frame to body frame
  Eigen::Matrix3d C = qk.toRotationMatrix();

  // Collect states info from previous state vector
  Eigen::VectorXd r = last_X.segment(0, 3);
  Eigen::VectorXd v = last_X.segment(3, 3);
  Eigen::VectorXd q = last_X.segment(6, 4);
  Eigen::VectorXd p = last_X.segment(10, 12);
  Eigen::VectorXd bf = last_X.segment(22, 3);
  Eigen::VectorXd bw = last_X.segment(25, 3);

  // calculate linear acceleration set z acceleration to -9.8 for now
  // a is the corrected IMU linear acceleration (fk hat)
  Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
  a = fk - bf;
  a[2] = -9.8;

  g = Eigen::VectorXd::Zero(3);
  g[2] = 9.8;
  // calculate angular acceleration
  // w is the corrected IMU angular acceleration (wk hat)
  Eigen::VectorXd w = Eigen::VectorXd::Zero(3);
  w = wk - bw;

  // state prediction X_pre
  X_pre = Eigen::VectorXd::Zero(num_state);
  X_pre.segment(0, 3) = r + dt * v + dt * dt * 0.5 * (C.transpose() * a + g);
  X_pre.segment(3, 3) = v + dt * (C.transpose() * a + g);
  // quaternion updates
  Eigen::VectorXd wdt = dt * w;
  Eigen::VectorXd q_pre = this->quaternionDynamics(wdt, q);

  X_pre.segment(6, 4) = q_pre;
  X_pre.segment(10, 12) = p;
  X_pre.segment(22, 3) = bf;
  X_pre.segment(25, 3) = bw;

  // std::cout << "this is X_pre " << X_pre << std::endl;
  // Linearized Dynamics Matrix
  F = Eigen::MatrixXd::Identity(num_cov, num_cov);
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  Eigen::MatrixXd fkskew = this->calcSkewsym(a);
  Eigen::MatrixXd r0 = this->calcRodrigues(dt, w, 0);
  Eigen::MatrixXd r1 = this->calcRodrigues(dt, w, 1);
  Eigen::MatrixXd r2 = this->calcRodrigues(dt, w, 2);
  Eigen::MatrixXd r3 = this->calcRodrigues(dt, w, 3);

  F.block<3, 3>(0, 6) = -(dt * dt / 2) * C.transpose() * fkskew;
  F.block<3, 3>(0, 21) = -(dt * dt / 2) * C.transpose();
  F.block<3, 3>(3, 6) = -dt * C.transpose() * fkskew;
  F.block<3, 3>(3, 21) = -dt * C.transpose();
  F.block<3, 3>(6, 6) = r0.transpose();
  F.block<3, 3>(6, 24) = -1 * r1.transpose();

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
  Q.block<3, 3>(6, 6) = dt * noise_gyro + (r3 + r3.transpose()) * bias_gyro;
  Q.block<3, 3>(6, 24) = -r2.transpose() * bias_gyro;
  for (int i = 0; i < num_feet; i++) {
    Q.block<3, 3>(9 + i * 3, 9 + i * 3) = dt * C.transpose() * noise_feet * C;
    // determine foot contact and set noise

    // if (p[i * 3 + 2] < 0.02) {
    //   Q.block<3, 3>(9 + i * 3, 9 + i * 3) = dt * C.transpose() * noise_feet *
    //   C;
    // } else {
    //   Q.block<3, 3>(9 + i * 3, 9 + i * 3) =
    //       1000 * dt * C.transpose() * noise_feet * C;
    // }
  }

  Q.block<3, 3>(21, 0) = -1 * (pow(dt, 3) / 6) * bias_acc * C;
  Q.block<3, 3>(21, 3) = -1 * (pow(dt, 2) / 2) * bias_acc * C;
  Q.block<3, 3>(21, 21) = dt * bias_acc;
  Q.block<3, 3>(24, 6) = -1 * bias_gyro * r2;
  Q.block<3, 3>(24, 24) = dt * bias_gyro;
  Q = Eigen::MatrixXd::Zero(num_cov, num_cov);
  // Covariance update

  P_pre = (F * P * F.transpose()) + Q;

  // std::cout << "this is Q " << Q << std::endl;
  std::cout << "this is P_pre " << P_pre << std::endl;
}

void EKFEstimator::update(const Eigen::VectorXd& jk) {
  // Collect states info from predicted state vector
  Eigen::VectorXd r_pre = X_pre.segment(0, 3);
  Eigen::VectorXd v_pre = X_pre.segment(3, 3);
  Eigen::VectorXd q_pre = X_pre.segment(6, 4);
  Eigen::VectorXd p_pre = X_pre.segment(10, 12);
  Eigen::VectorXd bf_pre = X_pre.segment(22, 3);
  Eigen::VectorXd bw_pre = X_pre.segment(25, 3);

  Eigen::Quaterniond quaternion_pre(q_pre[0], q_pre[1], q_pre[2], q_pre[3]);
  quaternion_pre.normalize();
  Eigen::Matrix3d C_pre = quaternion_pre.toRotationMatrix();

  // Measured feet positions in the body frame
  Eigen::VectorXd s = Eigen::VectorXd::Zero(3 * num_feet);
  for (int i = 0; i < num_feet; i++) {
    // foot index i:(0 = FL, 1 = BL, 2 = FR, 3 = BR)
    // FL: 8 0 1, BL: 9 2 3, FR: 10 4 5, BR: 11 6 7
    // joint_state_i << jk[8 + i], jk[i * 2], jk[i * 2 + 1];
    // FL: 0 1 2 , BL: 3 4 5, FR: 6 7 8, BR: 11 6 7
    Eigen::Vector3d joint_state_i;
    joint_state_i = jk.segment(3 * i, 3);

    Eigen::Vector3d toe_body_pos;
    quadKD_->bodyToFootFKBodyFrame(i, joint_state_i, toe_body_pos);
    s.segment(i * 3, 3) = toe_body_pos;
  }

  // std::cout << "foot positions" << s << std::endl;

  // measurement residual (12 * 1)
  Eigen::VectorXd y = Eigen::VectorXd::Zero(num_measure);
  for (int i = 0; i < num_feet; i++) {
    y.segment(i * 3, 3) =
        s.segment(i * 3, 3) - (C_pre * (p_pre.segment(i * 3, 3) - r_pre));
  }
  std::cout << "this is the residual " << std::endl;
  std::cout << y << std::endl;

  // Measurement jacobian (12 * 27)
  H = Eigen::MatrixXd::Zero(num_measure, num_cov);
  H.block<3, 3>(0, 0) = C_pre;
  H.block<3, 3>(3, 0) = C_pre;
  H.block<3, 3>(6, 0) = C_pre;
  H.block<3, 3>(9, 0) = C_pre;

  for (int i = 0; i < num_feet; i++) {
    Eigen::VectorXd vtemp = C_pre * (p_pre.segment(i * 3, 3) - r_pre);
    H.block<3, 3>(i * 3, 6) = this->calcSkewsym(vtemp);
    H.block<3, 3>(i * 3, 9 + i * 3) = C_pre;
  }

  // Measurement Noise Matrix (12 * 12)
  R = 0.0000 * Eigen::MatrixXd::Identity(num_measure, num_measure);

  // // Define vectors for state positions
  Eigen::VectorXd state_positions(18);
  // Load state positions
  state_positions << jk, r_pre, v_pre;
  // Compute jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  quadKD_->getJacobianBodyAngVel(state_positions, jacobian);

  for (int i = 0; i < num_feet; i++) {
    Eigen::MatrixXd jtemp = jacobian.block<3, 3>(i * 3, i * 3);
    R.block<3, 3>(i * 3, i * 3) =
        noise_fk + jtemp * noise_encoder * jtemp.transpose();
  }

  // update Covariance (12 * 12)
  Eigen::MatrixXd S = H * P_pre * H.transpose() + R;
  // K (27 * 12)
  Eigen::MatrixXd K = P_pre * H.transpose() * S.inverse();
  Eigen::VectorXd delta_X = K * y;

  // std::cout << "kalmin gain " << K << std::endl;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_cov, num_cov);

  P = (I - K * H) * P_pre;

  // update state
  X.segment(0, 3) = r_pre + delta_X.segment(0, 3);
  X.segment(3, 3) = v_pre + delta_X.segment(3, 3);
  Eigen::VectorXd delta_q = delta_X.segment(6, 3);
  Eigen::VectorXd q_upd = this->quaternionDynamics(delta_q, q_pre);
  X.segment(6, 4) = q_pre;
  X.segment(10, num_feet * 3) = p_pre + delta_X.segment(9, num_feet * 3);
  X.segment(22, 3) = bf_pre + delta_X.segment(21, 3);
  X.segment(25, 3) = bw_pre + delta_X.segment(24, 3);

  // set current state value to previous statex
  last_X = X;

  // std::cout << "this is y" << y << std::endl;
  // std::cout << "this is P  after the measurement" << P << std::endl;
  // std::cout << "this is X after the measurement " << X << std::endl;
  // std::cout << "this is r0" << r0 << std::endl;
  // std::cout << "this is r1" << r1 << std::endl;
  // std::cout << "this is r2" << r2 << std::endl;
  // std::cout << "this is F" << F << std::endl;
  // std::cout << "this is Q" << Q << std::endl;
  // std::cout << "this is K" << K << std::endl;
  // std::cout << "this is H" << H << std::endl;
  // std::cout << "this is deltaX" << delta_X << std::endl;
  // std::cout << "this is S" << S << std::endl;
  // std::cout << "this is S inverse" << S.inverse() << std::endl;
  // std::cout << "this is x y z" << X.segment(0, 3) << std::endl;
}

void EKFEstimator::readIMU(const sensor_msgs::Imu::ConstPtr& last_imu_msg_,
                           Eigen::VectorXd& fk, Eigen::VectorXd& wk,
                           Eigen::Quaterniond& qk) {
  if (last_imu_msg_ != NULL) {
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
}

void EKFEstimator::readJointEncoder(
    const sensor_msgs::JointState::ConstPtr& last_joint_state_msg_,
    Eigen::VectorXd& jk) {
  if (last_joint_state_msg_ != NULL) {
    for (int i = 0; i < 12; i++) {
      jk[i] = (*last_joint_state_msg_).position[i];
    }
  } else {
    // nominal joint encoder values
    // initial value for spirit_walking_005.bag
    jk << -0.085515, 0.068447, 0.050970, -0.090167, 0.067919, 0.041212,
        0.074548, 0.035950, 0.043139, 0.085534, 0.062638, 0.057826;

    // initial value for spirit_walking_002.bag
    // jk << 0.767431, 1.591838, 0.804742, 1.612967, 0.721895, 1.562485,
    // 0.729919,
    //     1.514980, -0.012140, -0.024205, 0.050132, 0.058434;
  }
}

Eigen::VectorXd EKFEstimator::quaternionDynamics(const Eigen::VectorXd& wdt,
                                                 const Eigen::VectorXd& q2v) {
  Eigen::VectorXd output = Eigen::VectorXd::Zero(4);

  double angle = wdt.norm();
  Eigen::Vector3d axis;
  if (angle == 0) {
    axis = Eigen::VectorXd::Zero(3);
  } else {
    axis = wdt / angle;
  }
  Eigen::Vector3d q_xyz = sin(angle / 2) * axis;
  double q_w = cos(angle / 2);
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
                                            const int& sub) {
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
  switch (sub) {
    case 0:
      output = output + sin(ang) * w_cap + (1 - cos(ang)) * (w_cap * w_cap);
      break;

    case 1:
      if (ang == 0) {
        break;
      } else {
        output = output + (1 - cos(ang)) * (w_cap / ang) +
                 (ang - sin(ang)) * (w_cap * w_cap) / ang;
        break;
      }

    case 2:
      if (ang == 0) {
        break;
      } else {
        output = output + (ang - sin(ang)) * (w_cap / (ang * ang)) +
                 (((cos(ang) - 1) + (pow(ang, 2) / 2)) / (ang * ang)) *
                     (w_cap * w_cap);
        break;
      }

    case 3:
      if (ang == 0) {
        break;
      } else {
        output = output +
                 (cos(ang) - 1 + (pow(ang, 2) / 2)) / (pow(ang, 3)) * w_cap +
                 ((sin(ang) - ang + (pow(ang, 3) / 6)) / (pow(ang, 3)) *
                  (w_cap * w_cap));
        break;
      }
    default:
      break;
  }
  return output;
}

void EKFEstimator::setNoise() {
  this->noise_acc = 0.05 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_gyro = 0.05 * Eigen::MatrixXd::Identity(3, 3);
  this->bias_acc = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->bias_gyro = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_feet = 0.03 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_fk = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_encoder = 0.01;
  return;
}

void EKFEstimator::spin() {
  ros::Rate r(update_rate_);

  // initial state
  X0 = Eigen::VectorXd::Zero(num_state);
  // initial state for spirit_walking_005.bag
  X0 << 2.731990, 0.037111, 0.063524, -0.002654, 0.013438, 0.005384, -0.999192,
      -0.023452, 0.000798, -0.032635, 2.947184, 0.221952, 0.053124, 2.495493,
      0.192468, 0.053249, 2.969032, -0.116851, 0.039909, 2.517641, -0.145698,
      0.034415, -0.08, -0.06, 0, 0, 0, 0;

  // initial state for spirit_walking_002.bag
  // X0 << -1.457778, 1.004244, 0.308681, 0, 0, 0, 0.998927, 0.004160,
  // -0.003017,
  //     -0.046032, -1.251841, 1.185387, 0.012734, -1.695057, 1.148678,
  //     0.007092, -1.236598, 0.861900, 0.016119, -1.678741, 0.831065, 0.020651,
  //     0, 0, 0, 0, 0, 0;
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
    quad_msgs::RobotState new_state_est = this->StepOnce();

    // Publish new state estimate
    state_estimate_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}
