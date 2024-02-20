#include "robot_driver/estimators/ekf_estimator.h"

EKFEstimator::EKFEstimator() {}

void EKFEstimator::init(ros::NodeHandle& nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, imu_topic, contact_topic, local_plan_topic,
      grf_topic, state_estimate_topic, ground_truth_topic;

  nh_.param<std::string>("topics/state/joints", joint_encoder_topic,
                         "/state/joints");
  nh_.param<std::string>("topics/state/imu", imu_topic, "/state/imu");
  // nh_.param<std::string>("topics/control/grfs", grf_topic, "/control/grfs");
  nh_.param<std::string>("topics/state/estimate", state_estimate_topic,
                         "/state/estimate");

  nh_.param<std::string>("topics/contact_mode", contact_topic, "/contact_mode");
  // nh_.param<std::string>("topic/state/ground_truth", ground_truth_topic,
  //                       "/state/ground_truth");
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth",
                           ground_truth_topic);
  quad_utils::loadROSParam(nh_, "topics/control/grfs", grf_topic);
  quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  // Load Update Rate, Joint State Time
  quad_utils::loadROSParam(nh_, "/robot_driver/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "/robot_driver/joint_state_max_time",
                           joint_state_msg_time_diff_max_);
  quad_utils::loadROSParamDefault(nh_, "robot_driver/is_hardware", is_hardware_,
                                  true);

  // Load initial IMU bias from robot_driver yaml
  quad_utils::loadROSParam(nh_, "/robot_driver/bias_x", bias_x_);
  quad_utils::loadROSParam(nh_, "/robot_driver/bias_y", bias_y_);
  quad_utils::loadROSParam(nh_, "/robot_driver/bias_z", bias_z_);
  quad_utils::loadROSParam(nh_, "/robot_driver/bias_r", bias_r_);
  quad_utils::loadROSParam(nh_, "/robot_driver/bias_p", bias_p_);
  quad_utils::loadROSParam(nh_, "/robot_driver/bias_w", bias_w_);

  // Load noise terms from robot_driver yaml
  quad_utils::loadROSParam(nh_, "/robot_driver/na", na_);
  quad_utils::loadROSParam(nh_, "/robot_driver/ng", ng_);
  quad_utils::loadROSParam(nh_, "/robot_driver/ba", ba_);
  quad_utils::loadROSParam(nh_, "/robot_driver/bg", bg_);
  quad_utils::loadROSParam(nh_, "/robot_driver/nf", nf_);
  quad_utils::loadROSParam(nh_, "/robot_driver/nfk", nfk_);
  quad_utils::loadROSParam(nh_, "/robot_driver/ne", ne_);
  quad_utils::loadROSParam(nh_, "/robot_driver/P0", P0_);
  quad_utils::loadROSParam(nh_, "/robot_driver/contact_w", contact_w_);
  quad_utils::loadROSParam(nh_, "/robot_driver/thresh_out", thresh_out);
  quad_utils::loadROSParamDefault(nh_, "robot_driver/is_hardware", is_hardware_,
                                  true);

  // Setup subs
  grf_sub_ = nh_.subscribe(grf_topic, 1, &EKFEstimator::grfCallback, this);
  contact_sub_ =
      nh_.subscribe(contact_topic, 1, &EKFEstimator::contactCallback, this);
  // local_plan_sub_ =
  //     nh_.subscribe(local_plan_topic, 1, &EKFEstimator::localPlanCallback, this,
  //                   ros::TransportHints().tcpNoDelay(true));

  // In Sim, Grab IMU, Joint Encoders from Gazebo
  if (!is_hardware_) {
    imu_sub_ = nh_.subscribe(imu_topic, 1, &EKFEstimator::imuCallback, this);
    joint_encoder_sub_ = nh_.subscribe(
        joint_encoder_topic, 1, &EKFEstimator::jointEncoderCallback, this);
    state_ground_truth_sub_ = nh_.subscribe(
        ground_truth_topic, 1, &EKFEstimator::groundtruthCallback, this);
  }

  // QuadKD class
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
  ROS_INFO_STREAM("Initialized EKF Estimator");
}

bool EKFEstimator::updateOnce(quad_msgs::RobotState& last_robot_state_msg_) {
  if (is_hardware_) {
    // ROS_INFO_STREAM("Makes it Here in Update Once");
    last_robot_state_msg_.joints = last_joint_state_msg_;
    // ROS_INFO_STREAM("Populates Message");
  }
  // Define Initial State, Preallocated Space for State Vectors
  X0 = Eigen::VectorXd::Zero(num_state);

  // set noise
  this->setNoise();
  if (initialized) {
    setInitialState(last_robot_state_msg_);
    quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
  }
  // Run Step Once to Calculate Change in State Once Local Planner Starts
  if (last_local_plan_msg_ != nullptr) {
    if (initialized) {
      // Set Start Time on Initialization
      last_time = ros::Time::now();
      setInitialState(last_robot_state_msg_);
      // Initialize Filter
      P = P0_ * Eigen::MatrixXd::Identity(num_cov, num_cov);
      X0 << last_robot_state_msg_.body.pose.position.x,
          last_robot_state_msg_.body.pose.position.y,
          last_robot_state_msg_.body.pose.position.z,
          last_robot_state_msg_.body.twist.linear.x,
          last_robot_state_msg_.body.twist.linear.y,
          last_robot_state_msg_.body.twist.linear.z,
          last_robot_state_msg_.feet.feet[0].position.x,
          last_robot_state_msg_.feet.feet[0].position.y,
          last_robot_state_msg_.feet.feet[0].position.z,
          last_robot_state_msg_.feet.feet[1].position.x,
          last_robot_state_msg_.feet.feet[1].position.y,
          last_robot_state_msg_.feet.feet[1].position.z,
          last_robot_state_msg_.feet.feet[2].position.x,
          last_robot_state_msg_.feet.feet[2].position.y,
          last_robot_state_msg_.feet.feet[2].position.z,
          last_robot_state_msg_.feet.feet[3].position.x,
          last_robot_state_msg_.feet.feet[3].position.y,
          last_robot_state_msg_.feet.feet[3].position.z;
      X = X0;
      X_pre = X0;
      last_X = X0;
      initialized = false;
    }
    auto new_state_est = this->StepOnce();
    last_robot_state_msg_ = new_state_est;
    last_robot_state_msg_.joints = last_joint_state_msg_;
  }
  ros::Time state_timestamp = ros::Time::now();
  last_joint_state_msg_.header.stamp = state_timestamp;
  last_imu_msg_.header.stamp = state_timestamp;
  quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp,
                                   "map", 0);
  // Check State Headers are Updating
  // ROS_INFO_STREAM("HERE" << last_robot_state_msg_.header.stamp);
  // ROS_INFO_STREAM(last_robot_state_msg_);
  return true;
}

void EKFEstimator::setInitialState(
    quad_msgs::RobotState& last_robot_state_msg_) {
  // body
  // Grab this Directly from the IMU
  last_robot_state_msg_.body.pose.orientation.w = 1.0;
  last_robot_state_msg_.body.pose.orientation.x = 0.0;
  last_robot_state_msg_.body.pose.orientation.y = 0.0;
  last_robot_state_msg_.body.pose.orientation.z = 0.0;

  last_robot_state_msg_.body.pose.position.x = 0.0;
  last_robot_state_msg_.body.pose.position.y = 0.0;
  last_robot_state_msg_.body.pose.position.z = 0.27;

  last_robot_state_msg_.body.twist.linear.x = 0;
  last_robot_state_msg_.body.twist.linear.y = 0;
  last_robot_state_msg_.body.twist.linear.z = 0;

  last_robot_state_msg_.body.twist.angular.x = 0;
  last_robot_state_msg_.body.twist.angular.y = 0;
  last_robot_state_msg_.body.twist.angular.z = 0;

  return;
}

void EKFEstimator::groundtruthCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  last_robot_state_msg_ = *msg;
}

void EKFEstimator::jointEncoderCallback(
    const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = *msg;
}

void EKFEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = *msg;
}

void EKFEstimator::contactCallback(
    const quad_msgs::ContactMode::ConstPtr& msg) {
  last_contact_msg_ = msg;
}

void EKFEstimator::grfCallback(const quad_msgs::GRFArray::ConstPtr& msg) {
  last_grf_msg_ = msg;
}

// void EKFEstimator::localPlanCallback(
//     const quad_msgs::RobotPlan::ConstPtr& msg) {
//   last_local_plan_msg_ = msg;
// }

void EKFEstimator::setX(Eigen::VectorXd Xin) { X = Xin; }

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
  // std::cout << "this is dt" << dt << std::endl;

  /// Collect and Process Data
  // IMU reading linear acceleration
  Eigen::VectorXd fk = Eigen::VectorXd::Zero(3);
  // IMU reading angular acceleration
  Eigen::VectorXd wk = Eigen::VectorXd::Zero(3);
  // IMU orientation (w, x, y, z)
  Eigen::Quaterniond qk(1, 0, 0, 0);
  // if there is good imu data: read data from bag file
  this->readIMU(last_imu_msg_, fk, wk, qk);
  // qk.normalize();
  // Joint data reading 3 joints * 4 legs
  Eigen::VectorXd jk = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd vk = Eigen::VectorXd::Zero(12);
  this->readJointEncoder(last_joint_state_msg_, jk, vk);
  std::vector<double> jkVector(jk.data(), jk.data() + jk.rows() * jk.cols());
  /// Prediction Step
  // std::cout << "this is X before" << X.transpose() << std::endl;
  this->predict(dt, fk, wk, qk);
  // std::cout << "this is X predict" << X_pre.transpose() << std::endl;

  // for testing prediction step
  // X = X_pre;
  // P = P_pre;
  // last_X = X;

  /// Update Step
  this->update(jk, fk, vk, wk);  // Uncomment for Update Step
  // std::cout << "this is X update" << X.transpose() << std::endl;

  // last_X = X;

  /// publish new message
  // new_state_est.header.stamp = ros::Time::now();

  // body
  // Grab this Directly from the IMU
  new_state_est.body.pose.orientation.w = qk.w();
  new_state_est.body.pose.orientation.x = qk.x();
  new_state_est.body.pose.orientation.y = qk.y();
  new_state_est.body.pose.orientation.z = qk.z();

  new_state_est.body.pose.position.x = X[0];
  new_state_est.body.pose.position.y = X[1];
  new_state_est.body.pose.position.z = X[2];

  new_state_est.body.twist.linear.x = X[3];
  new_state_est.body.twist.linear.y = X[4];
  new_state_est.body.twist.linear.z = X[5];

  // joint
  // new_state_est.joints.header.stamp = ros::Time::now();
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
  // Generate Conversion Matrix to Rotate the Body Frame IMU into World Frame
  // Double Check if this needs a transpose or not
  // Eigen::Matrix3d C1 = (qk.toRotationMatrix()).transpose();
  C1 = (qk.toRotationMatrix());

  // q = Eigen::VectorXd::Zero(4);
  // q << qk.w(), qk.x(), qk.y(), qk.z();
  // //   //get better C matrix:
  // double angle = q.norm();
  // Eigen::Vector3d axis;

  // if (angle == 0) {
  //   axis = Eigen::VectorXd::Zero(3);
  // } else {
  //   axis = q / angle;
  // }
  // Eigen::Vector3d q_xyz = sin(angle / 2) * axis;
  // double q_w = cos(angle / 2);
  // Eigen::Quaterniond q1(q_w, q_xyz[0], q_xyz[1], q_xyz[2]);

  // Eigen::Matrix3d C = q1.toRotationMatrix().transpose();

  // Collect states info from previous state vector
  // Segment Syntax .segment(start_index, number of values)
  Eigen::VectorXd r = last_X.segment(0, 3);
  Eigen::VectorXd v = last_X.segment(3, 3);
  Eigen::VectorXd p = last_X.segment(10, 12);
  // Generate Linearized Dynamics Matrix (18 x 18)
  F = Eigen::MatrixXd::Identity(num_cov, num_cov);
  F.block<3, 3>(0, 3) = dt * Eigen::MatrixXd::Identity(3, 3);

  // Generate a Process Prediction X (18 x 1)
  X_pre = Eigen::VectorXd::Zero(num_state);
  P_pre = Eigen::MatrixXd::Zero(num_state, num_state);
  // Generate Estimation State Transition (18 x 3)
  Eigen::MatrixXd B(num_state, 3);
  B.setZero();
  B.block<3, 3>(0, 0) =
      0.5 * dt * dt *
      Eigen::MatrixXd::Identity(3, 3);  // Add acceleration into the process
  B.block<3, 3>(3, 0) = dt * Eigen::MatrixXd::Identity(3, 3);

  // Generate Estimation State Transition Noise (18 x 18)
  Q = Eigen::MatrixXd::Identity(num_state, num_state);
  Q.block<3, 3>(0, 0) = na_ * dt / 20.0 * Eigen::MatrixXd::Identity(3, 3);
  Q.block<3, 3>(3, 3) =
      na_ * dt * 9.81 / 20.0 * Eigen::MatrixXd::Identity(3, 3);

  // Resolve for Q depending on contact states
  for (int i = 0; i < num_feet; ++i) {
    if ((*last_grf_msg_).contact_states[i]) {
      Q.block<3, 3>(3 * i + 6, 3 * i + 6) =
          (1.0) * nf_ * dt * Eigen::MatrixXd::Identity(3, 3);
    } else {
      Q.block<3, 3>(3 * i + 6, 3 * i + 6) =
          (1.0 + contact_w_) * nf_ * dt * Eigen::MatrixXd::Identity(3, 3);
    }
  }

  // Generate Control Input U
  // May Need to add a Rotation Matrix to Compensate for Frames Here
  g = Eigen::Vector3d(0, 0, -9.81);
  u = C1 * fk + g;

  // Solve for Process Prediction State and Covariance
  X_pre = (F * last_X) + B * u;         // (18 x 1)
  P_pre = (F * P * F.transpose()) + Q;  // (18 x 18)
}

void EKFEstimator::update(const Eigen::VectorXd& jk, const Eigen::VectorXd& fk,
                          const Eigen::VectorXd& vk,
                          const Eigen::VectorXd& wk) {
  // debug for update step, set the predicted state to be ground truth:
  // Preallocate Space and Generate C
  Eigen::MatrixXd C(num_measure, num_state);
  C.setZero();
  for (int i = 0; i < num_feet; ++i) {
    C.block<3, 3>(3 * i, 0) = -Eigen::MatrixXd::Identity(3, 3);
    C.block<3, 3>(3 * i, 6 + 3 * i) = Eigen::MatrixXd::Identity(3, 3);
    C.block<3, 3>(12 + 3 * i, 3) = Eigen::MatrixXd::Identity(3, 3);
    C(24 + i, 8 + 3 * i) = 1.0;
  }

  // Preallocate Space and Generate R
  R = Eigen::MatrixXd::Identity(num_measure, num_measure);
  for (int i = 0; i < num_feet; ++i) {
    if ((*last_grf_msg_).contact_states[i]) {
      R.block<3, 3>(3 * i, 3 * i) = na_ * Eigen::MatrixXd::Identity(3, 3);
      R.block<3, 3>(12 + 3 * i, 12 + 3 * i) =
          na_ * Eigen::MatrixXd::Identity(3, 3);
      R(24 + i, 24 + i) = nf_;
    } else {
      R.block<3, 3>(3 * i, 3 * i) =
          na_ * (1.0 + contact_w_) * Eigen::MatrixXd::Identity(3, 3);
      R.block<3, 3>(12 + 3 * i, 12 + 3 * i) =
          na_ * (1.0 + contact_w_) * Eigen::MatrixXd::Identity(3, 3);
      R(24 + i, 24 + i) = nf_ * (1.0 + contact_w_);
    }
  }

  // Generate Measurement y from Kinematics, Extract Process Pos and Vel
  y = Eigen::VectorXd::Zero(num_measure);
  // Eigen::VectorXd r_pre = X_pre.segment(0,3);
  // Eigen::VectorXd v_pre = X_pre.segment(3, 3);
  Eigen::VectorXd r_pre = last_X.segment(0, 3);
  Eigen::VectorXd v_pre = last_X.segment(3, 3);

  // Update Measurement Attempt #2
  // Rewritten to Account for Rotation Matrix of IMU Frame
  Eigen::VectorXd joint_state(num_state);
  Eigen::VectorXd joint_velocity(num_state);
  Eigen::VectorXd joint_velocities(num_state);
  Eigen::VectorXd rbs(6);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  joint_state << jk, r_pre, v_pre;
  // joint_velocity << vk, v_pre, C1*fk;
  rbs << r_pre, v_pre;

  // Solve for Linear Foot Velocities in the Body Frame
  quadKD_->getJacobianBodyAngVel(joint_state, jacobian);
  Eigen::VectorXd lin_foot_vel;

  // Method for Joint Velocity
  // joint_velocities =
  //     math_utils::sdlsInv(jacobian.leftCols(12)) *
  //     (vk - jacobian.rightCols(6) * rbs.tail(6));

  lin_foot_vel = jacobian.leftCols(12) * vk;
  // lin_foot_vel = jacobian*joint_velocity;

  for (int i = 0; i < num_feet; ++i) {
    // Solve for Foot Relative Positions
    Eigen::Vector3d joint_state_i;
    Eigen::Vector3d toe_body_pos;
    toe_body_pos.setZero();
    joint_state_i = jk.segment(3 * i, 3);
    quadKD_->bodyToFootFKBodyFrame(i, joint_state_i, toe_body_pos);
    y.segment(3 * i, 3) = C1 * (toe_body_pos);

    // Solve for Foot Heights
    y(24 + i) = (1.0 - (*last_grf_msg_).contact_states[i]) *
                    (r_pre(2) + toe_body_pos(2)) +
                (*last_grf_msg_).contact_states[i] * 0;

    // Solve for Foot Relative Velocties
    Eigen::VectorXd leg_v(3);
    Eigen::MatrixXd acc;

    acc = calcSkewsym(wk);
    leg_v = -(lin_foot_vel.segment(3 * i, 3)) - (acc * toe_body_pos);
    y.segment(12 + 3 * i, 3) =
        (*last_grf_msg_).contact_states[i] * C1 * leg_v +
        last_X.segment(3, 3) * (1.0 - (*last_grf_msg_).contact_states[i]);
  }

  // Solve for Error between Measured Y Residual and Process Residual
  error_y = y - (C * X_pre);
  // ROS_INFO_STREAM("Innovation Norm" << error_y.norm());
  // ROS_INFO("Worked");
  // Skip Update if the Innovation is too High
  if (error_y.norm() < thresh_out) {
    S = C * P_pre * C.transpose() + R;
    // ROS_INFO_STREAM("This is S" << S);
    S = 0.5 *
        (S +
         S.transpose());  // Ensure that the Innovation Covariance is Symmetric
    Serror_y = S.fullPivHouseholderQr().solve(error_y);
    // ROS_INFO_STREAM("This is S Error" << Serror_y);
    // EKF Filter Equations, Solve for Kalman Gain

    X = X_pre + P_pre * C.transpose() * Serror_y;
    SC = S.fullPivHouseholderQr().solve(C);
    P = P_pre - P_pre * C.transpose() * SC * P_pre;
    P = 0.5 *
        (P + P.transpose());  // Ensure that the Covariance Matrix is Symmetric

    // Shuo Method to Reduce Positional Drift (Try)
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
      P.block<2, 16>(0, 2).setZero();
      P.block<16, 2>(2, 0).setZero();
      P.block<2, 2>(0, 0) /= 10.0;
    }
  } else {
    X = X_pre;
    P = P_pre;
  }
  last_X = X;
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
  Eigen::Quaterniond q3 = q1 * q2;
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
  output = pow(dt, sub) * output;
  return output;
}

void EKFEstimator::setNoise() {
  this->noise_acc = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_gyro = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->bias_acc = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->bias_gyro = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_feet = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_fk = 0.001 * Eigen::MatrixXd::Identity(3, 3);
  this->noise_encoder = 0.001;
  return;
}
