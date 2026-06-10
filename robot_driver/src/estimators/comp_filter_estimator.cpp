#include "robot_driver/estimators/comp_filter_estimator.hpp"

CompFilterEstimator::CompFilterEstimator(
    rclcpp::Node::SharedPtr node, const std::string& robot_ns,
    std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : StateEstimator(node, robot_ns, quadKD) {}

void CompFilterEstimator::init() {
  // load Comp_filter params
  quad_utils::loadROSParam(node_, "robot_driver.high_pass_a", high_pass_a_);
  quad_utils::loadROSParam(node_, "robot_driver.high_pass_b", high_pass_b_);
  quad_utils::loadROSParam(node_, "robot_driver.high_pass_c", high_pass_c_);
  quad_utils::loadROSParam(node_, "robot_driver.high_pass_d", high_pass_d_);

  quad_utils::loadROSParam(node_, "robot_driver.low_pass_a", low_pass_a_);
  quad_utils::loadROSParam(node_, "robot_driver.low_pass_b", low_pass_b_);
  quad_utils::loadROSParam(node_, "robot_driver.low_pass_c", low_pass_c_);
  quad_utils::loadROSParam(node_, "robot_driver.low_pass_d", low_pass_d_);
  high_pass_filter.A =
      Eigen::Map<Eigen::Matrix<double, 2, 2>>(high_pass_a_.data()).transpose();
  high_pass_filter.B =
      Eigen::Map<Eigen::Matrix<double, 1, 2>>(high_pass_b_.data()).transpose();
  high_pass_filter.C =
      Eigen::Map<Eigen::Matrix<double, 2, 1>>(high_pass_c_.data()).transpose();
  high_pass_filter.D =
      Eigen::Map<Eigen::Matrix<double, 1, 1>>(high_pass_d_.data()).transpose();
  high_pass_filter.x.resize(3);
  high_pass_filter.init = false;

  low_pass_filter.A =
      Eigen::Map<Eigen::Matrix<double, 2, 2>>(low_pass_a_.data()).transpose();
  low_pass_filter.B =
      Eigen::Map<Eigen::Matrix<double, 1, 2>>(low_pass_b_.data()).transpose();
  low_pass_filter.C =
      Eigen::Map<Eigen::Matrix<double, 2, 1>>(low_pass_c_.data()).transpose();
  low_pass_filter.D =
      Eigen::Map<Eigen::Matrix<double, 1, 1>>(low_pass_d_.data()).transpose();
  low_pass_filter.x.resize(3);
  low_pass_filter.init = false;

  // Optional lateral drift rejection (for straight-line / beam walking).
  quad_utils::loadROSParamDefault(node_, "robot_driver.lateral_drift_reject",
                                  lat_drift_reject_, false);
  quad_utils::loadROSParamDefault(node_, "robot_driver.lateral_drift_tau",
                                  lat_drift_tau_, 0.01);
  y_ref_init_ = false;
}

bool CompFilterEstimator::updateOnce(
    quad_msgs::msg::RobotState& last_robot_state_msg_) {
  rclcpp::Time state_timestamp = node_->now();
  last_robot_state_msg_.body.twist.angular = last_imu_msg_.angular_velocity;
  last_robot_state_msg_.joints = last_joint_state_msg_;
  last_joint_state_msg_.header.stamp = state_timestamp;
  last_imu_msg_.header.stamp = state_timestamp;

  // Check if mocap data was received
  if (!last_mocap_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "No body pose (mocap) recieved");
    return false;
  }
  // Body position from mocap (absolute map-frame body origin)
  last_robot_state_msg_.body.pose.position = last_mocap_msg_->pose.position;

  // Optional lateral drift rejection (for straight-line / beam walking). The
  // mocap body_y carries a slow, motion-induced solve error (trunk markers
  // occluded by the swinging legs) that the controller mistakes for real
  // lateral drift -> asymmetric footholds -> tipping. When the robot is known
  // to walk straight (y_true ~ const), estimate that slow drift as a low-pass
  // of (mocap_y - y_ref) and subtract it, so body_y holds near its start value
  // while genuine fast lateral sway is preserved. OFF by default; only valid
  // when the robot does not intentionally translate sideways.
  if (lat_drift_reject_) {
    double mocap_y = last_mocap_msg_->pose.position.y;
    if (!y_ref_init_) {
      y_ref_ = mocap_y;
      y_drift_lp_ = 0.0;
      y_ref_init_ = true;
    }
    y_drift_lp_ += lat_drift_tau_ * ((mocap_y - y_ref_) - y_drift_lp_);
    last_robot_state_msg_.body.pose.position.y = mocap_y - y_drift_lp_;
  }

  // Body attitude: roll/pitch from the IMU (gravity-referenced), yaw from mocap
  // (absolute map-frame heading). comp_filter previously copied the full
  // orientation from mocap, but a residual ~0.7-1.0 deg roll/pitch offset in the
  // Motive rigid-body calibration biased the controller's COM/gravity projection
  // -> consistent left-heavy load, asymmetric hip abduction, and lateral tipping
  // while walking. The Go2 onboard IMU gives gravity-correct roll/pitch (already
  // used directly as body attitude on the learned-controller path in
  // robot_driver.cpp); mocap still supplies absolute yaw, which the IMU lacks.
  double roll_imu, pitch_imu, yaw_imu, roll_m, pitch_m, yaw_mocap;
  tf2::Quaternion q_imu(
      last_imu_msg_.orientation.x, last_imu_msg_.orientation.y,
      last_imu_msg_.orientation.z, last_imu_msg_.orientation.w);
  q_imu.normalize();
  tf2::Matrix3x3(q_imu).getRPY(roll_imu, pitch_imu, yaw_imu);

  tf2::Quaternion q_mocap(
      last_mocap_msg_->pose.orientation.x, last_mocap_msg_->pose.orientation.y,
      last_mocap_msg_->pose.orientation.z, last_mocap_msg_->pose.orientation.w);
  q_mocap.normalize();
  tf2::Matrix3x3(q_mocap).getRPY(roll_m, pitch_m, yaw_mocap);

  tf2::Quaternion q_fused;
  q_fused.setRPY(roll_imu, pitch_imu, yaw_mocap);
  q_fused.normalize();
  last_robot_state_msg_.body.pose.orientation.x = q_fused.x();
  last_robot_state_msg_.body.pose.orientation.y = q_fused.y();
  last_robot_state_msg_.body.pose.orientation.z = q_fused.z();
  last_robot_state_msg_.body.pose.orientation.w = q_fused.w();

  // IMU is in body frame
  Eigen::Vector3d acc;
  acc << last_imu_msg_.linear_acceleration.x,
      last_imu_msg_.linear_acceleration.y, last_imu_msg_.linear_acceleration.z;

  // Rotate the body-frame acceleration into the world frame using the fused
  // (gravity-correct) attitude for the velocity high-pass branch.
  Eigen::Matrix3d rot;
  Eigen::Vector3d rpy;
  rpy << roll_imu, pitch_imu, yaw_mocap;
  quadKD_->getRotationMatrix(rpy, rot);
  acc = rot * acc;

  // Ignore gravity
  acc[2] -= 9.81;

  if (!high_pass_filter.init) {
    // Init filter, we want to make sure that if the input is zero, the
    // output velocity is zero and the state remains the same
    for (size_t i = 0; i < 3; i++) {
      high_pass_filter.x.at(i) << 0, 0;
    }
    high_pass_filter.init = true;
  }

  // Apply filter
  for (size_t i = 0; i < 3; i++) {
    // Compute outputs
    imu_vel_estimate_(i) = (high_pass_filter.C * high_pass_filter.x.at(i) +
                            high_pass_filter.D * acc(i))(0, 0);

    // Compute states
    high_pass_filter.x.at(i) = high_pass_filter.A * high_pass_filter.x.at(i) +
                               high_pass_filter.B * acc(i);
  }

  // Complementary filter
  vel_estimate_ = imu_vel_estimate_ + mocap_vel_estimate_;
  quad_utils::Eigen3ToVector3Msg(vel_estimate_,
                                 last_robot_state_msg_.body.twist.linear);

  // Keep the lateral velocity consistent with the held position when rejecting
  // drift: drop the mocap low-pass (the slow drift) and keep only the IMU
  // high-pass (real fast sway), so the capture-point step does not chase the
  // phantom lateral velocity.
  if (lat_drift_reject_) {
    last_robot_state_msg_.body.twist.linear.y = imu_vel_estimate_(1);
  }

  // Fill in the rest of the state message (foot state and headers)
  quad_utils::updateDynamics(*quadKD_, last_robot_state_msg_);
  quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
  quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp, "map",
                                 0);
  return true;
}

void CompFilterEstimator::mocapCallBackHelper(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const Eigen::Vector3d& pos) {
  if (low_pass_filter.init) {
    // Apply filter
    for (size_t i = 0; i < 3; i++) {
      // Compute outputs
      mocap_vel_estimate_(i) = (low_pass_filter.C * low_pass_filter.x.at(i) +
                                low_pass_filter.D * pos(i))(0, 0);

      // Compute states
      low_pass_filter.x.at(i) = low_pass_filter.A * low_pass_filter.x.at(i) +
                                low_pass_filter.B * pos(i);
    }

  } else {
    // Init filter, we want to ensure that if the next reading is the same, the
    // output speed should be zero and the filter state remains the same
    Eigen::Matrix<double, 3, 2> left;
    left.topRows(2) = low_pass_filter.A - Eigen::Matrix2d::Identity();
    left.bottomRows(1) = low_pass_filter.C;

    Eigen::Matrix<double, 3, 1> right;
    right.topRows(2) = -low_pass_filter.B;
    right.bottomRows(1) = -low_pass_filter.D;

    for (size_t i = 0; i < 3; i++) {
      low_pass_filter.x.at(i) =
          left.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
              .solve(right * pos(i));
    }

    low_pass_filter.init = true;
  }
}
