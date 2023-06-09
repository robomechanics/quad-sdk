#include "robot_driver/estimators/comp_filter_estimator.h"

CompFilterEstimator::CompFilterEstimator() {}

void CompFilterEstimator::init(ros::NodeHandle& nh) {
  nh_ = nh;
  // load Comp_filter params
  quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_a", high_pass_a_);
  quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_b", high_pass_b_);
  quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_c", high_pass_c_);
  quad_utils::loadROSParam(nh_, "/robot_driver/high_pass_d", high_pass_d_);

  quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_a", low_pass_a_);
  quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_b", low_pass_b_);
  quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_c", low_pass_c_);
  quad_utils::loadROSParam(nh_, "/robot_driver/low_pass_d", low_pass_d_);
  high_pass_filter.A =
      Eigen::Map<Eigen::Matrix<double, 2, 2> >(high_pass_a_.data()).transpose();
  high_pass_filter.B =
      Eigen::Map<Eigen::Matrix<double, 1, 2> >(high_pass_b_.data()).transpose();
  high_pass_filter.C =
      Eigen::Map<Eigen::Matrix<double, 2, 1> >(high_pass_c_.data()).transpose();
  high_pass_filter.D =
      Eigen::Map<Eigen::Matrix<double, 1, 1> >(high_pass_d_.data()).transpose();
  high_pass_filter.x.resize(3);
  high_pass_filter.init = false;

  low_pass_filter.A =
      Eigen::Map<Eigen::Matrix<double, 2, 2> >(low_pass_a_.data()).transpose();
  low_pass_filter.B =
      Eigen::Map<Eigen::Matrix<double, 1, 2> >(low_pass_b_.data()).transpose();
  low_pass_filter.C =
      Eigen::Map<Eigen::Matrix<double, 2, 1> >(low_pass_c_.data()).transpose();
  low_pass_filter.D =
      Eigen::Map<Eigen::Matrix<double, 1, 1> >(low_pass_d_.data()).transpose();
  low_pass_filter.x.resize(3);
  low_pass_filter.init = false;
}

bool CompFilterEstimator::updateOnce(
    quad_msgs::RobotState& last_robot_state_msg_) {
  ros::Time state_timestamp = ros::Time::now();
  last_robot_state_msg_.body.twist.angular = last_imu_msg_.angular_velocity;
  last_robot_state_msg_.joints = last_joint_state_msg_;
  last_joint_state_msg_.header.stamp = state_timestamp;
  last_imu_msg_.header.stamp = state_timestamp;

  // Check if mocap data was received
  if (last_mocap_msg_ == NULL) {
    ROS_WARN_THROTTLE(1, "No body pose (mocap) recieved");
    return false;
  }
  // Copy mocap readings
  last_robot_state_msg_.body.pose.orientation =
      last_mocap_msg_->pose.orientation;
  last_robot_state_msg_.body.pose.position = last_mocap_msg_->pose.position;

  // IMU is in body frame
  Eigen::Vector3d acc;
  acc << last_imu_msg_.linear_acceleration.x,
      last_imu_msg_.linear_acceleration.y, last_imu_msg_.linear_acceleration.z;

  Eigen::Matrix3d rot;
  tf2::Quaternion q(
      last_mocap_msg_->pose.orientation.x, last_mocap_msg_->pose.orientation.y,
      last_mocap_msg_->pose.orientation.z, last_mocap_msg_->pose.orientation.w);
  q.normalize();
  tf2::Matrix3x3 m(q);
  Eigen::Vector3d rpy;
  m.getRPY(rpy[0], rpy[1], rpy[2]);
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

  // Fill in the rest of the state message (foot state and headers)
  quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
  quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp, "map",
                                 0);
  return true;
}

void CompFilterEstimator::mocapCallBackHelper(
    const geometry_msgs::PoseStamped::ConstPtr& msg,
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
