#include "robot_driver/estimators/ekf_estimator.hpp"

#include <quad_msgs/msg/contact_mode.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

EKFEstimator::EKFEstimator(rclcpp::Node::SharedPtr node,
                           const std::string& robot_ns,
                           std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : StateEstimator(node, robot_ns, quadKD),
      last_time_(node->get_clock()->now()) {}

void EKFEstimator::init() {
  // Load topic names from parameter server
  std::string grf_topic;
  quad_utils::loadROSParam(node_, "topics.control.grfs", grf_topic);

  // Load update rate and joint state time
  quad_utils::loadROSParam(node_, "robot_driver.update_rate", update_rate_);
  quad_utils::loadROSParamDefault(node_, "is_hardware", is_hardware_, true);
  quad_utils::loadROSParamDefault(node_, "robot_type", robot_name_,
                                  std::string("spirit"));

  // Load initial IMU bias from robot_driver yaml
  quad_utils::loadROSParam(node_, "robot_driver.bias_x", bias_x_);
  quad_utils::loadROSParam(node_, "robot_driver.bias_y", bias_y_);
  quad_utils::loadROSParam(node_, "robot_driver.bias_z", bias_z_);
  quad_utils::loadROSParam(node_, "robot_driver.bias_r", bias_r_);
  quad_utils::loadROSParam(node_, "robot_driver.bias_p", bias_p_);
  quad_utils::loadROSParam(node_, "robot_driver.bias_w", bias_w_);

  // Load noise terms from robot_driver yaml
  quad_utils::loadROSParam(node_, "robot_driver.na", na_);
  quad_utils::loadROSParam(node_, "robot_driver.nv", nv_);
  quad_utils::loadROSParam(node_, "robot_driver.ng", ng_);
  quad_utils::loadROSParam(node_, "robot_driver.ba", ba_);
  quad_utils::loadROSParam(node_, "robot_driver.bg", bg_);
  quad_utils::loadROSParam(node_, "robot_driver.nf", nf_);
  quad_utils::loadROSParam(node_, "robot_driver.nfk", nfk_);
  quad_utils::loadROSParam(node_, "robot_driver.ne", ne_);
  quad_utils::loadROSParam(node_, "robot_driver.nfh", nfh_);
  quad_utils::loadROSParam(node_, "robot_driver.P0", P0_);
  quad_utils::loadROSParam(node_, "robot_driver.contact_w", contact_w_);
  quad_utils::loadROSParam(node_, "robot_driver.thresh_out", thresh_out);
  quad_utils::loadROSParam(node_, "robot_driver.foot_radius", foot_radius);

  // Setup subscribers
  grf_sub_ = node_->create_subscription<quad_msgs::msg::GRFArray>(
      grf_topic, 1,
      [this](const quad_msgs::msg::GRFArray::SharedPtr msg) {
        last_grf_msg_ = msg;
      });
  // Contact topic - using default since not in global topics config
  contact_sub_ = node_->create_subscription<quad_msgs::msg::ContactMode>(
      robot_ns_ + "/contact_mode", 1,
      [this](const quad_msgs::msg::ContactMode::SharedPtr msg) {
        last_contact_msg_ = msg;
      });

  RCLCPP_INFO(node_->get_logger(), "Initialized EKF Estimator");
}

bool EKFEstimator::updateOnce(
    quad_msgs::msg::RobotState& last_robot_state_msg_) {
  rclcpp::Time state_timestamp = node_->now();
  last_joint_state_msg_.header.stamp = state_timestamp;
  last_imu_msg_.header.stamp = state_timestamp;
  if (is_hardware_) {
    last_robot_state_msg_.joints = last_joint_state_msg_;
  }

  X0 = Eigen::VectorXd::Zero(num_state);
  if (initialized) {
    if (is_hardware_) {
      setInitialState(last_robot_state_msg_);
      quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);
    }
  }

  // Run StepOnce when control mode is active
  // Note: In the original ROS1 code this was gated on control_mode_ == 1.
  // In ROS2 integration, robot_driver calls updateOnce() only when appropriate,
  // so we always run when initialized.
  if (initialized) {
    // Set start time on initialization
    last_time_ = node_->now();
    if (is_hardware_) {
      setInitialState(last_robot_state_msg_);
    }

    // Initialize filter
    P = P0_ * Eigen::MatrixXd::Identity(num_cov, num_cov);
    X0 << last_robot_state_msg_.body.pose.position.x,
        last_robot_state_msg_.body.pose.position.y,
        last_robot_state_msg_.body.pose.position.z,
        last_robot_state_msg_.body.twist.linear.x,
        last_robot_state_msg_.body.twist.linear.y,
        last_robot_state_msg_.body.twist.linear.z,
        last_robot_state_msg_.feet.feet[0].position.x,
        last_robot_state_msg_.feet.feet[0].position.y,
        last_robot_state_msg_.feet.feet[0].position.z - foot_radius,
        last_robot_state_msg_.feet.feet[1].position.x,
        last_robot_state_msg_.feet.feet[1].position.y,
        last_robot_state_msg_.feet.feet[1].position.z - foot_radius,
        last_robot_state_msg_.feet.feet[2].position.x,
        last_robot_state_msg_.feet.feet[2].position.y,
        last_robot_state_msg_.feet.feet[2].position.z - foot_radius,
        last_robot_state_msg_.feet.feet[3].position.x,
        last_robot_state_msg_.feet.feet[3].position.y,
        last_robot_state_msg_.feet.feet[3].position.z - foot_radius;
    X = X0;
    X_pre = X0;
    last_X = X0;
    initialized = false;
  }

  auto new_state_est = this->StepOnce();
  last_robot_state_msg_ = new_state_est;
  last_robot_state_msg_.joints = last_joint_state_msg_;

  // Update foot positions using forward kinematics
  quad_utils::fkRobotState(*quadKD_, last_robot_state_msg_);

  last_joint_state_msg_.header.stamp = state_timestamp;
  quad_utils::updateStateHeaders(last_robot_state_msg_, state_timestamp, "map",
                                 0);
  return true;
}

void EKFEstimator::setInitialState(
    quad_msgs::msg::RobotState& last_robot_state_msg_) {
  // Read IMU data directly from base class member
  Eigen::Vector3d fk = Eigen::Vector3d::Zero();
  Eigen::Vector3d wk = Eigen::Vector3d::Zero();
  Eigen::Quaterniond qk(1, 0, 0, 0);
  fk << last_imu_msg_.linear_acceleration.x,
      last_imu_msg_.linear_acceleration.y,
      last_imu_msg_.linear_acceleration.z;
  wk << last_imu_msg_.angular_velocity.x,
      last_imu_msg_.angular_velocity.y,
      last_imu_msg_.angular_velocity.z;
  qk.w() = last_imu_msg_.orientation.w;
  qk.x() = last_imu_msg_.orientation.x;
  qk.y() = last_imu_msg_.orientation.y;
  qk.z() = last_imu_msg_.orientation.z;
  qk.normalize();

  if (debug_ && last_mocap_msg_) {
    // SANITY-CHECK seed: initialize from the mocap (secondary) estimate.
    // Position starts at the true mocap location, and the world-yaw datum is
    // captured as (mocap_yaw - imu_yaw) and applied each step in StepOnce, so
    // the EKF heading is pinned to the mocap world frame at t=0. Lets us check
    // the heading against ground truth. Uses one mocap sample at init only.
    last_robot_state_msg_.body.pose.position = last_mocap_msg_->pose.position;
    last_robot_state_msg_.body.pose.orientation =
        last_mocap_msg_->pose.orientation;

    // setInitialState runs twice in updateOnce; capture the yaw offset once.
    if (!init_yaw_offset_set_) {
      tf2::Quaternion qm(last_mocap_msg_->pose.orientation.x,
                         last_mocap_msg_->pose.orientation.y,
                         last_mocap_msg_->pose.orientation.z,
                         last_mocap_msg_->pose.orientation.w);
      qm.normalize();
      double mr, mp, my;
      tf2::Matrix3x3(qm).getRPY(mr, mp, my);
      double ir, ip, iy;
      tf2::Matrix3x3(tf2::Quaternion(qk.x(), qk.y(), qk.z(), qk.w()))
          .getRPY(ir, ip, iy);
      init_yaw_offset_ = my - iy;
      init_yaw_offset_set_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "Seeding EKF from mocap: pos=(%.3f, %.3f, %.3f), "
                  "init_yaw_offset_ = %.2f deg",
                  last_mocap_msg_->pose.position.x,
                  last_mocap_msg_->pose.position.y,
                  last_mocap_msg_->pose.position.z,
                  init_yaw_offset_ * 180.0 / M_PI);
    }
  } else {
    // Fixed, mocap-free initial state: 0.3 m standing height at the origin,
    // identity orientation, zero velocity. Self-initializing, no mocap.
    if (debug_) {
      RCLCPP_WARN(node_->get_logger(),
                  "debug_ set but no mocap yet; using fixed (0,0,0.3) seed");
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Seeding EKF initial state to fixed pose (0, 0, 0.3), "
                  "identity orientation (mocap-free)");
    }
    last_robot_state_msg_.body.pose.position.x = 0.0;
    last_robot_state_msg_.body.pose.position.y = 0.0;
    last_robot_state_msg_.body.pose.position.z = 0.3;

    last_robot_state_msg_.body.pose.orientation.w = 1.0;
    last_robot_state_msg_.body.pose.orientation.x = 0.0;
    last_robot_state_msg_.body.pose.orientation.y = 0.0;
    last_robot_state_msg_.body.pose.orientation.z = 0.0;
  }

  last_robot_state_msg_.body.twist.linear.x = 0;
  last_robot_state_msg_.body.twist.linear.y = 0;
  last_robot_state_msg_.body.twist.linear.z = 0;

  last_robot_state_msg_.body.twist.angular.x = 0;
  last_robot_state_msg_.body.twist.angular.y = 0;
  last_robot_state_msg_.body.twist.angular.z = 0;
}

void EKFEstimator::setX(Eigen::VectorXd Xin) { X = Xin; }

void EKFEstimator::setP(Eigen::MatrixXd Pin) { P = Pin; }

Eigen::VectorXd EKFEstimator::getX() { return X; }

Eigen::VectorXd EKFEstimator::getX_pre() { return X_pre; }

quad_msgs::msg::RobotState EKFEstimator::StepOnce() {
  // Record start time
  rclcpp::Time start_time = node_->now();

  // Create skeleton message
  quad_msgs::msg::RobotState new_state_est;

  // Calculate dt
  double dt = (start_time - last_time_).seconds();
  last_time_ = start_time;

  /// Collect and process data
  // IMU reading linear acceleration
  Eigen::VectorXd fk = Eigen::VectorXd::Zero(3);
  // IMU reading angular acceleration
  Eigen::VectorXd wk = Eigen::VectorXd::Zero(3);
  // IMU orientation (w, x, y, z)
  Eigen::Quaterniond qk(1, 0, 0, 0);
  // Read IMU data directly from base class member
  fk << last_imu_msg_.linear_acceleration.x,
      last_imu_msg_.linear_acceleration.y,
      last_imu_msg_.linear_acceleration.z;
  wk << last_imu_msg_.angular_velocity.x,
      last_imu_msg_.angular_velocity.y,
      last_imu_msg_.angular_velocity.z;

  // Subtract IMU biases
  fk -= Eigen::Vector3d(bias_x_, bias_y_, bias_z_);
  wk -= Eigen::Vector3d(bias_r_, bias_p_, bias_w_);

  qk.w() = last_imu_msg_.orientation.w;
  qk.x() = last_imu_msg_.orientation.x;
  qk.y() = last_imu_msg_.orientation.y;
  qk.z() = last_imu_msg_.orientation.z;
  qk.normalize();

  // Joint data reading 3 joints * 4 legs
  Eigen::VectorXd jk = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd vk = Eigen::VectorXd::Zero(12);
  for (int i = 0; i < 12 && i < (int)last_joint_state_msg_.position.size();
       i++) {
    jk[i] = last_joint_state_msg_.position[i];
  }
  for (int i = 0; i < 12 && i < (int)last_joint_state_msg_.velocity.size();
       i++) {
    vk[i] = last_joint_state_msg_.velocity[i];
  }
  std::vector<double> jkVector(jk.data(), jk.data() + jk.rows() * jk.cols());

  // Extract rotation matrices
  Eigen::Matrix3d R_w_imu = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_imu_w = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_b_imu = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_imu_b = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d R_w_b = Eigen::Matrix3d::Zero();
  Eigen::Quaterniond qb;

  if (robot_name_ == "spirit" && is_hardware_) {
    // IMU on Spirit is flipped pi/2 along the Z axis clockwise
    R_b_imu << 0, -1, 0, -1, 0, 0, 0, 0, 1;
    R_imu_b = R_b_imu.transpose();
  } else {
    R_b_imu = Eigen::Matrix3d::Identity(3, 3);
    R_imu_b = Eigen::Matrix3d::Identity(3, 3);
    qb = qk;
  }

  R_w_imu = qk.toRotationMatrix();
  R_imu_w = R_w_imu.inverse();
  R_w_b = R_w_imu * R_imu_b;

  // Default: assume all feet in contact (safe for standing/walking)
  foot_contact_states = Eigen::VectorXd::Ones(num_feet);
  // Prefer the measured foot contact from the Unitree foot-force sensor
  // (pushed in by robot_driver via loadFootContactMsg) over the planner's
  // expected GRFs. contact_states is already thresholded against the tuned
  // foot_contact_threshold in robot_driver.
  if (foot_contact_received_ &&
      static_cast<int>(last_foot_contact_msg_.contact_states.size()) >=
          num_feet) {
    for (int i = 0; i < num_feet; i++) {
      foot_contact_states(i) =
          last_foot_contact_msg_.contact_states[i] ? 1.0 : 0.0;
    }
  }

  // Apply the constant world-yaw datum captured at init to the IMU
  // orientation: roll/pitch stay IMU (gravity-true), heading is pinned to the
  // world frame the position seed used. No live mocap dependence -- one yaw
  // sample at init only. Use qkw everywhere: output, R_w_imu, predict, update.
  tf2::Quaternion q_corr;
  q_corr.setRPY(0.0, 0.0, init_yaw_offset_);
  tf2::Quaternion q_w_tf =
      q_corr * tf2::Quaternion(qk.x(), qk.y(), qk.z(), qk.w());
  Eigen::Quaterniond qkw(q_w_tf.w(), q_w_tf.x(), q_w_tf.y(), q_w_tf.z());
  qkw.normalize();
  R_w_imu = qkw.toRotationMatrix();
  new_state_est.body.pose.orientation.w = qkw.w();
  new_state_est.body.pose.orientation.x = qkw.x();
  new_state_est.body.pose.orientation.y = qkw.y();
  new_state_est.body.pose.orientation.z = qkw.z();

  /// Prediction step
  // RCLCPP_INFO(node_->get_logger(), "Running EKF prediction step");
  Eigen::Vector3d u_dbg = R_w_imu * fk + Eigen::Vector3d(0, 0, -9.81);
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "\n  fk(body) = [%+.4f %+.4f %+.4f]  |fk|=%.4f"
      "\n  R_w_imu  = [%+.4f %+.4f %+.4f]"
      "\n             [%+.4f %+.4f %+.4f]"
      "\n             [%+.4f %+.4f %+.4f]"
      "\n  u = R*fk+g = [%+.4f %+.4f %+.4f]   (should be ~0 at rest)",
      fk(0), fk(1), fk(2), fk.norm(),
      R_w_imu(0, 0), R_w_imu(0, 1), R_w_imu(0, 2),
      R_w_imu(1, 0), R_w_imu(1, 1), R_w_imu(1, 2),
      R_w_imu(2, 0), R_w_imu(2, 1), R_w_imu(2, 2),
      u_dbg(0), u_dbg(1), u_dbg(2));

  this->predict(dt, fk, wk, R_w_imu);
  X = X_pre;
  P = P_pre;
  // last_X = X;
  // last_P = P;
  // RCLCPP_INFO(node_->get_logger(), "Running EKF update step");
  /// Update step
  this->update(jk, fk, vk, wk, qkw, R_w_imu);
  last_X = X;
  last_P = P;

  new_state_est.body.pose.position.x = X[0];
  new_state_est.body.pose.position.y = X[1];
  new_state_est.body.pose.position.z = X[2];

  new_state_est.body.twist.linear.x = X[3];
  new_state_est.body.twist.linear.y = X[4];
  new_state_est.body.twist.linear.z = X[5];

  // Angular velocity is in body frame (matches sim convention where
  // everything is world frame except angular velocity which is body frame)
  new_state_est.body.twist.angular.x = wk[0];
  new_state_est.body.twist.angular.y = wk[1];
  new_state_est.body.twist.angular.z = wk[2];

  // Joint state
  new_state_est.joints.name = {"8",  "0", "1", "9",  "2", "3",
                               "10", "4", "5", "11", "6", "7"};
  new_state_est.joints.position = jkVector;
  new_state_est.joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  new_state_est.joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  return new_state_est;
}

void EKFEstimator::predict(const double& dt, const Eigen::VectorXd& fk,
                           const Eigen::VectorXd& wk,
                           const Eigen::Matrix3d R_w_imu) {
  // Collect states from previous state vector
  Eigen::Vector3d r = last_X.segment(0, 3);
  Eigen::Vector3d v = last_X.segment(3, 3);

  // Generate linearized dynamics matrix (18 x 18)
  F = Eigen::MatrixXd::Identity(num_cov, num_cov);
  F.block<3, 3>(0, 3) = dt * Eigen::MatrixXd::Identity(3, 3);

  // Generate process prediction
  X_pre = Eigen::VectorXd::Zero(num_state);
  P_pre = Eigen::MatrixXd::Zero(num_state, num_state);

  // Generate estimation state transition (18 x 3)
  Eigen::MatrixXd B(num_state, 3);
  B.setZero();
  B.block<3, 3>(0, 0) = 0.5 * dt * dt * Eigen::MatrixXd::Identity(3, 3);
  B.block<3, 3>(3, 0) = dt * Eigen::MatrixXd::Identity(3, 3);

  // Generate process noise (18 x 18)
  Q = Eigen::MatrixXd::Identity(num_state, num_state);
  Q.block<3, 3>(0, 0) = na_ * dt / 20.0 * Eigen::MatrixXd::Identity(3, 3);
  Q.block<3, 3>(3, 3) =
      nv_ * dt * 9.81 / 20.0 * Eigen::MatrixXd::Identity(3, 3);
  // Resolve for Q depending on contact states
  for (int i = 0; i < num_feet; ++i) {
    Q.block<3, 3>(3 * i + 6, 3 * i + 6) =
        (1.0 + (1 - foot_contact_states[i]) * contact_w_) * nf_ * dt *
        Eigen::MatrixXd::Identity(3, 3);
  }

  // Generate control input
  g = Eigen::Vector3d(0, 0, -9.81);
  u = R_w_imu * fk + g;

  // Solve for process prediction state and covariance
  X_pre = (F * last_X) + B * u;
  P_pre = (F * P * F.transpose()) + Q;
}

void EKFEstimator::update(const Eigen::VectorXd& jk, const Eigen::VectorXd& fk,
                          const Eigen::VectorXd& vk, const Eigen::VectorXd& wk,
                          const Eigen::Quaterniond& qk,
                          const Eigen::Matrix3d& R_w_imu) {
  // Preallocate space and generate measurement matrix C
  Eigen::MatrixXd C(num_measure, num_state);
  C.setZero();
  R.setZero();
  for (int i = 0; i < num_feet; ++i) {
    C.block<3, 3>(3 * i, 0) = -Eigen::MatrixXd::Identity(3, 3);
    C.block<3, 3>(3 * i, 6 + 3 * i) = Eigen::MatrixXd::Identity(3, 3);
    C.block<3, 3>(12 + 3 * i, 3) = Eigen::MatrixXd::Identity(3, 3);
    C(24 + i, 8 + 3 * i) = 1.0;
  }

  // Preallocate space and generate measurement noise R
  R = Eigen::MatrixXd::Identity(num_measure, num_measure);
  for (int i = 0; i < num_feet; ++i) {
    R.block<3, 3>(3 * i, 3 * i) =
        nfk_ * (1.0 + (1.0 - foot_contact_states[i]) * contact_w_) *
        Eigen::MatrixXd::Identity(3, 3);
    R.block<3, 3>(12 + 3 * i, 12 + 3 * i) =
        ne_ * (1.0 + (1.0 - foot_contact_states[i]) * contact_w_) *
        Eigen::MatrixXd::Identity(3, 3);
    R(24 + i, 24 + i) =
        nfh_ * (1.0 + (1.0 - foot_contact_states[i]) * contact_w_);
  }

  // Generate measurement y from kinematics
  y = Eigen::VectorXd::Zero(num_measure);
  Eigen::VectorXd r_pre = X_pre.segment(0, 3);
  Eigen::VectorXd v_pre = X_pre.segment(3, 3);

  // Update pinocchio internal state before FK/Jacobian calls
  // Build a temporary RobotState to call updateDynamics
  quad_msgs::msg::RobotState temp_state;
  temp_state.body.pose.position.x = r_pre[0];
  temp_state.body.pose.position.y = r_pre[1];
  temp_state.body.pose.position.z = r_pre[2];
  temp_state.body.pose.orientation.w = qk.w();
  temp_state.body.pose.orientation.x = qk.x();
  temp_state.body.pose.orientation.y = qk.y();
  temp_state.body.pose.orientation.z = qk.z();
  temp_state.body.twist.linear.x = v_pre[0];
  temp_state.body.twist.linear.y = v_pre[1];
  temp_state.body.twist.linear.z = v_pre[2];
  temp_state.body.twist.angular.x = wk[0];
  temp_state.body.twist.angular.y = wk[1];
  temp_state.body.twist.angular.z = wk[2];
  temp_state.joints = last_joint_state_msg_;
  quad_utils::updateDynamics(*quadKD_, temp_state);

  // Compute Jacobian using QuadKD2 (uses internal pinocchio state)
  // nv = 12 joints + 6 base DOF = 18 for free-flyer quadruped
  static const int nv = 18;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, nv);
  quadKD_->getJacobianBodyAngVel(jacobian);

  // QuadKD2 Jacobian convention:
  //   foot_vel = J * [theta_dot(12), v_base_world(3), w_base_body(3)]
  //
  // For velocity measurement, compute foot velocity from joints and angular
  // velocity ONLY (exclude base linear velocity). For a contacting foot,
  // foot_vel_world = 0, so: v_body = -foot_vel_from_legs_and_angular.
  // This gives us a kinematic measurement of body velocity.
  Eigen::VectorXd q_dot_no_base_lin = Eigen::VectorXd::Zero(nv);
  q_dot_no_base_lin.head(12) = vk;         // joint velocities
  q_dot_no_base_lin.segment(15, 3) = wk;   // angular velocity (body frame)
  Eigen::VectorXd foot_vel_no_base = jacobian * q_dot_no_base_lin;

  for (int i = 0; i < num_feet; ++i) {
    // Solve for foot relative positions using QuadKD2
    Eigen::Vector3d toe_body_pos_body;
    toe_body_pos_body.setZero();
    quadKD_->bodyToFootFKBodyFrame(i, toe_body_pos_body);

    // toe_body_pos_body(2) -= foot_radius;
    // Rotate body-frame relative position into world frame to match C matrix
    y.segment(3 * i, 3) = R_w_imu * toe_body_pos_body;
    y(3 * i + 2) -= foot_radius;

    // Solve for foot heights
    // y(24 + i) =
    //     (1.0 - foot_contact_states[i]) * (r_pre(2) + toe_body_pos_body(2)) +
    //     foot_contact_states[i] * 0;
    y(24 + i) =
        (1.0 - foot_contact_states[i]) * (r_pre(2) + y(3 * i + 2)) +
        foot_contact_states[i] * 0;

    // Velocity measurement (C matrix extracts v_body from state):
    // Contacting foot: foot_vel_world = 0 = foot_vel_no_base + v_body
    //   => v_body_measured = -foot_vel_no_base  (Bloesch RSS 2012 eq. 16)
    // Non-contacting foot: no info, feed back prediction (zero innovation)
    Eigen::Vector3d v_measured = -foot_vel_no_base.segment(3 * i, 3);
    y.segment(12 + 3 * i, 3) =
        foot_contact_states[i] * v_measured +
        (1.0 - foot_contact_states[i]) * X_pre.segment(3, 3);
  }

  // Solve for error between measured Y residual and process residual
  error_y = y - (C * X_pre);

  // Innovation covariance
  S = C * P_pre * C.transpose() + R;
  S = 0.5 * (S + S.transpose());  // Ensure symmetry
  Serror_y = S.fullPivHouseholderQr().solve(error_y);

  // EKF filter equations - solve for Kalman gain
  X = X_pre + P_pre * C.transpose() * Serror_y;
  SC = S.fullPivHouseholderQr().solve(C);
  P = P_pre - P_pre * C.transpose() * SC * P_pre;
  P = 0.5 * (P + P.transpose());  // Ensure symmetry

  last_X = X;
  last_P = P;
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

