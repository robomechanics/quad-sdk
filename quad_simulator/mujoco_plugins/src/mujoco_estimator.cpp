#include "mujoco_estimator/mujoco_estimator.hpp"

#include <cmath>

using std::placeholders::_1;

MujocoEstimator::MujocoEstimator() : Node("mujoco_estimator") {
  // quad-sdk joint order (mirrors Gazebo plugin joint_names)
  quadsdk_joint_order_ = {"8",  "0", "1", "9",  "2", "3",
                          "10", "4", "5", "11", "6", "7"};

  // Odometry subscriber — MuJoCo publishes world-frame position/linear vel,
  // body-frame angular vel for the free joint (see mujoco_system_interface.cpp)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/robot_1/odom", 10, std::bind(&MujocoEstimator::odomCallback, this, _1));

  // Joint states subscriber — joint_state_broadcaster publishes RELIABLE +
  // VOLATILE
  rclcpp::QoS joint_qos(10);
  joint_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  joint_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot_1/joint_states", joint_qos,
      std::bind(&MujocoEstimator::jointCallback, this, _1));

  // Subscribe to robot_description to build QuadKD2 — mirrors Gazebo plugin
  // urdf_sub_ with transient_local QoS so we get the latched message.
  rclcpp::QoS urdf_qos(10);
  urdf_qos.transient_local().reliable();
  urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_1/robot_description", urdf_qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (urdf_received_) return;
        urdf_received_ = true;

        // Create a helper node that holds robot_description for QuadKD2
        rclcpp::NodeOptions opts;
        opts.automatically_declare_parameters_from_overrides(true);
        kd_node_ =
            std::make_shared<rclcpp::Node>("mujoco_ground_truth_kd", opts);

        kd_node_->declare_parameter<std::string>("robot_description",
                                                 msg->data);

        try {
          quadKD_ = std::make_shared<quad_utils::QuadKD2>(kd_node_);
          RCLCPP_INFO(this->get_logger(),
                      "QuadKD2 initialized from robot_description.");
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "QuadKD2 init failed: %s", e.what());
          urdf_received_ = false;
        }
      });

  // World-frame state — mirrors ground_truth_state_pub_ in Gazebo plugin
  state_pub_ = this->create_publisher<quad_msgs::msg::RobotState>(
      "/robot_1/state/ground_truth", 10);

  // Body-frame state — mirrors ground_truth_state_body_frame_pub_ in Gazebo
  // plugin
  state_body_frame_pub_ = this->create_publisher<quad_msgs::msg::RobotState>(
      "/robot_1/state/ground_truth_body_frame", 10);

  // 500 Hz publish timer
  timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / 500.0),
                              std::bind(&MujocoEstimator::publishState, this));

  RCLCPP_INFO(this->get_logger(), "MuJoCo Estimator initialized");
}

void MujocoEstimator::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = msg;
}

void MujocoEstimator::jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  latest_joints_ = msg;
}

// Rotate a vector from world frame into body frame using q_bw = q_wb^{-1}.
// Mirrors: const gz::math::Vector3d w_b = q_bw * w_w;
std::array<double, 3> MujocoEstimator::rotateWorldToBody(double qw, double qx,
                                                         double qy, double qz,
                                                         double vx, double vy,
                                                         double vz) {
  // Conjugate (inverse for unit quaternion) gives q_bw
  double iw = qw;
  double ix = -qx;
  double iy = -qy;
  double iz = -qz;

  double tx = 2.0 * (iy * vz - iz * vy);
  double ty = 2.0 * (iz * vx - ix * vz);
  double tz = 2.0 * (ix * vy - iy * vx);

  return {vx + iw * tx + (iy * tz - iz * ty),
          vy + iw * ty + (iz * tx - ix * tz),
          vz + iw * tz + (ix * ty - iy * tx)};
}

// Rotate a vector from body frame into world frame: q_wb * v
std::array<double, 3> MujocoEstimator::rotateBodyToWorld(double qw, double qx,
                                                         double qy, double qz,
                                                         double vx, double vy,
                                                         double vz) {
  double tx = 2.0 * (qy * vz - qz * vy);
  double ty = 2.0 * (qz * vx - qx * vz);
  double tz = 2.0 * (qx * vy - qy * vx);

  return {vx + qw * tx + (qy * tz - qz * ty),
          vy + qw * ty + (qz * tx - qx * tz),
          vz + qw * tz + (qx * ty - qy * tx)};
}

void MujocoEstimator::publishState() {
  // Gate on having all data — mirrors Gazebo plugin's urdf_received_ guard
  if (!latest_odom_ || !latest_joints_ || !urdf_received_) return;

  const auto& odom = *latest_odom_;

  // ------------------------------------------------------------------ //
  //  Body pose and velocities                                           //
  // ------------------------------------------------------------------ //
  double px = odom.pose.pose.position.x;
  double py = odom.pose.pose.position.y;
  double pz = odom.pose.pose.position.z;

  double qw = odom.pose.pose.orientation.w;
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;

  // MuJoCo free-joint qvel[0:3]: world-frame linear velocity
  double vx_w = odom.twist.twist.linear.x;
  double vy_w = odom.twist.twist.linear.y;
  double vz_w = odom.twist.twist.linear.z;

  // MuJoCo free-joint qvel[3:6]: body-frame angular velocity
  // (MuJoCo docs: rotational velocities of a free joint are in the local frame)
  // Mirrors: gz angular vel rotated to body frame via q_bw * w_w
  double wx_b = odom.twist.twist.angular.x;
  double wy_b = odom.twist.twist.angular.y;
  double wz_b = odom.twist.twist.angular.z;

  // Body-frame linear velocity — mirrors v_b = q_bw * v_w in Gazebo plugin
  auto [vx_b, vy_b, vz_b] = rotateWorldToBody(qw, qx, qy, qz, vx_w, vy_w, vz_w);

  // ------------------------------------------------------------------ //
  //  World-frame state (mirrors ground_truth_state_pub_)                //
  // ------------------------------------------------------------------ //
  quad_msgs::msg::RobotState state;
  state.header.stamp = this->get_clock()->now();

  state.body.pose = odom.pose.pose;
  state.body.twist.linear.x = vx_w;
  state.body.twist.linear.y = vy_w;
  state.body.twist.linear.z = vz_w;
  // Angular velocity stored in body frame — matches Gazebo plugin convention
  state.body.twist.angular.x = wx_b;
  state.body.twist.angular.y = wy_b;
  state.body.twist.angular.z = wz_b;

  // ------------------------------------------------------------------ //
  //  Joints — remap from MuJoCo joint_states to quad-sdk order         //
  // ------------------------------------------------------------------ //
  const auto& joints = *latest_joints_;

  std::unordered_map<std::string, size_t> joint_lookup;
  for (size_t i = 0; i < joints.name.size(); ++i) {
    joint_lookup[joints.name[i]] = i;
  }

  state.joints.name = quadsdk_joint_order_;

  for (const auto& jname : quadsdk_joint_order_) {
    if (joint_lookup.count(jname)) {
      size_t idx = joint_lookup.at(jname);
      state.joints.position.push_back(
          idx < joints.position.size() ? joints.position[idx] : 0.0);
      state.joints.velocity.push_back(
          idx < joints.velocity.size() ? joints.velocity[idx] : 0.0);
      double eff = (idx < joints.effort.size()) ? joints.effort[idx] : 0.0;
      if (std::isnan(eff)) eff = 0.0;
      state.joints.effort.push_back(eff);
    } else {
      state.joints.position.push_back(0.0);
      state.joints.velocity.push_back(0.0);
      state.joints.effort.push_back(0.0);
    }
  }

  // ------------------------------------------------------------------ //
  //  Feet — FK via QuadKD2 (mirrors Gazebo plugin updateDynamics +     //
  //  fkRobotState, then toe positions overridden with simulation data)  //
  // ------------------------------------------------------------------ //
  state.feet.feet.resize(4);
  quad_utils::updateDynamics(*quadKD_, state);
  quad_utils::fkRobotState(*quadKD_, state);

  state_pub_->publish(state);

  // ------------------------------------------------------------------ //
  //  Body-frame state (mirrors ground_truth_state_body_frame_pub_)      //
  // ------------------------------------------------------------------ //
  quad_msgs::msg::RobotState state_bf = state;

  // Identity pose — mirrors Gazebo plugin zeroing position and orientation
  state_bf.body.pose.position.x = 0.0;
  state_bf.body.pose.position.y = 0.0;
  state_bf.body.pose.position.z = 0.0;
  state_bf.body.pose.orientation.w = 1.0;
  state_bf.body.pose.orientation.x = 0.0;
  state_bf.body.pose.orientation.y = 0.0;
  state_bf.body.pose.orientation.z = 0.0;

  // Both velocities in body frame
  state_bf.body.twist.linear.x = vx_b;
  state_bf.body.twist.linear.y = vy_b;
  state_bf.body.twist.linear.z = vz_b;
  state_bf.body.twist.angular.x = wx_b;
  state_bf.body.twist.angular.y = wy_b;
  state_bf.body.twist.angular.z = wz_b;

  // Transform world-frame feet (from FK) to body frame.
  auto [wx_w, wy_w, wz_w] = rotateBodyToWorld(qw, qx, qy, qz, wx_b, wy_b, wz_b);

  for (int i = 0; i < 4; ++i) {
    double fPx = state.feet.feet[i].position.x;
    double fPy = state.feet.feet[i].position.y;
    double fPz = state.feet.feet[i].position.z;

    // r: foot position relative to body origin, in world frame
    double rx = fPx - px;
    double ry = fPy - py;
    double rz = fPz - pz;

    // Foot position in body frame
    auto [pfx, pfy, pfz] = rotateWorldToBody(qw, qx, qy, qz, rx, ry, rz);
    state_bf.feet.feet[i].position.x = pfx;
    state_bf.feet.feet[i].position.y = pfy;
    state_bf.feet.feet[i].position.z = pfz;

    // World-frame foot velocity from FK Jacobian
    double fVx = state.feet.feet[i].velocity.x;
    double fVy = state.feet.feet[i].velocity.y;
    double fVz = state.feet.feet[i].velocity.z;

    // omega_w x r_w
    double cx = wy_w * rz - wz_w * ry;
    double cy = wz_w * rx - wx_w * rz;
    double cz = wx_w * ry - wy_w * rx;

    // Relative velocity in world frame
    double vrel_x = (fVx - vx_w) - cx;
    double vrel_y = (fVy - vy_w) - cy;
    double vrel_z = (fVz - vz_w) - cz;

    auto [vfx, vfy, vfz] =
        rotateWorldToBody(qw, qx, qy, qz, vrel_x, vrel_y, vrel_z);
    state_bf.feet.feet[i].velocity.x = vfx;
    state_bf.feet.feet[i].velocity.y = vfy;
    state_bf.feet.feet[i].velocity.z = vfz;
  }

  state_body_frame_pub_->publish(state_bf);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MujocoEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
