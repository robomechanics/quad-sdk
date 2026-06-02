#include "quad_utils/quad_kd2.hpp"

#include "quad_utils/ros_utils.hpp"

using namespace quad_utils;

QuadKD2::QuadKD2(rclcpp::Node::SharedPtr node) : node_(node) { initModel(""); }

QuadKD2::QuadKD2(rclcpp::Node::SharedPtr node, std::string ns) : node_(node) {
  initModel("/" + ns + "/");
}

void QuadKD2::initModel(std::string ns) {
  std::string robot_description_string;

  if (!node_->get_parameter("robot_description", robot_description_string)) {
    RCLCPP_FATAL(node_->get_logger(),
                 "Failed to load robot_description. Shutting down.");
    rclcpp::shutdown();
  }

  try {
    pinocchio::urdf::buildModelFromXML(
        robot_description_string, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(node_->get_logger(),
                "Loaded Pinocchio model with %d joints and %d bodies.",
                model_.njoints, model_.nbodies);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node_->get_logger(), "Error loading model.");
    rclcpp::shutdown();
  }

  // Get the Body Frame ID
  std::string body_frame_name;
  loadROSParamDefault(node_, std::string("body.frame"), body_frame_name,
                      std::string("body"));
  if (body_frame_name.empty()) {
    RCLCPP_FATAL(node_->get_logger(),
                 "Parameter 'body.frame' must be set in the robot yaml.");
    rclcpp::shutdown();
    return;
  }
  body_fid_ = model_.getFrameId(body_frame_name);
  nv_ = model_.nv;
  nq_ = model_.nq;

  limbs_.clear();
  limbs_.resize(num_feet_);
  legbase_offsets_.resize(num_feet_);
  l0_vec_.resize(num_feet_);
  legbase_SE3_.resize(num_feet_);
  joint_min_.resize(num_feet_);
  joint_max_.resize(num_feet_);

  for (int i = 0; i < num_feet_; ++i) {
    std::string p = "leg_" + std::to_string(i);
    LimbInfo& limb = limbs_[i];

    // Load joint names
    std::string abad_name, hip_name, knee_name;
    loadROSParamDefault(node_, p + ".joints.abad.name", abad_name,
                        std::string(""));
    loadROSParamDefault(node_, p + ".joints.hip.name", hip_name,
                        std::string(""));
    loadROSParamDefault(node_, p + ".joints.knee.name", knee_name,
                        std::string(""));
    if (abad_name.empty() || hip_name.empty() || knee_name.empty()) {
      RCLCPP_FATAL(node_->get_logger(),
                   "Missing joint name config for %s. Expected "
                   "'%s.joints.(abad|hip|knee).name'",
                   p.c_str(), p.c_str());
      rclcpp::shutdown();
      return;
    }
    limb.joint_names = {abad_name, hip_name, knee_name};

    // Load Bridge Parameters which account for discrepancies between Robot URDF
    // Models (Diff Axis of Rotation, Origin)
    loadROSParamDefault(node_, p + ".joints.abad.sign", limb.abad_conv.sign,
                        1.0);
    loadROSParamDefault(node_, p + ".joints.abad.offset",
                        limb.abad_conv.origin_offset, 0.0);

    loadROSParamDefault(node_, p + ".joints.hip.sign", limb.hip_conv.sign, 1.0);
    loadROSParamDefault(node_, p + ".joints.hip.offset",
                        limb.hip_conv.origin_offset, 0.0);

    loadROSParamDefault(node_, p + ".joints.knee.sign", limb.knee_conv.sign,
                        1.0);
    loadROSParamDefault(node_, p + ".joints.knee.offset",
                        limb.knee_conv.origin_offset, 0.0);

    std::string hip_frame_name, upper_frame_name, lower_frame_name,
        toe_frame_name;
    loadROSParamDefault(node_, p + ".frames.hip", hip_frame_name,
                        std::string(""));
    loadROSParamDefault(node_, p + ".frames.upper", upper_frame_name,
                        std::string(""));
    loadROSParamDefault(node_, p + ".frames.lower", lower_frame_name,
                        std::string(""));
    loadROSParamDefault(node_, p + ".frames.toe", toe_frame_name,
                        std::string(""));
    if (hip_frame_name.empty() || upper_frame_name.empty() ||
        lower_frame_name.empty() || toe_frame_name.empty()) {
      RCLCPP_FATAL(node_->get_logger(),
                   "Missing frame name config for %s. Expected "
                   "'%s.frames.(hip|upper|lower|toe)'",
                   p.c_str(), p.c_str());
      rclcpp::shutdown();
      return;
    }

    // Pinocchio Internal ID's, Used for Accessing internal Pinocchio data_
    limb.toe_fid = model_.getFrameId(toe_frame_name);
    limb.lower_fid = model_.getFrameId(lower_frame_name);
    limb.upper_fid = model_.getFrameId(upper_frame_name);
    limb.hip_fid = model_.getFrameId(hip_frame_name);

    limb.abad_jid = model_.getJointId(limb.joint_names[0]);
    limb.hip_jid = model_.getJointId(limb.joint_names[1]);
    limb.knee_jid = model_.getJointId(limb.joint_names[2]);
    limb.toe_jid = limb.toe_fid;

    // Set Indicies for q and v vector creation (Pinocchio Internal Mapping)
    // i.e. Joint Order that Pinocchio Expects When Performing Updates
    limb.abad_pin_pos_idx = model_.joints[limb.abad_jid].idx_q();
    limb.hip_pin_pos_idx = model_.joints[limb.hip_jid].idx_q();
    limb.knee_pin_pos_idx = model_.joints[limb.knee_jid].idx_q();

    limb.abad_pin_vel_idx = model_.joints[limb.abad_jid].idx_v();
    limb.hip_pin_vel_idx = model_.joints[limb.hip_jid].idx_v();
    limb.knee_pin_vel_idx = model_.joints[limb.knee_jid].idx_v();

    // Extract Robot Specific Geometries, Offsets
    // Leg Lengths, Legbase, Knee, and Foot Offsets

    // Abad to Hip Offset
    legbase_offsets_[i] = model_.jointPlacements[limb.abad_jid].translation();
    legbase_SE3_[i] = model_.jointPlacements[limb.abad_jid];

    // Y offset between abad an hip rotational planes
    l0_vec_[i] = model_.jointPlacements[limb.hip_jid].translation()(1);

    // Extract Length of Thigh
    knee_offset_ = model_.jointPlacements[limb.knee_jid].translation();
    l1_ = knee_offset_.cwiseAbs().maxCoeff();

    // Extract Length of Calf
    foot_offset_ = model_.frames[limb.toe_jid].placement.translation();
    l2_ = foot_offset_.cwiseAbs().maxCoeff();

    // Extract Joint Limits for Each Leg
    joint_min_[i] = {model_.lowerPositionLimit[limb.abad_pin_pos_idx],
                     model_.lowerPositionLimit[limb.hip_pin_pos_idx],
                     model_.lowerPositionLimit[limb.knee_pin_pos_idx]};

    joint_max_[i] = {model_.upperPositionLimit[limb.abad_pin_pos_idx],
                     model_.upperPositionLimit[limb.hip_pin_pos_idx],
                     model_.upperPositionLimit[limb.knee_pin_pos_idx]};
  }

  std::vector<double> armature;
  loadROSParamDefault(node_, std::string("motor_limits.armature"), armature,
                      std::vector<double>(3, 0.0));
  for (int i = 0; i < num_feet_; ++i) {
    model_.armature[limbs_[i].abad_pin_vel_idx] = armature[0];
    model_.armature[limbs_[i].hip_pin_vel_idx] = armature[1];
    model_.armature[limbs_[i].knee_pin_vel_idx] = armature[2];
  }

  g_body_legbases_.resize(4);
  for (int leg_index = 0; leg_index < 4; leg_index++) {
    pinocchio::JointIndex j_abad = limbs_[leg_index].abad_jid;
    g_body_legbases_[leg_index] =
        convertSE3ToAffine(model_.jointPlacements[j_abad]);
  }
}

Eigen::Matrix4d QuadKD2::createAffineMatrix(Eigen::Vector3d trans,
                                            Eigen::Vector3d rpy) const {
  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

  return t.matrix();
}

Eigen::Matrix4d QuadKD2::createAffineMatrix(Eigen::Vector3d trans,
                                            Eigen::AngleAxisd rot) const {
  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

pinocchio::SE3 QuadKD2::convertAffineToSE3(Eigen::Matrix4d g_transform) const {
  Eigen::Matrix3d rot = g_transform.topLeftCorner<3, 3>();
  Eigen::Vector3d trans = g_transform.topRightCorner<3, 1>();
  pinocchio::SE3 se3_transform(rot, trans);

  return se3_transform;
}

Eigen::Matrix4d QuadKD2::convertSE3ToAffine(
    pinocchio::SE3 se3_transform) const {
  Eigen::Matrix4d g_transform = se3_transform.toHomogeneousMatrix();

  return g_transform;
}

double QuadKD2::getJointLowerLimit(int leg_index, int joint_index) const {
  return joint_min_[leg_index][joint_index];
}

double QuadKD2::getJointUpperLimit(int leg_index, int joint_index) const {
  return joint_max_[leg_index][joint_index];
}

double QuadKD2::getLinkLength(int leg_index, int link_index) const {
  switch (link_index) {
    case 0:
      return l0_vec_[leg_index];
    case 1:
      return l1_;
    case 2:
      return l2_;
    default:
      throw std::runtime_error("Invalid link index");
  }
}

std::vector<std::string> QuadKD2::getOrderedJointNames() const {
  std::vector<std::string> joint_names;
  joint_names.reserve(num_feet_ * 3);
  for (int leg_index = 0; leg_index < num_feet_; ++leg_index) {
    const auto& limb = limbs_.at(leg_index);
    joint_names.insert(joint_names.end(), limb.joint_names.begin(),
                       limb.joint_names.end());
  }
  return joint_names;
}

void QuadKD2::assembleQFromBodyAndJoints(const Eigen::VectorXd& body_state,
                                         const Eigen::VectorXd& joint_positions,
                                         Eigen::VectorXd& q) const {
  q.resize(nq_);

  // Base position
  q.segment<3>(0) = body_state.segment<3>(0);

  // Base orientation (RPY → quaternion)
  const double roll = body_state(3);
  const double pitch = body_state(4);
  const double yaw = body_state(5);

  Eigen::Quaterniond quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  // Pinocchio expects [qx, qy, qz, qw]
  q.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();

  // Joint positions
  for (int leg = 0; leg < num_feet_; ++leg) {
    const auto& L = limbs_[leg];
    const int u = 3 * leg;

    q[L.abad_pin_pos_idx] = joint_positions[u + 0];
    q[L.hip_pin_pos_idx] = joint_positions[u + 1];
    q[L.knee_pin_pos_idx] = joint_positions[u + 2];
  }
}

void QuadKD2::assembleQVFromBodyAndJoints(
    const Eigen::VectorXd& body_state, const Eigen::VectorXd& joint_positions,
    const Eigen::VectorXd& joint_velocities, Eigen::VectorXd& q,
    Eigen::VectorXd& v) const {
  q.resize(nq_);
  v.resize(nv_);

  // Base position
  q.segment<3>(0) = body_state.segment<3>(0);

  // Base orientation (RPY → quaternion)
  double roll = body_state(3);
  double pitch = body_state(4);
  double yaw = body_state(5);

  Eigen::Quaterniond quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  q.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();

  Eigen::Matrix3d R_BW = quat.toRotationMatrix().transpose();

  // Base linear velocity (body frame)
  v.segment<3>(0) = R_BW * body_state.segment<3>(6);

  // Base angular velocity (body frame)
  v.segment<3>(3) = body_state.segment<3>(9);

  // Joint positions and velocities
  for (int leg = 0; leg < num_feet_; ++leg) {
    const auto& L = limbs_[leg];
    const int u = 3 * leg;

    q[L.abad_pin_pos_idx] = joint_positions[u + 0];
    q[L.hip_pin_pos_idx] = joint_positions[u + 1];
    q[L.knee_pin_pos_idx] = joint_positions[u + 2];

    v[L.abad_pin_vel_idx] = joint_velocities[u + 0];
    v[L.hip_pin_vel_idx] = joint_velocities[u + 1];
    v[L.knee_pin_vel_idx] = joint_velocities[u + 2];
  }
}

void QuadKD2::updateFromBodyJoints(const Eigen::VectorXd& body_state,
                                   const Eigen::VectorXd& joint_positions,
                                   const Eigen::VectorXd& joint_velocities) {
  Eigen::VectorXd q, v;
  assembleQVFromBodyAndJoints(body_state, joint_positions, joint_velocities, q,
                              v);
  updateFromPinocchio(q, v);
}

void QuadKD2::updateFromBodyJoints(const Eigen::VectorXd& body_state,
                                   const Eigen::VectorXd& joint_positions) {
  Eigen::VectorXd q;
  assembleQFromBodyAndJoints(body_state, joint_positions, q);
  updateFromPinocchio(q);
}

void QuadKD2::updateFromPinocchio(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& v) {
  Eigen::VectorXd a = Eigen::VectorXd::Zero(nv_);

  // Compute only what we need instead of computeAllTerms (which also
  // computes CoM, kinetic/potential energy, etc.)
  pinocchio::crba(model_, data_, q);
  // crba only fills the upper triangle of M; mirror it to get full symmetric M
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model_, data_, q, v);

  // 3-arg FK gives us accelerations needed by computeInverseDynamics
  pinocchio::forwardKinematics(model_, data_, q, v, a);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  updated_ = true;
}

void QuadKD2::updateFromPinocchio(const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  updated_ = true;
}

void QuadKD2::transformBodyToWorld(Eigen::Vector3d body_pos,
                                   Eigen::Vector3d body_rpy,
                                   Eigen::Matrix4d transform_body,
                                   Eigen::Matrix4d& transform_world) const {
  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Get the desired transform in the world frame
  transform_world = g_world_body * transform_body;
}

void QuadKD2::transformWorldToBody(Eigen::Vector3d body_pos,
                                   Eigen::Vector3d body_rpy,
                                   Eigen::Matrix4d transform_world,
                                   Eigen::Matrix4d& transform_body) const {
  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute the desired transform in the body frame
  transform_body = g_world_body.inverse() * transform_world;
}

void QuadKD2::worldToLegbaseFKWorldFrame(
    int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
    Eigen::Matrix4d& g_world_legbase) const {
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute transform for leg base relative to the world frame
  g_world_legbase = g_world_body * g_body_legbases_[leg_index];
}

void QuadKD2::worldToLegbaseFKWorldFrame(
    int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
    Eigen::Vector3d& leg_base_pos_world) const {
  Eigen::Matrix4d g_world_legbase;
  worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

  leg_base_pos_world = g_world_legbase.block<3, 1>(0, 3);
}

void QuadKD2::worldToNominalHipFKWorldFrame(
    int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
    Eigen::Vector3d& nominal_hip_pos_world) const {
  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);
  // Compute transform from body to legbase but offset by l0
  Eigen::Matrix4d g_body_nominal_hip = g_body_legbases_[leg_index];
  g_body_nominal_hip(1, 3) += 1.0 * l0_vec_[leg_index];

  // Compute transform for offset leg base relative to the world frame
  Eigen::Matrix4d g_world_nominal_hip = g_world_body * g_body_nominal_hip;

  nominal_hip_pos_world = g_world_nominal_hip.block<3, 1>(0, 3);
}

void QuadKD2::getRotationMatrix(const Eigen::VectorXd& rpy,
                                Eigen::Matrix3d& rot) const {
  rot = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
}

void QuadKD2::bodyToFootFKBodyFrame(int leg_index,
                                    Eigen::Matrix4d& g_body_foot) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
    throw std::runtime_error("Leg Index is outside of valid range");
  }

  // Assume that a Pinocchio update has been called
  assert(updated_);

  /// World To Body Frame Transform
  const pinocchio::SE3& g_world_body_se3 = data_.oMf[body_fid_];

  /// World to Toe Frame Transform
  const LimbInfo& limb = limbs_.at(leg_index);
  const pinocchio::SE3& g_world_foot_se3 = data_.oMf[limb.toe_fid];

  // Convert To Body Frame Transformation
  pinocchio::SE3 g_body_foot_se3 =
      g_world_body_se3.inverse() * g_world_foot_se3;

  // Convert to Eigen Homogeneous Matrix
  g_body_foot = g_body_foot_se3.toHomogeneousMatrix();
}

void QuadKD2::bodyToFootFKBodyFrame(int leg_index,
                                    Eigen::Vector3d& foot_pos_body) const {
  Eigen::Matrix4d g_body_foot;
  QuadKD2::bodyToFootFKBodyFrame(leg_index, g_body_foot);

  // Extract cartesian position of foot in the body frame
  foot_pos_body = g_body_foot.block<3, 1>(0, 3);
}

void QuadKD2::worldToFootFKWorldFrame(int leg_index,
                                      Eigen::Matrix4d& g_world_foot) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
    throw std::runtime_error("Leg index is outside of valid range");
  }

  // Assume that a Pinocchio update has been called
  assert(updated_);

  /// World to Toe Frame Transform
  const LimbInfo& limb = limbs_.at(leg_index);
  const pinocchio::SE3& g_world_foot_se3 = data_.oMf[limb.toe_fid];

  // Convert to Eigen Homogenous Matrix
  g_world_foot = g_world_foot_se3.toHomogeneousMatrix();
}

void QuadKD2::worldToFootFKWorldFrame(int leg_index,
                                      Eigen::Vector3d& foot_pos_world) const {
  Eigen::Matrix4d g_world_foot;
  QuadKD2::worldToFootFKWorldFrame(leg_index, g_world_foot);

  // Extract cartesian position of the foot in the world frame
  foot_pos_world = g_world_foot.block<3, 1>(0, 3);
}

void QuadKD2::worldToKneeFKWorldFrame(int leg_index,
                                      Eigen::Matrix4d& g_world_knee) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
    throw std::runtime_error("Leg index is outside of valid range");
  }
  // Assume that a Pinocchio update has been called
  assert(updated_);

  // World To Knee Frame Transform
  const LimbInfo& limb = limbs_.at(leg_index);
  const pinocchio::SE3& g_world_knee_se3 = data_.oMf[limb.lower_fid];

  // Convert to Eigen Homogenous Matrix
  g_world_knee = g_world_knee_se3.toHomogeneousMatrix();
}

void QuadKD2::worldToKneeFKWorldFrame(int leg_index,
                                      Eigen::Vector3d& knee_pos_world) const {
  Eigen::Matrix4d g_world_knee;
  QuadKD2::worldToKneeFKWorldFrame(leg_index, g_world_knee);

  // Extract cartesian position of the foot in the world frame
  knee_pos_world = g_world_knee.block<3, 1>(0, 3);
}

bool QuadKD2::worldToFootIKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                                      Eigen::Vector3d body_rpy,
                                      Eigen::Vector3d foot_pos_world,
                                      Eigen::Vector3d& joint_state) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Calculate offsets
  Eigen::Vector3d legbase_offset = legbase_offsets_[leg_index];
  double l0 = l0_vec_[leg_index];

  // Initialize transforms
  Eigen::Matrix4d g_world_legbase;
  Eigen::Matrix4d g_world_foot;
  Eigen::Matrix4d g_legbase_foot;
  Eigen::Vector3d foot_pos_legbase;

  // Compute transforms
  worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

  g_world_foot = createAffineMatrix(
      foot_pos_world, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

  // Compute foot position relative to the leg base in cartesian coordinates
  g_legbase_foot = g_world_legbase.inverse() * g_world_foot;
  foot_pos_legbase = g_legbase_foot.block<3, 1>(0, 3);

  return legbaseToFootIKLegbaseFrame(leg_index, foot_pos_legbase, joint_state);
}

bool QuadKD2::legbaseToFootIKLegbaseFrame(int leg_index,
                                          Eigen::Vector3d foot_pos_legbase,
                                          Eigen::Vector3d& joint_state) const {
  // Initialize exact bool
  bool is_exact = true;

  // Calculate offsets
  Eigen::Vector3d legbase_offset = legbase_offsets_[leg_index];
  double l0 = l0_vec_[leg_index];

  // Extract coordinates and declare joint variables
  double x = foot_pos_legbase[0];
  double y = foot_pos_legbase[1];
  double z = foot_pos_legbase[2];
  double q0, q0_pin;
  double q1, q1_pin;
  double q2, q2_pin;

  // Start IK, check foot pos is at least l0 away from leg base, clamp otherwise
  double temp = l0 / sqrt(z * z + y * y);
  if (abs(temp) > 1) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Foot too close, choosing closest alternative\n");
    is_exact = false;
    temp = std::max(std::min(temp, 1.0), -1.0);
  }

  // Compute both solutions of q0, use hip-above-knee if z<0 (preferred)
  // Store the inverted solution in case hip limits are exceeded
  if (z > 0) {
    q0 = -acos(temp) + atan2(z, y);
  } else {
    q0 = acos(temp) + atan2(z, y);
  }

  q0_pin = (q0 * limbs_[leg_index].abad_conv.sign) +
           limbs_[leg_index].abad_conv.origin_offset;

  // Make sure abad is within joint limits, clamp otherwise
  if (q0_pin > joint_max_[leg_index][0] || q0_pin < joint_min_[leg_index][0]) {
    q0_pin = std::max(std::min(q0_pin, joint_max_[leg_index][0]),
                      joint_min_[leg_index][0]);
    q0 = (q0_pin - limbs_[leg_index].abad_conv.origin_offset) /
         limbs_[leg_index].abad_conv.sign;
    is_exact = false;
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Abad limits exceeded, clamping to %5.3f \n", q0);
  }

  // Rotate to ab-ad fixed frame
  double z_body_frame = z;
  z = -sin(q0) * y + cos(q0) * z_body_frame;

  // Check reachibility for hip
  double acos_eps = 1.0;
  double temp2 =
      (l1_ * l1_ + x * x + z * z - l2_ * l2_) / (2 * l1_ * sqrt(x * x + z * z));
  if (abs(temp2) > acos_eps) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Foot location too far for hip, choosing closest"
                          " alternative \n");
    is_exact = false;
    temp2 = std::max(std::min(temp2, acos_eps), -acos_eps);
  }

  // Check reachibility for knee
  double temp3 = (l1_ * l1_ + l2_ * l2_ - x * x - z * z) / (2 * l1_ * l2_);

  if (temp3 > acos_eps || temp3 < -acos_eps) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Foot location too far for knee, choosing closest"
                          " alternative \n");
    is_exact = false;

    temp3 = std::max(std::min(temp3, acos_eps), -acos_eps);
  }

  // Compute joint angles
  q1 = 0.5 * M_PI + atan2(x, -z) - acos(temp2);
  // q1 = atan2(x, -z) + acos(temp2);
  q1_pin = (q1 * limbs_[leg_index].hip_conv.sign) +
           limbs_[leg_index].hip_conv.origin_offset;

  // Make sure hip is within joint limits
  if (q1_pin > joint_max_[leg_index][1] || q1_pin < joint_min_[leg_index][1]) {
    q1_pin = std::max(std::min(q1_pin, joint_max_[leg_index][1]),
                      joint_min_[leg_index][1]);
    q1 = (q1_pin - limbs_[leg_index].hip_conv.origin_offset) /
         limbs_[leg_index].hip_conv.sign;
    is_exact = false;
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Hip limits exceeded, clamping to %5.3f \n", q1);
  }

  // Compute knee val to get closest toe position in the plane
  Eigen::Vector2d knee_pos, toe_pos, toe_offset;
  knee_pos << -l1_ * cos(q1), -l1_ * sin(q1);
  toe_pos << x, z;
  toe_offset = toe_pos - knee_pos;
  q2 = atan2(-toe_offset(1), toe_offset(0)) + q1;
  q2_pin = (q2 * limbs_[leg_index].knee_conv.sign) +
           limbs_[leg_index].knee_conv.origin_offset;

  // Make sure knee is within joint limits
  if (q2_pin > joint_max_[leg_index][2] || q2_pin < joint_min_[leg_index][2]) {
    q2_pin = std::max(std::min(q2_pin, joint_max_[leg_index][2]),
                      joint_min_[leg_index][2]);
    q2 = (q2_pin - limbs_[leg_index].knee_conv.origin_offset) /
         limbs_[leg_index].knee_conv.sign;
    is_exact = false;
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Knee limits exceeded, clamping to %5.3f \n", q2);
  }

  // q1 is undefined if q2=0, resolve this
  if (q2 == 0) {
    q1 = 0;
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "Hip value undefined (in singularity), setting to"
                          " %5.3f \n",
                          q1);
    is_exact = false;
  }

  if (z_body_frame - l0 * sin(q0) > 0) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                          "IK solution is in hip-inverted region! Beware!\n");
    is_exact = false;
  }

  joint_state = {q0_pin, q1_pin, q2_pin};
  return is_exact;
}

void QuadKD2::getJacobianGenCoord(Eigen::MatrixXd& jacobian) const {
  this->getJacobianBodyAngVel(jacobian);

  const pinocchio::SE3& g_world_body_se3 = data_.oMf[body_fid_];
  Eigen::Matrix3d R_WB = g_world_body_se3.rotation();
  Eigen::Vector3d ypr = R_WB.eulerAngles(2, 1, 0);
  double yaw = ypr[0];
  double pitch = ypr[1];
  double roll = ypr[2];

  // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame,
  // which is multiplied by Jacobian to map it to Euler angle change rate here
  for (int i = 0; i < 4; i++) {
    Eigen::MatrixXd transform_jac(3, 3);
    transform_jac << 1, 0, -sin(pitch), 0, cos(roll), cos(pitch) * sin(roll), 0,
        -sin(roll), cos(pitch) * cos(roll);

    jacobian.block(3 * i, 15, 3, 3) =
        jacobian.block(3 * i, 15, 3, 3) * transform_jac;
  }
}

void QuadKD2::getJacobianWorldAngVel(Eigen::MatrixXd& jacobian) const {
  this->getJacobianBodyAngVel(jacobian);

  const pinocchio::SE3& g_world_body_se3 = data_.oMf[body_fid_];
  Eigen::Matrix3d R_BW = g_world_body_se3.rotation().transpose();

  for (int i = 0; i < num_feet_; ++i) {
    jacobian.block(3 * i, nv_ - 3, 3, 3) =
        jacobian.block(3 * i, nv_ - 3, 3, 3) * R_BW;
  }
}

void QuadKD2::getJacobianBodyAngVel(Eigen::MatrixXd& jacobian) const {
  assert(jacobian.rows() == 3 * num_feet_ && jacobian.cols() == nv_);

  // Assume that a Pinocchio update has been called
  assert(updated_);

  const pinocchio::SE3& g_world_body_se3 = data_.oMf[body_fid_];
  const Eigen::Matrix3d R_WB = g_world_body_se3.rotation();
  const Eigen::Matrix3d R_BW = R_WB.transpose();

  jacobian.setZero();
  Eigen::MatrixXd jac_block(6, nv_);
  for (int i = 0; i < num_feet_; i++) {
    jac_block.setZero();
    pinocchio::getFrameJacobian(model_, data_, limbs_[i].toe_fid,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                jac_block);

    // Pinocchio Convention (free flyer)
    // q = [x_world y_world z_world congugate_quaternion_body_to_world theta]
    // Returns a J(q) that enables this mapping:
    // [v_foot_world; w_foot_world] = J_world(q) q_dot
    // where q_dot = [v_base_body w_base_body theta_dot]

    // Reordering Columns and Rows to Reflect Quad-SDK Convention
    const auto jac_block_lin = jac_block.block(0, 0, 3, nv_);
    const int u = 3 * i;
    jacobian.block(3 * i, u + 0, 3, 1) =
        jac_block_lin.col(limbs_[i].abad_pin_vel_idx);
    jacobian.block(3 * i, u + 1, 3, 1) =
        jac_block_lin.col(limbs_[i].hip_pin_vel_idx);
    jacobian.block(3 * i, u + 2, 3, 1) =
        jac_block_lin.col(limbs_[i].knee_pin_vel_idx);

    jacobian.block(3 * i, nv_ - 6, 3, 3) =
        jac_block_lin.block(0, 0, 3, 3) * R_BW;
    jacobian.block(3 * i, nv_ - 3, 3, 3) = jac_block_lin.block(0, 3, 3, 3);
  }
  // Function Internal Convention (Refer to this When Using)
  // v = [v_foot0_world, v_foot1_world, v_foot2_world, v_foot3_world]
  // q_dot_func = [theta_dot v_base_world w_base_body]
  // v = jacobian * q_dot_func
}

void QuadKD2::computeInverseDynamics(const Eigen::VectorXd& foot_acc,
                                     const Eigen::VectorXd& grf,
                                     const std::vector<int>& contact_mode,
                                     Eigen::VectorXd& tau) const {
  // Assume that a Pinocchio update has been called
  assert(updated_);

  // Build raw Pinocchio-ordered Jacobian J_pin
  Eigen::MatrixXd J_pin = Eigen::MatrixXd::Zero(12, nv_);
  for (int i = 0; i < num_feet_; ++i) {
    Eigen::MatrixXd jac_block(6, nv_);
    jac_block.setZero();
    pinocchio::getFrameJacobian(model_, data_, limbs_[i].toe_fid,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                jac_block);
    J_pin.block(3 * i, 0, 3, nv_) = jac_block.block(0, 0, 3, nv_);
  }

  // Build Pinocchio-to-quad-sdk joint reordering index
  // jidx(k) = Pinocchio velocity index for quad-sdk joint k
  Eigen::Matrix<int, 12, 1> jidx;
  for (int leg = 0; leg < num_feet_; ++leg) {
    jidx(3 * leg + 0) = limbs_[leg].abad_pin_vel_idx;
    jidx(3 * leg + 1) = limbs_[leg].hip_pin_vel_idx;
    jidx(3 * leg + 2) = limbs_[leg].knee_pin_vel_idx;
  }

  // Reorder Jacobian columns: [base(6) | joints in quad-sdk order(12)]
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, nv_);
  jacobian.block(0, 0, 12, 6) = J_pin.block(0, 0, 12, 6);
  for (int k = 0; k < 12; ++k) {
    jacobian.col(6 + k) = J_pin.col(jidx(k));
  }

  // Compute the equivalent force in generalized coordinates
  Eigen::VectorXd tau_stance = -jacobian.transpose() * grf;

  // Reorder M and N from Pinocchio order to quad-sdk order so all
  // block operations below use a consistent joint ordering.
  const Eigen::MatrixXd& M_pin = data_.M;
  const Eigen::VectorXd& N_pin = data_.nle;

  Eigen::MatrixXd M(nv_, nv_);
  Eigen::VectorXd N(nv_);

  // Floating-base block (rows/cols 0..5) — no reorder
  M.block(0, 0, 6, 6) = M_pin.block(0, 0, 6, 6);
  N.segment(0, 6) = N_pin.segment(0, 6);

  // Cross blocks (base ↔ joints) and joint-joint block
  for (int k = 0; k < 12; ++k) {
    M.block(0, 6 + k, 6, 1) = M_pin.block(0, jidx(k), 6, 1);
    M.block(6 + k, 0, 1, 6) = M_pin.block(jidx(k), 0, 1, 6);
    N(6 + k) = N_pin(jidx(k));
    for (int l = 0; l < 12; ++l) {
      M(6 + k, 6 + l) = M_pin(jidx(k), jidx(l));
    }
  }

  // Compute J_dot*q_dot
  Eigen::VectorXd foot_acc_J_dot(12);
  for (int i = 0; i < 4; i++) {
    const pinocchio::Motion& a_world = pinocchio::getFrameAcceleration(
        model_, data_, limbs_[i].toe_fid,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    foot_acc_J_dot.segment(3 * i, 3) = a_world.linear();
  }

  // Compute constraint Jacobian A and A_dot*q_dot
  int constraints_num =
      3 * std::count(contact_mode.begin(), contact_mode.end(), true);
  Eigen::MatrixXd A(constraints_num, nv_);
  Eigen::VectorXd A_dotq_dot(constraints_num);
  int constraints_count = 0;
  for (int i = 0; i < num_feet_; i++) {
    if (contact_mode.at(i)) {
      A.block(3 * constraints_count, 0, 3, nv_) =
          jacobian.block(3 * i, 0, 3, nv_);
      A_dotq_dot.segment(3 * constraints_count, 3) =
          foot_acc_J_dot.segment(3 * i, 3);
      constraints_count++;
    }
  }

  // Compute acceleration from J*q_ddot
  Eigen::VectorXd foot_acc_q_ddot = foot_acc - foot_acc_J_dot;

  // Compuate damped jacobian inverser
  Eigen::MatrixXd jacobian_inv =
      math_utils::sdlsInv(jacobian.block(0, 6, 12, 12));

  // In the EOM, we know M, N, tau_grf, and a = J_b*q_ddot_b + J_l*q_ddot_l, we
  // need to solve q_ddot_b and tau_swing
  Eigen::MatrixXd blk_mat =
      Eigen::MatrixXd::Zero(nv_ + constraints_num, nv_ + constraints_num);
  blk_mat.block(0, 0, 6, 6) =
      -M.block(0, 0, 6, 6) +
      M.block(0, 6, 6, 12) * jacobian_inv * jacobian.block(0, 0, 12, 6);
  blk_mat.block(6, 0, 12, 6) =
      -M.block(6, 0, 12, 6) +
      M.block(6, 6, 12, 12) * jacobian_inv * jacobian.block(0, 0, 12, 6);
  for (int i = 0; i < num_feet_; i++) {
    if (!contact_mode.at(i)) {
      blk_mat.block(3 * i + 6, 3 * i + 6, 3, 3).diagonal().fill(1);
    }
  }
  blk_mat.block(0, nv_, nv_, constraints_num) = -A.transpose();
  blk_mat.block(nv_, 0, constraints_num, 6) =
      -A.leftCols(6) +
      A.rightCols(12) * jacobian_inv * jacobian.block(0, 0, 12, 6);

  // Perform inverse dynamics
  Eigen::VectorXd tau_swing(12), blk_sol(nv_ + constraints_num),
      blk_vec(nv_ + constraints_num);
  blk_vec.segment(0, 6) << N.segment(0, 6) + M.block(0, 6, 6, 12) *
                                                 jacobian_inv * foot_acc_q_ddot;
  blk_vec.segment(6, 12) << N.segment(6, 12) +
                                M.block(6, 6, 12, 12) * jacobian_inv *
                                    foot_acc_q_ddot -
                                tau_stance.segment(6, 12);
  blk_vec.segment(nv_, constraints_num)
      << A_dotq_dot + A.leftCols(12) * jacobian_inv * foot_acc_q_ddot;
  blk_sol =
      blk_mat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(blk_vec);
  tau_swing = blk_sol.segment(6, 12);

  // Convert the order back
  for (int i = 0; i < num_feet_; i++) {
    if (contact_mode.at(i)) {
      tau.segment(3 * i, 3) = tau_stance.segment(6 + 3 * i, 3);
    } else {
      tau.segment(3 * i, 3) = tau_swing.segment(3 * i, 3);
    }
  }

  // Check inf or nan
  if (!(tau.array() == tau.array()).all() ||
      !((tau - tau).array() == (tau - tau).array()).all()) {
    tau.setZero();
  }
}

bool QuadKD2::convertCentroidalToFullBody(
    const Eigen::VectorXd& body_state, const Eigen::VectorXd& foot_positions,
    const Eigen::VectorXd& foot_velocities, const Eigen::VectorXd& grfs,
    Eigen::VectorXd& joint_positions, Eigen::VectorXd& joint_velocities,
    Eigen::VectorXd& torques) {
  // Assume the conversion is exact unless a check below fails
  bool is_exact = true;

  // Extract kinematic quantities
  Eigen::Vector3d body_pos = body_state.segment<3>(0);
  Eigen::Vector3d body_rpy = body_state.segment<3>(3);

  auto t_start = std::chrono::steady_clock::now();
  // Perform IK for each leg
  for (int i = 0; i < num_feet_; i++) {
    Eigen::Vector3d leg_joint_state;
    Eigen::Vector3d foot_pos = foot_positions.segment<3>(3 * i);
    is_exact = is_exact && worldToFootIKWorldFrame(i, body_pos, body_rpy,
                                                   foot_pos, leg_joint_state);
    joint_positions.segment<3>(3 * i) = leg_joint_state;
  }

  auto t_ik = std::chrono::steady_clock::now();

  // Load state positions
  Eigen::VectorXd state_positions(18), state_velocities(18);

  // Compute jacobian
  updateFromBodyJoints(body_state, joint_positions);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  getJacobianBodyAngVel(jacobian);

  auto t_jacob = std::chrono::steady_clock::now();

  // Compute joint velocities
  joint_velocities = jacobian.leftCols(12).colPivHouseholderQr().solve(
      foot_velocities - jacobian.rightCols(6) * body_state.tail(6));

  auto t_ik_vel = std::chrono::steady_clock::now();

  torques = -jacobian.leftCols(12).transpose() * grfs;

  auto t_id = std::chrono::steady_clock::now();

  std::chrono::duration<double> t_diff_ik =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_ik - t_start);
  std::chrono::duration<double> t_diff_jacob =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_jacob - t_ik);
  std::chrono::duration<double> t_diff_ik_vel =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_ik_vel -
                                                                t_jacob);
  std::chrono::duration<double> t_diff_id =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_id -
                                                                t_ik_vel);

  return is_exact;
}

bool QuadKD2::applyMotorModel(const Eigen::VectorXd& torques,
                              Eigen::VectorXd& constrained_torques) {
  // Constrain torques to max values
  constrained_torques.resize(torques.size());
  constrained_torques = torques.cwiseMax(-tau_max_).cwiseMin(tau_max_);

  // Check if torques was modified
  return constrained_torques.isApprox(torques);
}

bool QuadKD2::applyMotorModel(const Eigen::VectorXd& joint_torques,
                              const Eigen::VectorXd& joint_velocities,
                              Eigen::VectorXd& constrained_joint_torques) {
  // Constrain torques to max values
  Eigen::VectorXd constraint_violation(joint_torques.size());
  constrained_joint_torques.resize(joint_torques.size());
  constrained_joint_torques =
      joint_torques.cwiseMax(-tau_max_).cwiseMin(tau_max_);

  // Apply linear motor model
  Eigen::VectorXd emf = joint_velocities.cwiseProduct(mm_slope_);
  constrained_joint_torques =
      constrained_joint_torques.cwiseMax(-tau_max_ - emf)
          .cwiseMin(tau_max_ - emf);

  // Check if torques were modified
  return constrained_joint_torques.isApprox(joint_torques);
}

bool QuadKD2::isValidFullState(const Eigen::VectorXd& body_state,
                               const Eigen::VectorXd& joint_positions,
                               const Eigen::VectorXd& joint_velocities,
                               const Eigen::VectorXd& joint_torques,
                               const grid_map::GridMap& terrain,
                               Eigen::VectorXd& state_violation,
                               Eigen::VectorXd& control_violation) {
  // Assume that a Pinocchio update has been called
  assert(updated_);

  // Check state constraints
  // Kinematics
  state_violation.setZero(num_feet_);
  for (int i = 0; i < num_feet_; i++) {
    Eigen::Vector3d knee_pos_world;
    worldToKneeFKWorldFrame(i, knee_pos_world);
    state_violation[i] = getGroundClearance(knee_pos_world, terrain);
  }
  bool state_valid = (state_violation.array() >= 0).all();

  // Check control constraints
  // Motor model
  Eigen::VectorXd constrained_joint_torques(12);
  bool control_valid = applyMotorModel(joint_torques, joint_velocities,
                                       constrained_joint_torques);
  control_violation.setZero(joint_torques.size());
  control_violation = -(constrained_joint_torques - joint_torques).cwiseAbs();

  // Only valid if each subcheck is valid
  return (state_valid && control_valid);
}

bool QuadKD2::isValidCentroidalState(
    const Eigen::VectorXd& body_state, const Eigen::VectorXd& foot_positions,
    const Eigen::VectorXd& foot_velocities, const Eigen::VectorXd& grfs,
    const grid_map::GridMap& terrain, Eigen::VectorXd& joint_positions,
    Eigen::VectorXd& joint_velocities, Eigen::VectorXd& joint_torques,
    Eigen::VectorXd& state_violation, Eigen::VectorXd& control_violation) {
  // Convert to full
  bool is_exact = convertCentroidalToFullBody(
      body_state, foot_positions, foot_velocities, grfs, joint_positions,
      joint_velocities, joint_torques);

  updateFromBodyJoints(body_state, joint_positions, joint_velocities);

  bool is_valid = isValidFullState(body_state, joint_positions,
                                   joint_velocities, joint_torques, terrain,
                                   state_violation, control_violation);

  return (is_exact && is_valid);
}
