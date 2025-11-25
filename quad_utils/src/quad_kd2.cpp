#include "quad_utils/quad_kd2.hpp"

using namespace quad_utils;

// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

QuadKD2::QuadKD2(rclcpp::Node::SharedPtr node) : node_(node) { initModel(""); }

QuadKD2::QuadKD2(rclcpp::Node::SharedPtr node, std::string ns) : node_(node)
{
  initModel("/" + ns + "/");
}

void QuadKD2::initModel(std::string ns)
{
  std::string robot_description_string;

  if (!node_->get_parameter("robot_description", robot_description_string))
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Failed to load robot_description. Shutting down.");
    rclcpp::shutdown();
  }

  try
  {
    pinocchio::urdf::buildModelFromXML(robot_description_string, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(node_->get_logger(),
                "Loaded Pinocchio model with %d joints and %d bodies.",
                model_.njoints, model_.nbodies);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(node_->get_logger(), "Error loading model.");
    rclcpp::shutdown();
  }

  std::vector<std::string> body_name_list = {"body"};
  std::vector<std::string> hip_name_list = {"hip0", "hip1", "hip2", "hip3"};
  std::vector<std::string> upper_name_list = {"upper0", "upper1", "upper2",
                                              "upper3"};
  std::vector<std::string> lower_name_list = {"lower0", "lower1", "lower2",
                                              "lower3"};
  std::vector<std::string> toe_name_list = {"toe0", "toe1", "toe2", "toe3"};
  
  std::vector<std::string> abad_joint_list = {"8", "9", "10", "11"};
  std::vector<std::string> hip_joint_list = {"0", "2", "4", "6"};
  std::vector<std::string> knee_joint_list = {"1", "3", "5", "7"};
  std::vector<std::string> toe_joint_list = {"jtoe0", "jtoe1", "jtoe2", "jtoe3"};

  // Get the Body Frame ID
  body_fid_ = model_.getFrameId(body_name_list[0]);
  

  limbs_.clear();
  limbs_.resize(num_feet_);
  for (size_t i = 0; i < 4; ++i){
    LimbInfo limb;

    limb.toe_fid = model_.getFrameId(toe_name_list[i]);
    limb.lower_fid = model_.getFrameId(lower_name_list[i]);
    limb.upper_fid = model_.getFrameId(upper_name_list[i]);
    limb.hip_fid = model_.getFrameId(hip_name_list[i]);

    limb.abad_jid = model_.getJointId(abad_joint_list[i]);
    limb.hip_jid  = model_.getJointId(hip_joint_list[i]);
    limb.knee_jid = model_.getJointId(knee_joint_list[i]);

    limbs_[i] = limb;
  }

  // Read leg geometry from URDF
  legbase_offsets_.resize(4);
  l0_vec_.resize(4);
  for (size_t i = 0; i < 4; i++)
  {
    pinocchio::JointIndex j_abad = model_.getJointId(abad_joint_list.at(i));
    pinocchio::JointIndex j_hip = model_.getJointId(hip_joint_list.at(i));
    pinocchio::JointIndex j_knee = model_.getJointId(knee_joint_list.at(i));
    pinocchio::JointIndex j_toe = model_.getJointId(toe_joint_list.at(i));

    legbase_offsets_[i] = model_.jointPlacements[j_abad].translation();
    legbase_SE3_[i] = model_.jointPlacements[j_abad];

    l0_vec_[i] = model_.jointPlacements[j_hip].translation()(1);

    knee_offset_ = model_.jointPlacements[j_knee].translation();
    l1_ = knee_offset_.cwiseAbs().maxCoeff();

    foot_offset_ = model_.jointPlacements[j_toe].translation();
    l2_ = foot_offset_.cwiseAbs().maxCoeff();
  }

  // Abad offset from legbase
  abad_offset_ = {0, 0, 0};

  g_body_legbases_.resize(4);
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // Compute transforms
    pinocchio::JointIndex j_abad = model_.getJointId(abad_joint_list.at(leg_index));
    g_body_legbases_[leg_index] =
      convertSE3ToAffine(model_.jointPlacements[j_abad]);
  }

  joint_min_.resize(num_feet_);
  joint_max_.resize(num_feet_);

  // Set Limits for Each of the Joints from the URDF
  for (int leg_index = 0; leg_index < 4; leg_index++){
    // Grab the Upper and Lower Limits for Leg 0(8, 0, 1), Leg 1(9, 2, 3), Leg 2(10, 4, 5), Leg 3(11, 6, 7)
    // USing the Abad, Hip and Knee Joint Lists from the Loaded Model

    // Compile a Per Leg List of Joint Names
    std::vector<std::string> leg_joint_names = {
      abad_joint_list[leg_index], 
      hip_joint_list[leg_index],
      knee_joint_list[leg_index]
    };

    std::vector<double> lower_leg_limits;
    std::vector<double> upper_leg_limits;

    // For each Joint in the Leg, Grab its Upper and Lower Torque Limits
    for (const auto &name : leg_joint_names){
      pinocchio::JointIndex jid = model_.getJointId(name);
      Eigen::Index iq = static_cast<Eigen::Index>(model_.joints[jid].idx_q());

      lower_leg_limits.push_back(model_.lowerPositionLimit[iq]);
      upper_leg_limits.push_back(model_.upperPositionLimit[iq]);

    }

    joint_min_[leg_index] = lower_leg_limits;
    joint_max_[leg_index] = upper_leg_limits;
  }
 
}

Eigen::Matrix4d QuadKD2::createAffineMatrix(Eigen::Vector3d trans,
                                           Eigen::Vector3d rpy) const
{
  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

  return t.matrix();
}

Eigen::Matrix4d QuadKD2::createAffineMatrix(Eigen::Vector3d trans,
                                           Eigen::AngleAxisd rot) const
{
  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

pinocchio::SE3 QuadKD2::convertAffineToSE3(Eigen::Matrix4d g_transform) const
{
  Eigen::Matrix3d rot = g_transform.topLeftCorner<3,3>();
  Eigen::Vector3d trans = g_transform.topRightCorner<3,1>();
  pinocchio::SE3 se3_transform(rot, trans);

  return se3_transform;
}

Eigen::Matrix4d QuadKD2::convertSE3ToAffine(pinocchio::SE3 se3_transform) const
{
  Eigen::Matrix4d g_transform = se3_transform.toHomogeneousMatrix();

  return g_transform;
}

double QuadKD2::getJointLowerLimit(int leg_index, int joint_index) const
{
  return joint_min_[leg_index][joint_index];
}

double QuadKD2::getJointUpperLimit(int leg_index, int joint_index) const
{
  return joint_max_[leg_index][joint_index];
}

double QuadKD2::getLinkLength(int leg_index, int link_index) const
{
  switch (link_index)
  {
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

void QuadKD2::update(const Eigen::VectorXd q, const Eigen::VectorXd v){
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_);
}

void QuadKD2::update(const Eigen::VectorXd q){
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_);
}

void QuadKD2::transformBodyToWorld(Eigen::Vector3d body_pos,
                                  Eigen::Vector3d body_rpy,
                                  Eigen::Matrix4d transform_body,
                                  Eigen::Matrix4d &transform_world) const {
  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Get the desired transform in the world frame
  transform_world = g_world_body * transform_body;
}

void QuadKD2::transformWorldToBody(Eigen::Vector3d body_pos,
                                  Eigen::Vector3d body_rpy,
                                  Eigen::Matrix4d transform_world,
                                  Eigen::Matrix4d &transform_body) const {
  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute the desired transform in the body frame
  transform_body = g_world_body.inverse() * transform_world;
}

void QuadKD2::worldToLegbaseFKWorldFrame(
    int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
    Eigen::Matrix4d &g_world_legbase) const {
  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute transform for leg base relative to the world frame
  g_world_legbase = g_world_body * g_body_legbases_[leg_index];
}

void QuadKD2::worldToLegbaseFKWorldFrame(
    int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
    Eigen::Vector3d &leg_base_pos_world) const {
  Eigen::Matrix4d g_world_legbase;
  worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

  leg_base_pos_world = g_world_legbase.block<3, 1>(0, 3);
}

void QuadKD2::worldToNominalHipFKWorldFrame(
    int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
    Eigen::Vector3d &nominal_hip_pos_world) const {
  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);
  // Compute transform from body to legbase but offset by l0
  Eigen::Matrix4d g_body_nominal_hip = g_body_legbases_[leg_index];
  g_body_nominal_hip(1, 3) += 1.0 * l0_vec_[leg_index];

  // Compute transform for offset leg base relative to the world frame
  Eigen::Matrix4d g_world_nominal_hip = g_world_body * g_body_nominal_hip;

  nominal_hip_pos_world = g_world_nominal_hip.block<3, 1>(0, 3);
}

void QuadKD2::getRotationMatrix(const Eigen::VectorXd &rpy,
                               Eigen::Matrix3d &rot) const {
  rot = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
}

void QuadKD2::bodyToFootFKBodyFrame(int leg_index,
                                  Eigen::Matrix4d &g_body_foot) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
    throw std::runtime_error("Leg Index is outside of valid range");
  }
  
  /// World To Body Frame Transform
  const pinocchio::SE3 &g_world_body_se3 = data_.oMf[body_fid_];

  /// World to Toe Frame Transform
  const LimbInfo &limb = limbs_.at(leg_index);
  const pinocchio::SE3 &g_world_foot_se3 = data_.oMf[limb.toe_fid];

  // Convert To Body Frame Transformation
  pinocchio::SE3 g_body_foot_se3 =
      g_world_body_se3.inverse() * g_world_foot_se3;

  // Convert to Eigen Homogeneous Matrix
  g_body_foot = g_body_foot_se3.toHomogeneousMatrix();
}

void QuadKD2::bodyToFootFKBodyFrame(int leg_index, Eigen::Vector3d &foot_pos_body) const {
  Eigen::Matrix4d g_body_foot;
  QuadKD2::bodyToFootFKBodyFrame(leg_index, g_body_foot);

  // Extract cartesian position of foot in the body frame
  foot_pos_body = g_body_foot.block<3,1>(0,3);
}

void QuadKD2::worldToFootFKWorldFrame(int leg_index, Eigen::Matrix4d &g_world_foot) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
    throw std::runtime_error("Leg index is outside of valid range");
  }

  /// World to Toe Frame Transform
  const LimbInfo &limb = limbs_.at(leg_index);
  const pinocchio::SE3 &g_world_foot_se3 = data_.oMf[limb.toe_fid];

  // Convert to Eigen Homogenous Matrix
  g_world_foot = g_world_foot_se3.toHomogeneousMatrix();

}

void QuadKD2::worldToFootFKWorldFrame(int leg_index, Eigen::Vector3d &foot_pos_world) const {
  Eigen::Matrix4d g_world_foot;
  QuadKD2::worldToFootFKWorldFrame(leg_index, g_world_foot);

  // Extract cartesian position of the foot in the world frame
  foot_pos_world = g_world_foot.block<3,1>(0,3);
}

void QuadKD2::worldToKneeFKWorldFrame(int leg_index, Eigen::Matrix4d &g_world_knee) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0){
    throw std::runtime_error("Leg index is outside of valid range");
  }

  // World To Knee Frame Transform
  const LimbInfo &limb = limbs_.at(leg_index);
  const pinocchio::SE3 &g_world_knee_se3 = data_.oMf[limb.lower_fid];

  // Convert to Eigen Homogenous Matrix
  g_world_knee = g_world_knee_se3.toHomogeneousMatrix();

}

void QuadKD2::worldToKneeFKWorldFrame(int leg_index, Eigen::Vector3d &knee_pos_world) const{
  Eigen::Matrix4d g_world_knee;
  QuadKD2::worldToKneeFKWorldFrame(leg_index, g_world_knee);

  // Extract cartesian position of the foot in the world frame
  knee_pos_world = g_world_knee.block<3,1>(0,3);
}

bool QuadKD2::worldToFootIKWorldFrame(int leg_index, Eigen::Vector3d foot_pos_world,
                                        Eigen::Vector3d &joint_state) const {
  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0){
    throw std::runtime_error("Leg index is outside valid range");
  }

  const pinocchio::SE3 &g_world_body_se3 = data_.oMf[body_fid_];

  const pinocchio::SE3 &g_body_legbase_se3 = legbase_SE3_[leg_index];

  pinocchio::SE3 g_world_legbase_se3 = g_world_body_se3 * g_body_legbase_se3;

  pinocchio::SE3 g_world_foot_se3(pinocchio::SE3::Quaternion::Identity(), pinocchio::SE3::Vector3(foot_pos_world));

  pinocchio::SE3 g_legbase_foot_se3 = g_world_legbase_se3.inverse() * g_world_foot_se3;

  Eigen::Vector3d foot_pos_legbase = g_legbase_foot_se3.translation();
  return legbaseToFootIKLegbaseFrame(leg_index, foot_pos_legbase, joint_state);
}

bool QuadKD2::legbaseToFootIKLegbaseFrame(int leg_index,
                                         Eigen::Vector3d foot_pos_legbase,
                                         Eigen::Vector3d &joint_state) const {
  // Initialize exact bool
  bool is_exact = true;

  // Calculate offsets
  Eigen::Vector3d legbase_offset = legbase_offsets_[leg_index];
  double l0 = l0_vec_[leg_index];

  // Extract coordinates and declare joint variables
  double x = foot_pos_legbase[0];
  double y = foot_pos_legbase[1];
  double z = foot_pos_legbase[2];
  double q0;
  double q1;
  double q2;

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

  // Make sure abad is within joint limits, clamp otherwise
  if (q0 > joint_max_[leg_index][0] || q0 < joint_min_[leg_index][0]) {
    q0 = std::max(std::min(q0, joint_max_[leg_index][0]),
                  joint_min_[leg_index][0]);
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

  // Make sure hip is within joint limits
  if (q1 > joint_max_[leg_index][1] || q1 < joint_min_[leg_index][1]) {
    q1 = std::max(std::min(q1, joint_max_[leg_index][1]),
                  joint_min_[leg_index][1]);
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

  // Make sure knee is within joint limits
  if (q2 > joint_max_[leg_index][2] || q2 < joint_min_[leg_index][2]) {
    q2 = std::max(std::min(q2, joint_max_[leg_index][2]),
                  joint_min_[leg_index][2]);
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

  joint_state = {q0, q1, q2};
  return is_exact;
}

void QuadKD2::getJacobianGenCoord(const Eigen::VectorXd &state,
                                 Eigen::MatrixXd &jacobian) const {
  this->getJacobianBodyAngVel(state, jacobian);

  // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame,
  // which is multiplied by Jacobian to map it to Euler angle change rate here
  for (size_t i = 0; i < 4; i++) {
    Eigen::MatrixXd transform_jac(3, 3);
    transform_jac << 1, 0, -sin(state(16)), 0, cos(state(15)),
        cos(state(16)) * sin(state(15)), 0, -sin(state(15)),
        cos(state(15)) * cos(state(16));
    jacobian.block(3 * i, 15, 3, 3) =
        jacobian.block(3 * i, 15, 3, 3) * transform_jac;
  }
}

void QuadKD2::getJacobianBodyAngVel(const Eigen::VectorXd &state,
                           Eigen::MatrixXd &jacobian) const{

}

void QuadKD2::getJacobianWorldAngVel(const Eigen::VectorXd &state,
                           Eigen::MatrixXd &jacobian) const{

}