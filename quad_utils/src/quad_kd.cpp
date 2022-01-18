#include "quad_utils/quad_kd.h"

using namespace quad_utils;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

QuadKD::QuadKD()
{
  std::string robot_description_string;
  if (!ros::param::get("robot_description", robot_description_string))
  {
    std::cerr << "Error loading robot_description " << std::endl;
    abort();
  }

  model_ = new RigidBodyDynamics::Model();
  if (!RigidBodyDynamics::Addons::URDFReadFromString(robot_description_string.c_str(), model_, true))
  {
    std::cerr << "Error loading model " << std::endl;
    abort();
  }

  body_name_list_ = {"toe0", "toe1", "toe2", "toe3"};

  body_id_list_.resize(4);
  for (size_t i = 0; i < body_name_list_.size(); i++)
  {
    body_id_list_.at(i) = model_->GetBodyId(body_name_list_.at(i).c_str());
  }

  leg_idx_list_.resize(4);
  std::iota(leg_idx_list_.begin(), leg_idx_list_.end(), 0);
  std::sort(leg_idx_list_.begin(), leg_idx_list_.end(), [&](int i, int j)
            { return body_id_list_.at(i) < body_id_list_.at(j); });

  // Read leg geometry from URDF
  legbase_offsets_.resize(4);
  l0_vec_.resize(4);
  std::vector<std::string> hip_name_list = {"hip0", "hip1", "hip2", "hip3"};
  std::vector<std::string> upper_name_list = {"upper0", "upper1", "upper2", "upper3"};
  std::vector<std::string> lower_name_list = {"lower0", "lower1", "lower2", "lower3"};
  std::vector<std::string> toe_name_list = {"toe0", "toe1", "toe2", "toe3"};
  RigidBodyDynamics::Math::SpatialTransform tform;
  for (size_t i = 0; i < 4; i++)
  {
    // From body COM to abad
    tform = model_->GetJointFrame(model_->GetBodyId(hip_name_list.at(i).c_str()));
    legbase_offsets_[i] = tform.r;

    // From abad to hip
    tform = model_->GetJointFrame(model_->GetBodyId(upper_name_list.at(i).c_str()));
    l0_vec_[i] = tform.r(1);

    // From hip to knee (we know they should be the same and the equation in IK uses the magnitute of it)
    tform = model_->GetJointFrame(model_->GetBodyId(lower_name_list.at(i).c_str()));
    l1_ = abs(tform.r(0));
    knee_offset_ = tform.r;

    // From knee to toe (we know they should be the same and the equation in IK uses the magnitute of it)
    tform = model_->GetJointFrame(model_->GetBodyId(toe_name_list.at(i).c_str()));
    l2_ = abs(tform.r(0));
    foot_offset_ = tform.r;
  }

  // Abad offset from legbase
  abad_offset_ = {0, 0, 0};

  g_body_legbases_.resize(4);
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // Compute transforms
    g_body_legbases_[leg_index] = createAffineMatrix(legbase_offsets_[leg_index],
                                                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
  }
}

Eigen::Matrix4d QuadKD::createAffineMatrix(Eigen::Vector3d trans,
                                                     Eigen::Vector3d rpy) const
{

  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

  return t.matrix();
}

Eigen::Matrix4d QuadKD::createAffineMatrix(Eigen::Vector3d trans,
                                                     Eigen::AngleAxisd rot) const
{

  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

double QuadKD::getJointLowerLimit(int joint_index) const
{
  return joint_min_[joint_index];
}

double QuadKD::getJointUpperLimit(int joint_index) const
{
  return joint_max_[joint_index];
}

double QuadKD::getLinkLength(int leg_index, int link_index) const
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

void QuadKD::transformBodyToWorld(Eigen::Vector3d body_pos,
                                            Eigen::Vector3d body_rpy, Eigen::Matrix4d transform_body, Eigen::Matrix4d &transform_world) const
{

  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Get the desired transform in the world frame
  transform_world = g_world_body * transform_body;
}

void QuadKD::transformWorldToBody(Eigen::Vector3d body_pos,
                                            Eigen::Vector3d body_rpy, Eigen::Matrix4d transform_world, Eigen::Matrix4d &transform_body) const
{

  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute the desired transform in the body frame
  transform_body = g_world_body.inverse() * transform_world;
}

void QuadKD::worldToLegbaseFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                                 Eigen::Vector3d body_rpy, Eigen::Matrix4d &g_world_legbase) const
{

  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute transform for leg base relative to the world frame
  g_world_legbase = g_world_body * g_body_legbases_[leg_index];
}

void QuadKD::worldToLegbaseFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                                 Eigen::Vector3d body_rpy, Eigen::Vector3d &leg_base_pos_world) const
{

  Eigen::Matrix4d g_world_legbase;
  worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

  leg_base_pos_world = g_world_legbase.block<3, 1>(0, 3);
}

void QuadKD::worldToNominalHipFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                                    Eigen::Vector3d body_rpy, Eigen::Vector3d &nominal_hip_pos_world) const
{

  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute transform from body to legbase but offset by l0
  Eigen::Matrix4d g_body_nominal_hip = g_body_legbases_[leg_index];
  g_body_nominal_hip(1, 3) += 1.0*l0_vec_[leg_index];

  // Compute transform for offset leg base relative to the world frame
  Eigen::Matrix4d g_world_nominal_hip = g_world_body * g_body_nominal_hip;

  nominal_hip_pos_world = g_world_nominal_hip.block<3, 1>(0, 3);
}

void QuadKD::bodyToFootFKBodyFrame(int leg_index,
                                    Eigen::Vector3d joint_state, Eigen::Matrix4d &g_body_foot) const
{

  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0)
  {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Define hip offset
  Eigen::Vector3d hip_offset = {0, l0_vec_[leg_index], 0};

  // Initialize transforms
  Eigen::Matrix4d g_legbase_abad;
  Eigen::Matrix4d g_abad_hip;
  Eigen::Matrix4d g_hip_knee;
  Eigen::Matrix4d g_knee_foot;

  g_legbase_abad = createAffineMatrix(abad_offset_,
                                      Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

  g_abad_hip = createAffineMatrix(hip_offset,
                                  Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

  g_hip_knee = createAffineMatrix(knee_offset_,
                                  Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

  g_knee_foot = createAffineMatrix(foot_offset_,
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

  // Get foot transform in world frame
  g_body_foot = g_body_legbases_[leg_index] * g_legbase_abad *
                g_abad_hip * g_hip_knee * g_knee_foot;
}

void QuadKD::bodyToFootFKBodyFrame(int leg_index,
                                    Eigen::Vector3d joint_state, Eigen::Vector3d &foot_pos_body) const
{

  Eigen::Matrix4d g_body_foot;
  QuadKD::bodyToFootFKBodyFrame(leg_index, joint_state, g_body_foot);

  // Extract cartesian position of foot
  foot_pos_body = g_body_foot.block<3, 1>(0, 3);
}

void QuadKD::worldToFootFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                             Eigen::Vector3d body_rpy, Eigen::Vector3d joint_state,
                             Eigen::Matrix4d &g_world_foot) const
{

  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0)
  {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Define hip offset
  Eigen::Vector3d hip_offset = {0, l0_vec_[leg_index], 0};

  // Initialize transforms
  Eigen::Matrix4d g_body_legbase;
  Eigen::Matrix4d g_legbase_abad;
  Eigen::Matrix4d g_abad_hip;
  Eigen::Matrix4d g_hip_knee;
  Eigen::Matrix4d g_knee_foot;

  // Compute transforms
  Eigen::Matrix4d g_world_legbase;
  worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

  g_legbase_abad = createAffineMatrix(abad_offset_,
                                      Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

  g_abad_hip = createAffineMatrix(hip_offset,
                                  Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

  g_hip_knee = createAffineMatrix(knee_offset_,
                                  Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

  g_knee_foot = createAffineMatrix(foot_offset_,
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

  // Get foot transform in world frame
  g_world_foot = g_world_legbase * g_legbase_abad *
                 g_abad_hip * g_hip_knee * g_knee_foot;
}

void QuadKD::worldToFootFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                             Eigen::Vector3d body_rpy, Eigen::Vector3d joint_state,
                             Eigen::Vector3d &foot_pos_world) const
{

  Eigen::Matrix4d g_world_foot;
  worldToFootFKWorldFrame(leg_index, body_pos, body_rpy, joint_state, g_world_foot);

  // Extract cartesian position of foot
  foot_pos_world = g_world_foot.block<3, 1>(0, 3);

  // std::cout << "World to legbase\n" << g_world_legbase.format(CleanFmt)
  //   << std::endl;
  // std::cout << "legbase to abad\n" << g_legbase_abad.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Abad to upper\n" << g_abad_upper.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Upper to lower\n" << g_upper_lower.format(CleanFmt)
  //   << std::endl;
  // std::cout << "World to lower\n" << g_world_foot.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Joint state in:\n" << joint_state.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Foot pos out:\n" << foot_pos_world.format(CleanFmt)
  //   << std::endl;
}


void QuadKD::worldToFootIKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                             Eigen::Vector3d body_rpy, Eigen::Vector3d foot_pos_world,
                             Eigen::Vector3d &joint_state) const
{

  if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0)
  {
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

  g_world_foot = createAffineMatrix(foot_pos_world, Eigen::AngleAxisd(
                                                        0, Eigen::Vector3d::UnitY()));

  // Compute foot position relative to the leg base in cartesian coordinates
  g_legbase_foot = g_world_legbase.inverse() * g_world_foot;
  foot_pos_legbase = g_legbase_foot.block<3, 1>(0, 3);

  legbaseToFootIKLegbaseFrame(leg_index, foot_pos_legbase, joint_state);
}

void QuadKD::legbaseToFootIKLegbaseFrame(int leg_index, Eigen::Vector3d foot_pos_legbase,
                             Eigen::Vector3d &joint_state) const
{

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
  if (abs(temp) > 1)
  {
    ROS_DEBUG_THROTTLE(0.5, "Foot too close, choosing closest alternative\n");
    temp = std::max(std::min(temp, 1.0), -1.0);
  }

  // Compute both solutions of q0, use hip-above-knee if z<0 (preferred)
  // Store the inverted solution in case hip limits are exceeded
  double q0_inverted;
  if (z > 0)
  {
    q0 = -acos(temp) + atan2(z, y);
    q0_inverted = acos(temp) + atan2(z, y);
  }
  else
  {
    q0 = acos(temp) + atan2(z, y);
    q0_inverted = -acos(temp) + atan2(z, y);
  }

  // Make sure abad is within joint limits, clamp otherwise
  if (q0 > joint_max_[0] || q0 < joint_min_[0])
  {
    q0 = std::max(std::min(q0, joint_max_[0]), joint_min_[0]);
    ROS_DEBUG_THROTTLE(0.5, "Abad limits exceeded, clamping to %5.3f \n", q0);
  }

  // Rotate to ab-ad fixed frame
  double z_body_frame = z;
  z = -sin(q0) * y + cos(q0) * z_body_frame;

  // Check reachibility for hip
  double acos_eps = 1.0;
  double temp2 = (l1_ * l1_ + x * x + z * z - l2_ * l2_) / (2 * l1_ * sqrt(x * x + z * z));
  if (abs(temp2) > acos_eps)
  {
    ROS_DEBUG_THROTTLE(0.5, "Foot location too far for hip, choosing closest"
                            " alternative \n");
    temp2 = std::max(std::min(temp2, acos_eps), -acos_eps);
  }

  // Check reachibility for knee
  double temp3 = (l1_ * l1_ + l2_ * l2_ - x * x - z * z) / (2 * l1_ * l2_);
  if (temp3 > acos_eps || temp3 < -acos_eps)
  {
    ROS_DEBUG_THROTTLE(0.5, "Foot location too far for knee, choosing closest"
                            " alternative \n");
    temp3 = std::max(std::min(temp3, acos_eps), -acos_eps);
  }

  // Compute joint angles
  q1 = 0.5 * M_PI + atan2(x, -z) - acos(temp2);
  q2 = acos(temp3);

  // Make sure hip is within joint limits (try other direction if fails)
  if (q1 > joint_max_[1] || q1 < joint_min_[1])
  {
    ROS_DEBUG_THROTTLE(0.5, "Hip limits exceeded, using inverted config\n");

    q0 = q0_inverted;
    z = -sin(q0) * y + cos(q0) * z_body_frame;
    q1 = 0.5 * M_PI + atan2(x, -z) - acos(temp2);
    q2 = acos(temp3);

    if (q1 > joint_max_[1] || q1 < joint_min_[1])
    {
      q1 = std::max(std::min(q1, joint_max_[1]), joint_min_[1]);
      ROS_DEBUG_THROTTLE(0.5, "Hip limits exceeded, clamping to %5.3f \n", q1);
    }
  }

  // Make sure knee is within joint limits
  if (q2 > joint_max_[2] || q2 < joint_min_[2])
  {
    q2 = std::max(std::min(q2, joint_max_[2]), joint_min_[2]);
    ROS_DEBUG_THROTTLE(0.5, "Knee limit exceeded, clamping to %5.3f \n", q2);
  }

  // q1 is undefined if q2=0, resolve this
  if (q2 == 0)
  {
    q1 = 0;
    ROS_DEBUG_THROTTLE(0.5, "Hip value undefined (in singularity), setting to"
                            " %5.3f \n",
                       q1);
  }

  if (z_body_frame - l0 * sin(q0) > 0)
  {
    ROS_DEBUG_THROTTLE(0.5, "IK solution is in hip-inverted region! Beware!\n");
  }

  joint_state = {q0, q1, q2};
}

void QuadKD::getJacobianGenCoord(const Eigen::VectorXd &state, Eigen::MatrixXd &jacobian) const
{
  this->getJacobianBodyAngVel(state, jacobian);

  // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame, which is multiplied by Jacobian to map it to Euler angle change rate here
  for (size_t i = 0; i < 4; i++)
  {
    Eigen::MatrixXd transform_jac(3, 3);
    transform_jac << 1, 0, -sin(state(16)),
        0, cos(state(15)), cos(state(16)) * sin(state(15)),
        0, -sin(state(15)), cos(state(15)) * cos(state(16));
    jacobian.block(3 * i, 15, 3, 3) = jacobian.block(3 * i, 15, 3, 3) * transform_jac;
  }
}

void QuadKD::getJacobianBodyAngVel(const Eigen::VectorXd &state, Eigen::MatrixXd &jacobian) const
{
  assert(state.size() == 18);

  // RBDL state vector has the floating base state in the front and the joint state in the back
  // When reading from URDF, the order of the legs is 2301, which should be corrected by sorting the bodyID
  Eigen::VectorXd q(19);
  q.setZero();

  q.head(3) = state.segment(12, 3);

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(state(15), state(16), state(17));
  q(3) = quat_tf.getX();
  q(4) = quat_tf.getY();
  q(5) = quat_tf.getZ();

  // RBDL uses quaternion for floating base direction, but w is placed at the end of the state vector
  q(18) = quat_tf.getW();

  for (size_t i = 0; i < leg_idx_list_.size(); i++)
  {
    q.segment(6 + 3 * i, 3) = state.segment(3 * leg_idx_list_.at(i), 3);
  }

  jacobian.setZero();

  for (size_t i = 0; i < body_id_list_.size(); i++)
  {
    Eigen::MatrixXd jac_block(3, 18);
    jac_block.setZero();
    RigidBodyDynamics::CalcPointJacobian(*model_, q, body_id_list_.at(i), Eigen::Vector3d::Zero(), jac_block);

    for (size_t j = 0; j < 4; j++)
    {
      jacobian.block(3 * i, 3 * leg_idx_list_.at(j), 3, 3) = jac_block.block(0, 6 + 3 * j, 3, 3);
    }
    jacobian.block(3 * i, 12, 3, 6) = jac_block.block(0, 0, 3, 6);
  }
}

void QuadKD::getJacobianWorldAngVel(const Eigen::VectorXd &state, Eigen::MatrixXd &jacobian) const
{
  this->getJacobianBodyAngVel(state, jacobian);

  // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame, which is multiplied by rotation matrix to map it to angular velocity in world frame here
  for (size_t i = 0; i < 4; i++)
  {
    Eigen::Matrix3d rot;
    this->getRotationMatrix(state.segment(15, 3), rot);
    jacobian.block(3 * i, 15, 3, 3) = jacobian.block(3 * i, 15, 3, 3) * rot;
  }
}

void QuadKD::getRotationMatrix(const Eigen::VectorXd &rpy, Eigen::Matrix3d &rot) const
{
  rot = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
}

void QuadKD::computeInverseDynamics(const Eigen::VectorXd &state_pos,
                                  const Eigen::VectorXd &state_vel,
                                  const Eigen::VectorXd &foot_acc,
                                  const Eigen::VectorXd &grf,
                                  const std::vector<int> &contact_mode,
                                  Eigen::VectorXd &tau) const
{

  // Convert q, q_dot into RBDL order
  Eigen::VectorXd q(19), q_dot(18);
  q.setZero();
  q_dot.setZero();

  q.head(3) = state_pos.segment(12, 3);
  q_dot.head(3) = state_vel.segment(12, 3);

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(state_pos(15), state_pos(16), state_pos(17));
  q(3) = quat_tf.getX();
  q(4) = quat_tf.getY();
  q(5) = quat_tf.getZ();

  // RBDL uses quaternion for floating base direction, but w is placed at the end of the state vector
  q(18) = quat_tf.getW();

  q_dot.segment(3, 3) = state_vel.segment(15, 3);

  for (size_t i = 0; i < leg_idx_list_.size(); i++)
  {
    q.segment(6 + 3 * i, 3) = state_pos.segment(3 * leg_idx_list_.at(i), 3);
    q_dot.segment(6 + 3 * i, 3) = state_vel.segment(3 * leg_idx_list_.at(i), 3);
  }

  // Compute jacobians
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  jacobian.setZero();
  for (size_t i = 0; i < body_id_list_.size(); i++)
  {
    Eigen::MatrixXd jac_block(3, 18);
    jac_block.setZero();
    RigidBodyDynamics::CalcPointJacobian(*model_, q, body_id_list_.at(i), Eigen::Vector3d::Zero(), jac_block);
    jacobian.block(3 * i, 0, 3, 18) = jac_block;
  }

  // Compute the equivalent force in generalized coordinates
  Eigen::VectorXd tau_stance = -jacobian.transpose() * grf;

  // Compute EOM
  Eigen::MatrixXd M(18, 18);
  M.setZero();
  Eigen::VectorXd N(18);
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model_, q, M);
  RigidBodyDynamics::NonlinearEffects(*model_, q, q_dot, N);

  // Compute J_dot*q_dot
  Eigen::VectorXd foot_acc_J_dot(12);
  for (size_t i = 0; i < 4; i++)
  {
    foot_acc_J_dot.segment(3 * i, 3) = RigidBodyDynamics::CalcPointAcceleration(*model_, q, q_dot, Eigen::VectorXd::Zero(18), body_id_list_.at(i), Eigen::Vector3d::Zero());
  }

  // Compute acceleration from J*q_ddot
  Eigen::VectorXd foot_acc_q_ddot = foot_acc - foot_acc_J_dot;

  // Compuate damped jacobian inverser
  Eigen::MatrixXd jacobian_inv = math_utils::sdlsInv(jacobian.block(0, 6, 12, 12));

  // In the EOM, we know M, N, tau_grf, and a = J_b*q_ddot_b + J_l*q_ddot_l, we need to solve q_ddot_b and tau_swing
  Eigen::MatrixXd blk_mat = Eigen::MatrixXd::Zero(18, 18);
  blk_mat.block(0, 0, 6, 6) = -M.block(0, 0, 6, 6) + M.block(0, 6, 6, 12) * jacobian_inv * jacobian.block(0, 0, 12, 6);
  blk_mat.block(6, 0, 12, 6) = -M.block(6, 0, 12, 6) + M.block(6, 6, 12, 12) * jacobian_inv * jacobian.block(0, 0, 12, 6);
  blk_mat.block(6, 6, 12, 12).diagonal().fill(1);

  // Perform inverse dynamics
  Eigen::VectorXd tau_swing(12), blk_sol(18), tau_stance_constrained(18);
  tau_stance_constrained << tau_stance.segment(0, 6), Eigen::VectorXd::Zero(12);
  blk_sol = blk_mat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(M.block(0, 6, 18, 12) * jacobian_inv * foot_acc_q_ddot + N + tau_stance_constrained);
  tau_swing = blk_sol.segment(6, 12);

  // Convert the order back
  for (size_t i = 0; i < 4; i++)
  {
    if (contact_mode.at(leg_idx_list_.at(i))) {
      tau.segment(3 * leg_idx_list_.at(i), 3) = tau_stance.segment(6 + 3 * i, 3);
    } else {
      tau.segment(3 * leg_idx_list_.at(i), 3) = tau_swing.segment(3 * i, 3);
      // tau.segment(3 * leg_idx_list_.at(i), 3) = Eigen::VectorXd::Zero(3);
    }
  }

  // Check inf or nan
  if (!(tau.array() == tau.array()).all() || !((tau - tau).array() == (tau - tau).array()).all())
  {
    tau.setZero();
  }
}