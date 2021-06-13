#include "spirit_utils/kinematics.h"

using namespace spirit_utils;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

SpiritKinematics::SpiritKinematics() {
  
  legbase_offsets_.resize(4);
  legbase_offsets_[0] = legbase_offset_0_;
  legbase_offsets_[1] = legbase_offset_1_;
  legbase_offsets_[2] = legbase_offset_2_;
  legbase_offsets_[3] = legbase_offset_3_;

  g_body_legbases_.resize(4);
  for (int leg_index = 0; leg_index < 4; leg_index++) {

      // Compute transforms
    g_body_legbases_[leg_index] = createAffineMatrix(legbase_offsets_[leg_index], 
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
  }
}

Eigen::Matrix4d SpiritKinematics::createAffineMatrix(Eigen::Vector3d trans, 
    Eigen::Vector3d rpy) const{

  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

  return t.matrix();
}

Eigen::Matrix4d SpiritKinematics::createAffineMatrix(Eigen::Vector3d trans, 
    Eigen::AngleAxisd rot) const{

  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

double SpiritKinematics::getJointLowerLimit(int joint_index) const{
  return joint_min_[joint_index];
}

double SpiritKinematics::getJointUpperLimit(int joint_index) const {
  return joint_max_[joint_index]; 
}

double SpiritKinematics::getLinkLength(int leg_index,int link_index) const{
  switch(link_index) {
    case 0: return l0_vec_[leg_index];
    case 1: return l1_;
    case 2: return l2_;
    default: throw std::runtime_error("Invalid link index");
  }
}

void SpiritKinematics::transformBodyToWorld(Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Matrix4d transform_body, Eigen::Matrix4d &transform_world) const{

  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Get the desired transform in the world frame
  transform_world = g_world_body*transform_body;
}

void SpiritKinematics::transformWorldToBody(Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Matrix4d transform_world, Eigen::Matrix4d &transform_body) const{

  // Compute transform from world to body frame
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute the desired transform in the body frame
  transform_body = g_world_body.inverse()*transform_world;
}

void SpiritKinematics::legBaseFK(int leg_index, Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Matrix4d &g_world_legbase) const {

  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute transform for leg base relative to the world frame
  g_world_legbase = g_world_body*g_body_legbases_[leg_index];
}

void SpiritKinematics::legBaseFK(int leg_index, Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Vector3d &leg_base_pos_world) const {

  Eigen::Matrix4d g_world_legbase;
  legBaseFK(leg_index, body_pos, body_rpy, g_world_legbase);

  leg_base_pos_world = g_world_legbase.block<3,1>(0,3);
}

void SpiritKinematics::nominalFootstepFK(int leg_index, Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Vector3d &nominal_footstep_pos_world) const {

  // Compute transforms
  Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

  // Compute transform from body to legbase but offset by l0
  Eigen::Matrix4d g_body_nominal_footstep = g_body_legbases_[leg_index];
  g_body_nominal_footstep(1,3) += l0_vec_[leg_index];

  // Compute transform for offset leg base relative to the world frame
  Eigen::Matrix4d g_world_nominal_footstep = g_world_body*g_body_nominal_footstep;

  nominal_footstep_pos_world = g_world_nominal_footstep.block<3,1>(0,3);
}

void SpiritKinematics::bodyToFootFK(int leg_index, 
  Eigen::Vector3d joint_state, Eigen::Matrix4d &g_body_foot) const {

  if (leg_index > (legbase_offsets_.size()-1) || leg_index<0) {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Define hip offset
  Eigen::Vector3d hip_offset = {0,l0_vec_[leg_index],0};

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
  g_body_foot = g_body_legbases_[leg_index]*g_legbase_abad*
      g_abad_hip*g_hip_knee*g_knee_foot;

}

void SpiritKinematics::bodyToFootFK(int leg_index, 
  Eigen::Vector3d joint_state, Eigen::Vector3d &foot_pos_body) const {

  Eigen::Matrix4d g_body_foot;
  SpiritKinematics::bodyToFootFK(leg_index, joint_state, g_body_foot);

  // Extract cartesian position of foot
  foot_pos_body = g_body_foot.block<3,1>(0,3);
}

void SpiritKinematics::legFK(int leg_index, Eigen::Vector3d body_pos, 
  Eigen::Vector3d body_rpy, Eigen::Vector3d joint_state, 
  Eigen::Matrix4d &g_world_foot) const {

  if (leg_index > (legbase_offsets_.size()-1) || leg_index<0) {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Define hip offset
  Eigen::Vector3d hip_offset = {0,l0_vec_[leg_index],0};

  // Initialize transforms
  Eigen::Matrix4d g_body_legbase;
  Eigen::Matrix4d g_legbase_abad;
  Eigen::Matrix4d g_abad_hip;
  Eigen::Matrix4d g_hip_knee;
  Eigen::Matrix4d g_knee_foot;

    // Compute transforms
  Eigen::Matrix4d g_world_legbase;
  legBaseFK(leg_index, body_pos, body_rpy, g_world_legbase);

  g_legbase_abad = createAffineMatrix(abad_offset_,
    Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

  g_abad_hip = createAffineMatrix(hip_offset,
    Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

  g_hip_knee = createAffineMatrix(knee_offset_,
    Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

  g_knee_foot = createAffineMatrix(foot_offset_,
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

  // Get foot transform in world frame
  g_world_foot = g_world_legbase*g_legbase_abad*
      g_abad_hip*g_hip_knee*g_knee_foot;

}

void SpiritKinematics::legFK(int leg_index, Eigen::Vector3d body_pos, 
  Eigen::Vector3d body_rpy, Eigen::Vector3d joint_state, 
  Eigen::Vector3d &foot_pos_world) const {

  Eigen::Matrix4d g_world_foot;
  legFK(leg_index, body_pos, body_rpy, joint_state, g_world_foot);

  // Extract cartesian position of foot
  foot_pos_world = g_world_foot.block<3,1>(0,3);

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

void SpiritKinematics::legIK(int leg_index, Eigen::Vector3d body_pos, 
  Eigen::Vector3d body_rpy, Eigen::Vector3d foot_pos_world,
  Eigen::Vector3d &joint_state) const {

  if (leg_index > (legbase_offsets_.size()-1) || leg_index<0) {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Calculate offsets
  Eigen::Vector3d legbase_offset = legbase_offsets_[leg_index];
  double l0 = l0_vec_[leg_index];

  // Initialize transforms
  Eigen::Matrix4d g_world_legbase;
  Eigen::Matrix4d g_world_foot;
  Eigen::Matrix4d g_legbase_foot;
  Eigen::Vector3d foot_pos_rel;

  // Compute transforms
  legBaseFK(leg_index, body_pos, body_rpy, g_world_legbase);

  g_world_foot = createAffineMatrix(foot_pos_world, Eigen::AngleAxisd(
      0, Eigen::Vector3d::UnitY()));

  // Compute foot position relative to the leg base in cartesian coordinates
  g_legbase_foot = g_world_legbase.inverse()*g_world_foot;
  foot_pos_rel = g_legbase_foot.block<3,1>(0,3);

  // Extract coordinates and declare joint variables
  double x = foot_pos_rel[0];
  double y = foot_pos_rel[1];
  double z = foot_pos_rel[2];
  double q0;
  double q1;
  double q2;

  // IK closed-form solution based on Anthropomorphic Arm
  double tmp_q2 = ((l0*l0 + l1_*l1_ + l2_*l2_) - (x*x + y*y + z*z))/(2*l1_*l2_);
  tmp_q2 = std::max(std::min(tmp_q2, 1.), -1.);

  // Two possible solutions, acos(tmp_q2) and -acos(tmp_q2), but q2 should be always greater or equal to 0, std::acos returns [0, pi]
  q2 = std::acos(tmp_q2); 

  double tmp_q1 = y*y + z*z - l0*l0;
  tmp_q1 = std::max(tmp_q1, 0.);

  // Two possible solutions since square root could be positive or negative
  double s11 = std::sqrt(tmp_q1)*(l1_ - l2_*std::cos(q2)) + x*l2_*std::sin(q2);
  double c11 = std::sqrt(tmp_q1)*l2_*std::sin(q2) - x*(l1_ - l2_*std::cos(q2));
  double s12 = -std::sqrt(tmp_q1)*(l1_ - l2_*std::cos(q2)) + x*l2_*std::sin(q2);
  double c12 = -std::sqrt(tmp_q1)*l2_*std::sin(q2) - x*(l1_ - l2_*std::cos(q2));

  double q11 = std::atan2(s11, c11);
  double q12 = std::atan2(s12, c12);

  // We take the smaller one
  if (std::abs(q11) < std::abs(q12))
  {
    q1 = q11;
  }
  else
  {
    q1 = q12;
  }

  double tmp_q0 = l1_*std::sin(q1) - l2_*std::sin(q1 - q2);
  double s0 = y*tmp_q0 + z*l0;
  double c0 = y*l0 - z*tmp_q0;

  // At this point, q0 has unique solution
  q0 = std::atan2(s0, c0);

  // Clamp the results
  q0 = std::max(std::min(q0,joint_max_[0]),joint_min_[0]);
  q1 = std::max(std::min(q1,joint_max_[1]),joint_min_[1]);
  q2 = std::max(std::min(q2,joint_max_[2]),joint_min_[2]);

  joint_state = {q0,q1,q2};

  // std::cout << "Foot pos in:\n" << foot_pos_world.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Foot pos in (rel):\n" << foot_pos_rel.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Joint state out:\n" << joint_state.format(CleanFmt)
  //   << std::endl;
}

// void SpiritKinematics::legIKVel(int leg_index, Eigen::Vector3d body_state, 
//   Eigen::Vector3d foot_vel_world, Eigen::Vector3d &joint_vel) const {

//   // Compute IKVel here

// }