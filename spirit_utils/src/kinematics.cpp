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

  // Start IK, check foot pos is at least l0 away from leg base, clamp otherwise
  double temp = l0/sqrt(z*z+y*y);
  if (abs(temp) > 1) {
    ROS_DEBUG_THROTTLE(0.5,"Foot too close, choosing closest alternative\n");
    temp = std::max(std::min(temp,1.0),-1.0);
  }

  // Compute both solutions of q0, use hip-above-knee if z<0 (preferred)
  // Store the inverted solution in case hip limits are exceeded
  double q0_inverted;
  if (z>0) {
    q0 = -acos(temp) + atan2(z,y);
    q0_inverted = acos(temp) + atan2(z,y);
  } else {
    q0 = acos(temp) + atan2(z,y);
    q0_inverted = -acos(temp) + atan2(z,y);
  }

  // Make sure abad is within joint limits, clamp otherwise
  if (q0 > joint_max_[0] || q0 < joint_min_[0]) {
    q0 = std::max(std::min(q0,joint_max_[0]),joint_min_[0]);
    ROS_DEBUG_THROTTLE(0.5,"Abad limits exceeded, clamping to %5.3f \n", q0);
  }

  // Rotate to ab-ad fixed frame
  double z_body_frame = z;
  z = -sin(q0)*y + cos(q0)*z_body_frame;

  // Check reachibility for hip
  double acos_eps = 1.0;
  double temp2 = (l1_*l1_ + x*x + z*z - l2_*l2_)/(2*l1_*sqrt(x*x+z*z));
  if (abs(temp2) > acos_eps) {
    ROS_DEBUG_THROTTLE(0.5,"Foot location too far for hip, choosing closest"
      " alternative \n");
    temp2 = std::max(std::min(temp2,acos_eps),-acos_eps);
  }

  // Check reachibility for knee
  double temp3 = (l1_*l1_ + l2_*l2_ - x*x - z*z)/(2*l1_*l2_);
  if (temp3 > acos_eps  || temp3 < -acos_eps) {
    ROS_DEBUG_THROTTLE(0.5,"Foot location too far for knee, choosing closest"
      " alternative \n");
    temp3 = std::max(std::min(temp3,acos_eps),-acos_eps);
  }

  // Compute joint angles
  q1 = 0.5*M_PI + atan2(x,-z) - acos(temp2);
  q2 = acos(temp3);

  // Make sure hip is within joint limits (try other direction if fails)
  if (q1 > joint_max_[1] || q1 < joint_min_[1]) {
    ROS_DEBUG_THROTTLE(0.5,"Hip limits exceeded, using inverted config\n");

    q0 = q0_inverted;
    z = -sin(q0)*y + cos(q0)*z_body_frame;
    q1 = 0.5*M_PI + atan2(x,-z) - acos(temp2);
    q2 = acos(temp3);

    if (q1 > joint_max_[1] || q1 < joint_min_[1]) {
      q1 = std::max(std::min(q1,joint_max_[1]),joint_min_[1]);
      ROS_DEBUG_THROTTLE(0.5,"Hip limits exceeded, clamping to %5.3f \n", q1);
    }
  }

  // Make sure knee is within joint limits
  if (q2 > joint_max_[2] || q2 < joint_min_[2]) {
    q2 = std::max(std::min(q2,joint_max_[2]),joint_min_[2]);
    ROS_DEBUG_THROTTLE(0.5,"Knee limit exceeded, clamping to %5.3f \n", q2);
  }

  // q1 is undefined if q2=0, resolve this
  if (q2==0) {
    q1 = 0;
    ROS_DEBUG_THROTTLE(0.5,"Hip value undefined (in singularity), setting to"
      " %5.3f \n", q1);
  }

  if (z_body_frame - l0*sin(q0) > 0) {
    ROS_DEBUG_THROTTLE(0.5,"IK solution is in hip-inverted region! Beware!\n");
  }

  joint_state = {q0,q1,q2};

  // std::cout << "Foot pos in:\n" << foot_pos_world.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Foot pos in (rel):\n" << foot_pos_rel.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Joint state out:\n" << joint_state.format(CleanFmt)
  //   << std::endl;
}