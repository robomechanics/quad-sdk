#include "spirit_utils/kinematics.h"

SpiritKinematics::SpiritKinematics() {
  shoulder_offsets_.push_back(shoulder_offset_0_);
  shoulder_offsets_.push_back(shoulder_offset_1_);
  shoulder_offsets_.push_back(shoulder_offset_2_);
  shoulder_offsets_.push_back(shoulder_offset_3_);
}

Eigen::Matrix4d SpiritKinematics::createAffineMatrix(Eigen::Vector3d trans, 
    Eigen::Vector3d rpy) {

  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

  return t.matrix();
}

Eigen::Matrix4d SpiritKinematics::createAffineMatrix(Eigen::Vector3d trans, 
    Eigen::AngleAxisd rot) {

  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

double SpiritKinematics::getJointLowerLimit(int joint_index) {
  return joint_min_[joint_index];
}

double SpiritKinematics::getJointUpperLimit(int joint_index) {
  return joint_max_[joint_index]; 
}

double SpiritKinematics::getLinkLength(int leg_index,int link_index) {
  switch(link_index) {
    case 0: return l0_vec_[leg_index];
    case 1: return l1_;
    case 2: return l2_;
    default: throw std::runtime_error("Invalid link index");
  }
}

void SpiritKinematics::legBaseFK(int leg_index, Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Matrix4d &g_world_shoulder) {

  // Calculate offset
  Eigen::Vector3d shoulder_offset = shoulder_offsets_[leg_index];

  // Compute transforms
  Eigen::Matrix4d g_world_base = createAffineMatrix(body_pos, body_rpy);
  Eigen::Matrix4d g_base_shoulder = createAffineMatrix(shoulder_offset, 
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

  // Compute transform for leg base relative to the world frame
  g_world_shoulder = g_world_base*g_base_shoulder;
}

void SpiritKinematics::legBaseFK(int leg_index, Eigen::Vector3d body_pos,
  Eigen::Vector3d body_rpy, Eigen::Vector3d &leg_base_pos_world) {

  Eigen::Matrix4d g_world_shoulder;
  legBaseFK(leg_index, body_pos, body_rpy, g_world_shoulder);

  leg_base_pos_world = g_world_shoulder.block<3,1>(0,3);
}

void SpiritKinematics::legFK(int leg_index, Eigen::Vector3d body_pos, 
  Eigen::Vector3d body_rpy, Eigen::Vector3d joint_state, 
  Eigen::Vector3d &foot_pos_world) {

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  if (leg_index > (shoulder_offsets_.size()-1) || leg_index<0) {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Define trig identities
  double s0 = sin(joint_state[0]);
  double s1 = sin(joint_state[1]);
  double s2 = sin(joint_state[2]);
  double c0 = cos(joint_state[0]);
  double c1 = cos(joint_state[1]);
  double c2 = cos(joint_state[2]);
  double l0 = l0_vec_[leg_index];

  // Calculate offsets
  Eigen::Vector3d shoulder_offset = shoulder_offsets_[leg_index];
  Eigen::Vector3d abad_offset = {0,l0*c0,l0*s0};
  Eigen::Vector3d upper_offset = {-l1_*c1,0,-l1_*s1};
  Eigen::Vector3d lower_offset = {l2_*c2,0,-l2_*s2};

  // Initialize transforms
  Eigen::Matrix4d g_world_base;
  Eigen::Matrix4d g_base_shoulder;
  Eigen::Matrix4d g_shoulder_abad;
  Eigen::Matrix4d g_abad_upper;
  Eigen::Matrix4d g_upper_lower;
  Eigen::Matrix4d g_world_foot;

  // Compute transforms
  Eigen::Matrix4d g_world_shoulder;
  legBaseFK(leg_index, body_pos, body_rpy, g_world_shoulder);

  g_shoulder_abad = createAffineMatrix(abad_offset,
    Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

  g_abad_upper = createAffineMatrix(upper_offset,
    Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

  g_upper_lower = createAffineMatrix(lower_offset,
    Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

  // Get foot transform in world frame
  g_world_foot = g_world_shoulder*g_shoulder_abad*
      g_abad_upper*g_upper_lower;

  // Extract cartesian position of foot
  foot_pos_world = g_world_foot.block<3,1>(0,3);

  // std::cout << "World to shoulder\n" << g_world_shoulder.format(CleanFmt)
  //   << std::endl;
  // std::cout << "shoulder to abad\n" << g_shoulder_abad.format(CleanFmt)
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
  Eigen::Vector3d &joint_state) {

  if (leg_index > (shoulder_offsets_.size()-1) || leg_index<0) {
    throw std::runtime_error("Leg index is outside valid range");
  }

  // Calculate offsets
  Eigen::Vector3d shoulder_offset = shoulder_offsets_[leg_index];
  double l0 = l0_vec_[leg_index];

  // Initialize transforms
  Eigen::Matrix4d g_world_shoulder;
  Eigen::Matrix4d g_world_foot;
  Eigen::Matrix4d g_shoulder_foot;
  Eigen::Vector3d foot_pos_rel;

  // Compute transforms
  legBaseFK(leg_index, body_pos, body_rpy, g_world_shoulder);

  g_world_foot = createAffineMatrix(foot_pos_world, Eigen::AngleAxisd(
      0, Eigen::Vector3d::UnitY()));

  // Compute foot position relative to the leg base in cartesian coordinates
  g_shoulder_foot = g_world_shoulder.inverse()*g_world_foot;
  foot_pos_rel = g_shoulder_foot.block<3,1>(0,3);

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
    ROS_WARN_THROTTLE(0.5,"Foot too close, choosing closest alternative\n");
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
    ROS_WARN_THROTTLE(0.5,"Abad limits exceeded, clamping to %5.3f \n", q0);
  }

  // Rotate to ab-ad fixed frame
  double z_body_frame = z;
  z = -sin(q0)*y + cos(q0)*z_body_frame;

  // Check reachibility for hip
  double acos_eps = 1.0;
  double temp2 = (l1_*l1_ + x*x + z*z - l2_*l2_)/(2*l1_*sqrt(x*x+z*z));
  if (abs(temp2) > acos_eps) {
    ROS_WARN_THROTTLE(0.5,"Foot location too far for hip, choosing closest"
      " alternative \n");
    temp2 = std::max(std::min(temp2,acos_eps),-acos_eps);
  }

  // Check reachibility for knee
  double temp3 = (l1_*l1_ + l2_*l2_ - x*x - z*z)/(2*l1_*l2_);
  if (temp3 > acos_eps  || temp3 < -acos_eps) {
    ROS_WARN_THROTTLE(0.5,"Foot location too far for knee, choosing closest"
      " alternative \n");
    temp3 = std::max(std::min(temp3,acos_eps),-acos_eps);
  }

  // Compute joint angles
  q1 = 0.5*M_PI + atan2(x,-z) - acos(temp2);
  q2 = acos(temp3);

  // Make sure hip is within joint limits (try other direction if fails)
  if (q1 > joint_max_[1] || q1 < joint_min_[1]) {
    ROS_WARN_THROTTLE(0.5,"Hip limits exceeded, using inverted config\n");

    q0 = q0_inverted;
    z = -sin(q0)*y + cos(q0)*z_body_frame;
    q1 = 0.5*M_PI + atan2(x,-z) - acos(temp2);
    q2 = acos(temp3);

    if (q1 > joint_max_[1] || q1 < joint_min_[1]) {
      q1 = std::max(std::min(q1,joint_max_[1]),joint_min_[1]);
      ROS_WARN_THROTTLE(0.5,"Hip limits exceeded, clamping to %5.3f \n", q1);
    }
  }

  // Make sure knee is within joint limits
  if (q2 >= joint_max_[2]) {
    q2 = std::max(std::min(q2,joint_max_[2]),joint_min_[2]);
    ROS_WARN_THROTTLE(0.5,"Knee max exceeded, clamping to %5.3f \n", q2);
  } else if (q2 <= joint_min_[2]) {
    q2 = joint_min_[2];
    ROS_WARN_THROTTLE(0.5,"Knee minimum exceeded, clamping to %5.3f \n", q2);
  }

  // q1 is undefined if q2=0, resolve this
  if (q2==0) {
    q1 = 0;
    ROS_WARN_THROTTLE(0.5,"Hip value undefined (in singularity), setting to"
      " %5.3f \n", q1);
  }

  if (z_body_frame - l0*sin(q0) > 0) {
    ROS_WARN_THROTTLE(0.5,"IK solution is in hip-inverted region! Beware!\n");
  }

  joint_state = {q0,q1,q2};

  // std::cout << "Foot pos in:\n" << foot_pos_world.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Foot pos in (rel):\n" << foot_pos_rel.format(CleanFmt)
  //   << std::endl;
  // std::cout << "Joint state out:\n" << joint_state.format(CleanFmt)
  //   << std::endl;
}