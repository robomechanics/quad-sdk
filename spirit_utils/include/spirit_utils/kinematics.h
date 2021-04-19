#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include "spirit_utils/function_timer.h"
#include <chrono>
#include <random>
#include <math.h>

namespace spirit_utils {
 
//! A lightweight library for spirit kinematic functions
/*!
  This library includes several functions and classes to aid in spirit kinematic
  calculations. It relies on Eigen, as well as some MATLAB codegen for more 
  complicated computations that would be a pain to write out by hand.
*/
class SpiritKinematics {

  public:
  /**
   * @brief Constructor for SpiritKinematics Class
   * @return Constructed object of type SpiritKinematics
   */
  SpiritKinematics();

  /**
   * @brief Create an Eigen Eigen::Matrix4d containing a homogeneous transform 
   * from a specified translation and a roll, pitch, and yaw vector
   * @param[in] trans Translation from input frame to output frame
   * @param[in] rpy Rotation from input frame to output frame as roll, pitch, yaw
   * @return Homogenous transformation matrix
   */
  Eigen::Matrix4d createAffineMatrix(Eigen::Vector3d trans, 
      Eigen::Vector3d rpy) const;

  /**
   * @brief Create an Eigen Eigen::Matrix4d containing a homogeneous transform 
   * from a specified translation and an AngleAxis object
   * @param[in] trans Translation from input frame to output frame
   * @param[in] rot Rotation from input frame to output frame as AngleAxis
   * @return Homogenous transformation matrix
   */
  Eigen::Matrix4d createAffineMatrix(Eigen::Vector3d trans, 
      Eigen::AngleAxisd rot) const;

  /**
   * @brief Tranform a transformation matrix from the body frame to the world frame
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] transform Sprcified transform in the world frame
   */
  void transformBodyToWorld(Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy, 
    Eigen::Matrix4d transform_body, Eigen::Matrix4d &transform_world) const;

  /**
   * @brief Tranform a transformation matrix from the world frame to the body frame
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] transform Sprcified transform in the body frame
   */
  void transformWorldToBody(Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy, 
    Eigen::Matrix4d transform_world, Eigen::Matrix4d &transform_body) const;

  /**
   * @brief Compute forward kinematics for a specified leg from the body COM
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] g_body_foot Transform of the specified foot in world frame
   */
  void bodyToFootFK(int leg_index, Eigen::Vector3d joint_state, 
    Eigen::Matrix4d &g_body_foot) const;

  /**
   * @brief Compute forward kinematics for a specified leg from the body COM
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] foot_pos_world Position of the specified foot in world frame
   */
  void bodyToFootFK(int leg_index, Eigen::Vector3d joint_state, 
    Eigen::Vector3d &foot_pos_body) const;

  /**
   * @brief Compute forward kinematics for a specified leg
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] g_world_foot Transform of the specified foot in world frame
   */
  void legFK(int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
      Eigen::Vector3d joint_state, Eigen::Matrix4d &g_world_foot) const;

  /**
   * @brief Compute forward kinematics for a specified leg
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] foot_pos_world Position of the specified foot in world frame
   */
  void legFK(int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
      Eigen::Vector3d joint_state, Eigen::Vector3d &foot_pos_world) const;

  /**
   * @brief Compute inverse kinematics for a specified leg
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] foot_pos_world Position of the specified foot in world frame
   * @param[out] joint_state Joint states for the specified leg (abad, hip, knee)
   */
  void legIK(int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
      Eigen::Vector3d foot_pos_world, Eigen::Vector3d &joint_state) const;

  /**
   * @brief Get the lower joint limit of a particular joint
   * @param[in] joint_index Index for joint (0 = abad, 1 = hip, 2 = knee)
   * @return Requested joint limit
   */
  double getJointLowerLimit(int joint_index) const;

   /**
   * @brief Get the upper joint limit of a particular joint
   * @param[in] joint_index Index for joint (0 = abad, 1 = hip, 2 = knee)
   * @return Requested joint limit
   */
  double getJointUpperLimit(int joint_index) const;

  /**
   * @brief Get the upper joint limit of a particular joint
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] link_index Index for link (0 = abad, 1 = upper, 2 = lower)
   * @return Requested link length
   */
  double getLinkLength(int leg_index, int link_index) const;

  /**
   * @brief Get the transform from the world frame to the leg base
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] g_world_legbase Transformation matrix of world to leg base
   */
  void legBaseFK(int leg_index, Eigen::Vector3d body_pos,
    Eigen::Vector3d body_rpy, Eigen::Matrix4d &g_world_legbase) const;

  /**
   * @brief Get the position of the leg base frame origin in the world frame
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] leg_base_pos_world Origin of leg base frame in world frame
   */
  void legBaseFK(int leg_index, Eigen::Vector3d body_pos,
    Eigen::Vector3d body_rpy, Eigen::Vector3d &leg_base_pos_world) const;

  /**
   * @brief Get the position of the nominal footstep in the world frame
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] nominal_footstep_pos_world Location of nominal footstep in world frame
   */
  void nominalFootstepFK(int leg_index, Eigen::Vector3d body_pos,
    Eigen::Vector3d body_rpy, Eigen::Vector3d &nominal_footstep_pos_world) const;

  private:

    /// Vector of the abad link lengths 
    const std::vector<double> l0_vec_ = {0.1,0.1,-0.1,-0.1};

    /// Upper link length
    const double l1_ = 0.206;

    /// Lower link length
    const double l2_ = 0.206;

    /// Abad offset from legbase
    Eigen::Vector3d abad_offset_ = {0,0,0};

    /// Knee offset from hip
    Eigen::Vector3d knee_offset_ = {-l1_,0,0};

    /// Foot offset from knee
    Eigen::Vector3d foot_offset_ = {l2_,0,0};

    /// Leg 0 legbase offset
    const Eigen::Vector3d legbase_offset_0_ = {0.2263, 0.07, 0};

    /// Leg 1 legbase offset
    const Eigen::Vector3d legbase_offset_1_ = {-0.2263, 0.07, 0};

    /// Leg 2 legbase offset
    const Eigen::Vector3d legbase_offset_2_ = {0.2263, -0.07, 0};

    /// Leg 3 legbase offset
    const Eigen::Vector3d legbase_offset_3_ = {-0.2263, -0.07, 0};

    /// Vector of legbase offsets
    std::vector<Eigen::Vector3d> legbase_offsets_;

    /// Vector of legbase offsets
    std::vector<Eigen::Matrix4d> g_body_legbases_;

    /// Epsilon offset for joint bounds
    const double joint_eps = 0.1;

    /// Vector of the joint lower limits 
    const std::vector<double> joint_min_ = {-1+joint_eps,-0.5*M_PI + joint_eps,joint_eps};

    /// Vector of the joint upper limits
    const std::vector<double> joint_max_ = {1-joint_eps,M_PI-joint_eps,M_PI-joint_eps};
};

}

#endif // KINEMATICS_H