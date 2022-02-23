#ifndef QUAD_KD_H
#define QUAD_KD_H

#include <math.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>
#include <chrono>
#include <grid_map_core/GridMap.hpp>
#include <random>
#include <vector>

#include "quad_utils/function_timer.h"
#include "quad_utils/math_utils.h"

namespace quad_utils {

//! A lightweight library for quad kinematic functions
/*!
  This library includes several functions and classes to aid in quad kinematic
  calculations. It relies on Eigen, as well as some MATLAB codegen for more
  complicated computations that would be a pain to write out by hand.
*/
class QuadKD {
 public:
  /**
   * @brief Constructor for QuadKD Class
   * @return Constructed object of type QuadKD
   */
  QuadKD();

  /**
   * @brief Create an Eigen Eigen::Matrix4d containing a homogeneous transform
   * from a specified translation and a roll, pitch, and yaw vector
   * @param[in] trans Translation from input frame to output frame
   * @param[in] rpy Rotation from input frame to output frame as roll, pitch,
   * yaw
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
   * @brief Transform a transformation matrix from the body frame to the world
   * frame
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] transform_body Specified transform in the body frame
   * @param[out] transform_world Specified transform in the world frame
   */
  void transformBodyToWorld(Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
                            Eigen::Matrix4d transform_body,
                            Eigen::Matrix4d &transform_world) const;

  /**
   * @brief Transform a transformation matrix from the world frame to the body
   * frame
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] transform_world Specified transform in the world frame
   * @param[out] transform_body Specified transform in the body frame
   */
  void transformWorldToBody(Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
                            Eigen::Matrix4d transform_world,
                            Eigen::Matrix4d &transform_body) const;

  /**
   * @brief Compute forward kinematics for a specified leg from the body COM
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] g_body_foot Transform of the specified foot in world frame
   */
  void bodyToFootFKBodyFrame(int leg_index, Eigen::Vector3d joint_state,
                             Eigen::Matrix4d &g_body_foot) const;

  /**
   * @brief Compute forward kinematics for a specified leg from the body COM
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] foot_pos_world Position of the specified foot in world frame
   */
  void bodyToFootFKBodyFrame(int leg_index, Eigen::Vector3d joint_state,
                             Eigen::Vector3d &foot_pos_body) const;

  /**
   * @brief Compute forward kinematics for a specified leg
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] g_world_foot Transform of the specified foot in world frame
   */
  void worldToFootFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                               Eigen::Vector3d body_rpy,
                               Eigen::Vector3d joint_state,
                               Eigen::Matrix4d &g_world_foot) const;

  /**
   * @brief Compute forward kinematics for a specified leg
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] foot_pos_world Position of the specified foot in world frame
   */
  void worldToFootFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                               Eigen::Vector3d body_rpy,
                               Eigen::Vector3d joint_state,
                               Eigen::Vector3d &foot_pos_world) const;

  /**
   * @brief Compute forward kinematics for a specified leg
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] g_world_knee Transform of the specified knee in world frame
   */
  void worldToKneeFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                               Eigen::Vector3d body_rpy,
                               Eigen::Vector3d joint_state,
                               Eigen::Matrix4d &g_world_knee) const;

  /**
   * @brief Compute forward kinematics for a specified leg
   * @param[in] leg_index Spirit leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] joint_state Joint states for the specified leg (abad, hip, knee)
   * @param[out] knee_pos_world Position of the specified knee in world frame
   */
  void worldToKneeFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                               Eigen::Vector3d body_rpy,
                               Eigen::Vector3d joint_state,
                               Eigen::Vector3d &knee_pos_world) const;

  /**
   * @brief Compute inverse kinematics for a specified leg
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[in] foot_pos_world Position of the specified foot in world frame
   * @param[out] joint_state Joint states for the specified leg (abad, hip,
   * knee)
   */
  bool worldToFootIKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                               Eigen::Vector3d body_rpy,
                               Eigen::Vector3d foot_pos_world,
                               Eigen::Vector3d &joint_state) const;

  /**
   * @brief Compute inverse kinematics for a specified leg in the leg base frame
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] foot_pos_legbase Position of the specified foot in leg base
   * frame
   * @param[out] joint_state Joint states for the specified leg (abad, hip,
   * knee)
   */
  bool legbaseToFootIKLegbaseFrame(int leg_index,
                                   Eigen::Vector3d foot_pos_legbase,
                                   Eigen::Vector3d &joint_state) const;

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
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] link_index Index for link (0 = abad, 1 = upper, 2 = lower)
   * @return Requested link length
   */
  double getLinkLength(int leg_index, int link_index) const;

  /**
   * @brief Get the transform from the world frame to the leg base
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] g_world_legbase Transformation matrix of world to leg base
   */
  void worldToLegbaseFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                                  Eigen::Vector3d body_rpy,
                                  Eigen::Matrix4d &g_world_legbase) const;

  /**
   * @brief Get the position of the leg base frame origin in the world frame
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] leg_base_pos_world Origin of leg base frame in world frame
   */
  void worldToLegbaseFKWorldFrame(int leg_index, Eigen::Vector3d body_pos,
                                  Eigen::Vector3d body_rpy,
                                  Eigen::Vector3d &leg_base_pos_world) const;

  /**
   * @brief Get the position of the nominal hip location in the world frame
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] body_pos Position of center of body frame
   * @param[in] body_rpy Orientation of body frame in roll, pitch, yaw
   * @param[out] nominal_hip_pos_world Location of nominal hip in world frame
   */
  void worldToNominalHipFKWorldFrame(
      int leg_index, Eigen::Vector3d body_pos, Eigen::Vector3d body_rpy,
      Eigen::Vector3d &nominal_hip_pos_world) const;

  /**
   * @brief Compute Jacobian for generalized coordinates
   * @param[in] state Joint and body states
   * @param[out] jacobian Jacobian for generalized coordinates
   */
  void getJacobianGenCoord(const Eigen::VectorXd &state,
                           Eigen::MatrixXd &jacobian) const;

  /**
   * @brief Compute Jacobian for angular velocity in body frame
   * @param[in] state Joint and body states
   * @param[out] jacobian Jacobian for angular velocity in body frame
   */
  void getJacobianBodyAngVel(const Eigen::VectorXd &state,
                             Eigen::MatrixXd &jacobian) const;

  /**
   * @brief Compute Jacobian for angular velocity in world frame
   * @param[in] state Joint and body states
   * @param[out] jacobian Jacobian for angular velocity in world frame
   */
  void getJacobianWorldAngVel(const Eigen::VectorXd &state,
                              Eigen::MatrixXd &jacobian) const;

  /**
   * @brief Compute rotation matrix given roll pitch and yaw
   * @param[in] rpy Roll pitch and yaw
   * @param[out] rot Rotation matrix
   */
  void getRotationMatrix(const Eigen::VectorXd &rpy,
                         Eigen::Matrix3d &rot) const;

  /**
   * @brief Compute inverse dynamics for swing leg
   * @param[in] state_pos Position states
   * @param[in] state_vel Velocity states
   * @param[in] foot_acc Foot absolute acceleration in world frame
   * @param[in] grf Ground reaction force
   * @param[in] contact_mode Contact mode of the legs
   * @param[out] tau Joint torques
   */
  void computeInverseDynamics(const Eigen::VectorXd &state_pos,
                              const Eigen::VectorXd &state_vel,
                              const Eigen::VectorXd &foot_acc,
                              const Eigen::VectorXd &grf,
                              const std::vector<int> &contact_mode,
                              Eigen::VectorXd &tau) const;

  /**
   * @brief Convert centroidal model states (foot coordinates and grfs) to full
   * body (joints and torques)
   * @param[in] body_state Position states
   * @param[in] foot_positions Foot positions in the world frame
   * @param[in] foot_velocities Foot velocities in the world frame
   * @param[in] grfs Ground reaction forces
   * @param[out] joint_positions Joint positions
   * @param[out] joint_velocities Joint velocities
   * @param[out] tau Joint torques
   * @return boolean for exactness of kinematics
   */
  bool convertCentroidalToFullBody(const Eigen::VectorXd &body_state,
                                   const Eigen::VectorXd &foot_positions,
                                   const Eigen::VectorXd &foot_velocities,
                                   const Eigen::VectorXd &grfs,
                                   Eigen::VectorXd &joint_positions,
                                   Eigen::VectorXd &joint_velocities,
                                   Eigen::VectorXd &torques);

  /**
   * @brief Apply a uniform maximum torque to a given set of joint torques
   * @param[in] torques Joint torques. in Nm
   * @param[in] constrained_torques Joint torques after applying max, in Nm
   * @return Boolean to indicate if initial torques is feasible (checks if
   * torques == constrained_torques)
   */
  bool applyMotorModel(const Eigen::VectorXd &torques,
                       Eigen::VectorXd &constrained_torques);

  /**
   * @brief Apply a linear motor model to a given set of joint torques and
   * velocities
   * @param[in] torques Joint torques. in Nm
   * @param[in] joint_velocities Velocities of each joint. in rad/s
   * @param[in] constrained_torques Joint torques after applying motor model, in
   * Nm
   * @return Boolean to indicate if initial torques is feasible (checks if
   * torques == constrained_torques)
   */
  bool applyMotorModel(const Eigen::VectorXd &torques,
                       const Eigen::VectorXd &joint_velocities,
                       Eigen::VectorXd &constrained_torques);

  /**
   * @brief Check if state is valid
   * @param[in] body_state Robot body positions and velocities
   * @param[in] joint_state Joint positions and velocities
   * @param[in] torques Joint torques
   * @param[in] terrain Map of the terrain for collision checking
   * @return Boolean for state validity
   */
  bool isValidFullState(const Eigen::VectorXd &body_state,
                        const Eigen::VectorXd &joint_state,
                        const Eigen::VectorXd &torques,
                        const grid_map::GridMap &terrain,
                        Eigen::VectorXd &state_violation,
                        Eigen::VectorXd &control_violation);

  /**
   * @brief Check if state is valid
   * @param[in] body_state Robot body positions and velocities
   * @param[in] foot_positions Foot positions
   * @param[in] foot_velocities Foot velocities
   * @param[in] grfs Ground reaction forces in the world frame
   * @param[in] terrain Map of the terrain for collision checking
   * @return Boolean for state validity
   */
  bool isValidCentroidalState(
      const Eigen::VectorXd &body_state, const Eigen::VectorXd &foot_positions,
      const Eigen::VectorXd &foot_velocities, const Eigen::VectorXd &grfs,
      const grid_map::GridMap &terrain, Eigen::VectorXd &joint_positions,
      Eigen::VectorXd &joint_velocities, Eigen::VectorXd &torques,
      Eigen::VectorXd &state_violation, Eigen::VectorXd &control_violation);

  inline double getGroundClearance(const Eigen::Vector3d &point,
                                   const grid_map::GridMap &terrain) {
    grid_map::Position pos = {point.x(), point.y()};
    return (point.z() - terrain.atPosition("z", pos));
  }

 private:
  /// Number of feet
  const int num_feet_ = 4;

  /// Vector of the abad link lengths
  std::vector<double> l0_vec_;

  /// Upper link length
  double l1_;

  /// Lower link length
  double l2_;

  /// Abad offset from legbase
  Eigen::Vector3d abad_offset_;

  /// Knee offset from hip
  Eigen::Vector3d knee_offset_;

  /// Foot offset from knee
  Eigen::Vector3d foot_offset_;

  /// Vector of legbase offsets
  std::vector<Eigen::Vector3d> legbase_offsets_;

  /// Vector of legbase offsets
  std::vector<Eigen::Matrix4d> g_body_legbases_;

  /// Epsilon offset for joint bounds
  const double joint_eps = 0.1;

  /// Vector of the joint lower limits
  const std::vector<double> joint_min_ = {-1 + joint_eps,
                                          -0.5 * M_PI + joint_eps, joint_eps};

  /// Vector of the joint upper limits
  const std::vector<double> joint_max_ = {1 - joint_eps, M_PI - joint_eps,
                                          M_PI - joint_eps};

  RigidBodyDynamics::Model *model_;

  std::vector<std::string> body_name_list_;

  std::vector<unsigned int> body_id_list_;

  std::vector<int> leg_idx_list_;

  /// Abad max joint torque
  const double abad_tau_max_ = 21;

  /// Hip max joint torque
  const double hip_tau_max_ = 21;

  /// Knee max joint torque
  const double knee_tau_max_ = 32;

  /// Vector of max torques
  const Eigen::VectorXd tau_max_ =
      (Eigen::VectorXd(12) << abad_tau_max_, hip_tau_max_, knee_tau_max_,
       abad_tau_max_, hip_tau_max_, knee_tau_max_, abad_tau_max_, hip_tau_max_,
       knee_tau_max_, abad_tau_max_, hip_tau_max_, knee_tau_max_)
          .finished();

  /// Abad max joint velocity
  const double abad_vel_max_ = 37.7;

  /// Hip max joint velocity
  const double hip_vel_max_ = 37.7;

  /// Knee max joint velocity
  const double knee_vel_max_ = 25.1;

  /// Vector of max velocities
  const Eigen::VectorXd vel_max_ =
      (Eigen::VectorXd(12) << abad_vel_max_, hip_vel_max_, knee_vel_max_,
       abad_vel_max_, hip_vel_max_, knee_vel_max_, abad_vel_max_, hip_vel_max_,
       knee_vel_max_, abad_vel_max_, hip_vel_max_, knee_vel_max_)
          .finished();

  const Eigen::VectorXd mm_slope_ = tau_max_.cwiseQuotient(vel_max_);
};

}  // namespace quad_utils

#endif  // QUAD_KD_H
