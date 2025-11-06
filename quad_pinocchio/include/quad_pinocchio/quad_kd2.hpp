#ifndef QUAD_PINOCCHIO_H
#define QUAD_PINOCCHIO_H

#include <math.h>
#include <rclcpp/rclcpp.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <Eigen/Geometry>
// #include <chrono>
// #include <random>
// #include <vector>
#include <Eigen/Core>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

// #include "quad_utils/function_timer.hpp"
// #include "quad_utils/math_utils.hpp"

namespace quad_pinocchio {

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
  QuadKD(rclcpp::Node::SharedPtr node);

  /**
   * @brief Constructor for QuadKD Class
   * @param[in] ns Namespace
   * @return Constructed object of type QuadKD
   */
  QuadKD(rclcpp::Node::SharedPtr node, std::string ns);

  /**
   * @brief Initialize model for the class
   * @param[in] ns Namespace
   */
  void initModel(std::string ns);

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
   * @brief Convert an Eigen Eigen::Matrix4d containing a homogenous transform 
   * to a pinocchio pinocchio::SE3 Special Euclidean Group
   * @param[in] g_transform Homogenous transformation matrix
   * @return pinocchio SE3 transform
   */
  pinocchio::SE3 convertAffineToSE3(Eigen::Vector4d g_transform) const;

  /**
   * @brief Convert an pinocchio pinocchio::SE# Special Euclidean Group
   * to a Eigen Eigen::Matrix4d
   * @param[in] se3_transform pinocchio SE3 transform
   * @return Homogenous transformation matrix
   */
  Eigen::Matrix4d convertSE3ToAffine(pinocchio::SE3 se3_transform) const;

  /**
   * @brief Get the lower joint limit of a particular joint
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] joint_index Index for joint (0 = abad, 1 = hip, 2 = knee)
   * @return Requested joint limit
   */
  double getJointLowerLimit(int leg_index, int joint_index) const;

  /**
   * @brief Get the upper joint limit of a particular joint
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] joint_index Index for joint (0 = abad, 1 = hip, 2 = knee)
   * @return Requested joint limit
   */
  double getJointUpperLimit(int leg_index, int joint_index) const;

  /**
   * @brief Get the upper joint limit of a particular joint
   * @param[in] leg_index Quad leg (0 = FL, 1 = BL, 2 = FR, 3 = BR)
   * @param[in] link_index Index for link (0 = abad, 1 = upper, 2 = lower)
   * @return Requested link length
   */
  double getLinkLength(int leg_index, int link_index) const;

  const pinocchio::Model& model() const { return model_; }
  pinocchio::Data& data() { return data_; }

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
  std::vector<std::vector<double>> joint_min_;

  /// Vector of the joint upper limits
  std::vector<std::vector<double>> joint_max_;

  // RigidBodyDynamics::Model *model_;

  pinocchio::Model model_;

  pinocchio::Data data_;

  rclcpp::Node::SharedPtr node_;

  std::vector<std::string> body_name_list_;

  std::vector<unsigned int> body_id_list_;

  std::vector<pinocchio::FrameIndex> toe_fids_;

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

}  // namespace quad_pinocchio

#endif  // QUAD_PINOCCHIO_H
