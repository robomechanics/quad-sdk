#ifndef FORCE_ESTIMATOR_DYNAMICS_H
#define FORCE_ESTIMATOR_DYNAMICS_H

#include <math.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

// Runtime robot-dynamics dispatch.
//
// Each robot's dynamics implementation lives in its own nested namespace:
//   force_estimation_dynamics::spirit_impl   (src/spirit_body_force_estimator_dynamics.cpp)
//   force_estimation_dynamics::go2_impl      (src/go2_body_force_estimator_dynamics.cpp)
// Both nested namespaces define identical symbols (f_M, f_beta, f_J_MO,
// MO_fric, MO_damp, MO_ktau) and both are compiled into the binary.
//
// At the outer force_estimation_dynamics:: scope, f_M/f_beta/f_J_MO are
// function pointers and MO_* are double pointers — call
// force_estimation_dynamics::loadRobot("go2" | "spirit") at startup
// (typically from a ROS parameter) to swap them to point at the correct
// implementation. The call syntax at the use site (`f_M(q, RL, M)`,
// `MO_ktau[j]`) is identical for pointers vs. direct symbols.
#include <string>
namespace force_estimation_dynamics {

namespace spirit_impl {
void f_M(Eigen::Vector3d q, int RL, Eigen::Matrix3d& F);
void f_beta(Eigen::Vector3d q, Eigen::Vector3d qd, int RL, Eigen::Vector3d& F);
void f_J_MO(Eigen::Vector3d q, int RL, Eigen::Matrix3d& F);
extern double MO_fric[3];
extern double MO_damp[3];
extern double MO_ktau[3];
// Per-joint wire→URDF conversion for the frame the mex above expects:
//   q_urdf  = joint_scale[j] * q_wire  + joint_offset[j]
//   qd_urdf = joint_scale[j] * qd_wire
//   tau_urdf = joint_scale[j] * tau_wire            (virtual-work, scale=±1)
// Spirit40 mex was generated with the hip axis flipped in the URDF and no
// offsets, matching the previous hardcoded {1,-1,1} preprocessing.
extern double joint_scale[3];
extern double joint_offset[3];
// Extra per-joint sign correction applied ONLY to the effort feedback read
// from state.joints.effort. Compensates for the Gazebo estimator_plugin
// (quad_simulator/gazebo_plugins/src/estimator_plugin.cpp:361) hardcoding
// `torque = -torque_msg.y()` for the hip channel — a Spirit-URDF-specific
// negation that silently inverts hip torque when the loaded robot's hip
// URDF axis is +y instead of -y. For Spirit this evaluates to identity.
extern double effort_sign[3];
}  // namespace spirit_impl

namespace go2_impl {
void f_M(Eigen::Vector3d q, int RL, Eigen::Matrix3d& F);
void f_beta(Eigen::Vector3d q, Eigen::Vector3d qd, int RL, Eigen::Vector3d& F);
void f_J_MO(Eigen::Vector3d q, int RL, Eigen::Matrix3d& F);
extern double MO_fric[3];
extern double MO_damp[3];
extern double MO_ktau[3];
// Go2 mex was generated from importrobot(go2.urdf) so it expects URDF-frame
// q. Quad-SDK publishes wire-frame joint state (hip negated + pi/2 offset,
// knee -pi offset — see quad_utils/config/go2.yaml).
extern double joint_scale[3];
extern double joint_offset[3];
// See spirit_impl::effort_sign comment. Go2's URDF hip axis is +y (not
// Spirit's -y), so the plugin's hardcoded -torque_msg.y() publishes the
// hip effort with the WRONG sign for Go2. Undo the flip here at the
// consumer boundary rather than touching the shared plugin.
extern double effort_sign[3];
}  // namespace go2_impl

// Runtime-selected pointers. Set via loadRobot(); default to Spirit40.
extern void (*f_M)(Eigen::Vector3d, int, Eigen::Matrix3d&);
extern void (*f_beta)(Eigen::Vector3d, Eigen::Vector3d, int, Eigen::Vector3d&);
extern void (*f_J_MO)(Eigen::Vector3d, int, Eigen::Matrix3d&);
extern double* MO_fric;
extern double* MO_damp;
extern double* MO_ktau;
extern double* joint_scale;
extern double* joint_offset;
extern double* effort_sign;

// Call once at startup with the robot_type ROS parameter value.
// Recognized values: "spirit" (default fallback), "go2".
void loadRobot(const std::string& robot_type);

}  // namespace force_estimation_dynamics

#endif  // FORCE_ESTIMATOR_DYNAMICS_H
