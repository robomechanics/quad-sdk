// Runtime robot-dynamics dispatcher for the body_force_estimator.
//
// Both spirit_body_force_estimator_dynamics.cpp and
// go2_body_force_estimator_dynamics.cpp are compiled into the binary; each
// defines its own copies of the f_M / f_beta / f_J_MO functions and the
// MO_fric / MO_damp / MO_ktau arrays inside a distinct nested namespace
// (spirit_impl and go2_impl respectively). This file exposes those symbols
// as function pointers and array pointers at the outer
// force_estimation_dynamics:: scope, and loadRobot(robot_type) swaps them
// to point at either variant at runtime.
//
// The calling code (body_force_estimator.cpp) can still say `f_M(q, RL, M)`
// and `MO_ktau[j]` unchanged — the function-pointer/array-pointer syntax
// is identical to direct-symbol access at the call site.

#include "body_force_estimator/body_force_estimator_dynamics.hpp"

#include <string>

namespace force_estimation_dynamics {

// Default to Spirit40 (matches the previous behavior when no robot_type
// param is set).
void (*f_M)(Eigen::Vector3d, int, Eigen::Matrix3d&) = &spirit_impl::f_M;
void (*f_beta)(Eigen::Vector3d, Eigen::Vector3d, int, Eigen::Vector3d&) =
    &spirit_impl::f_beta;
void (*f_J_MO)(Eigen::Vector3d, int, Eigen::Matrix3d&) = &spirit_impl::f_J_MO;

double* MO_fric = spirit_impl::MO_fric;
double* MO_damp = spirit_impl::MO_damp;
double* MO_ktau = spirit_impl::MO_ktau;
double* joint_scale  = spirit_impl::joint_scale;
double* joint_offset = spirit_impl::joint_offset;
double* effort_sign  = spirit_impl::effort_sign;

void loadRobot(const std::string& robot_type) {
  if (robot_type == "go2") {
    f_M    = &go2_impl::f_M;
    f_beta = &go2_impl::f_beta;
    f_J_MO = &go2_impl::f_J_MO;
    MO_fric = go2_impl::MO_fric;
    MO_damp = go2_impl::MO_damp;
    MO_ktau = go2_impl::MO_ktau;
    joint_scale  = go2_impl::joint_scale;
    joint_offset = go2_impl::joint_offset;
    effort_sign  = go2_impl::effort_sign;
  } else {
    // Default / "spirit" / anything unrecognized → Spirit40.
    f_M    = &spirit_impl::f_M;
    f_beta = &spirit_impl::f_beta;
    f_J_MO = &spirit_impl::f_J_MO;
    MO_fric = spirit_impl::MO_fric;
    MO_damp = spirit_impl::MO_damp;
    MO_ktau = spirit_impl::MO_ktau;
    joint_scale  = spirit_impl::joint_scale;
    joint_offset = spirit_impl::joint_offset;
    effort_sign  = spirit_impl::effort_sign;
  }
}

}  // namespace force_estimation_dynamics
