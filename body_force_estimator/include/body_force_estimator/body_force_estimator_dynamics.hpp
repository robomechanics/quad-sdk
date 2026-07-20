#ifndef FORCE_ESTIMATOR_DYNAMICS_H
#define FORCE_ESTIMATOR_DYNAMICS_H

#include <math.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

namespace force_estimation_dynamics {

/**
 * @brief Compute the identified single-leg joint-space mass matrix.
 * @param[in] q Joint positions for one leg (abad, hip, knee), rad
 * @param[in] RL Leg side sign (-1 for left legs, 1 for right legs)
 * @param[out] F 3x3 joint-space mass matrix
 */
void f_M(Eigen::Vector3d q, int RL, Eigen::Matrix3d& F);

/**
 * @brief Compute the identified single-leg bias dynamics.
 * @param[in] q Joint positions for one leg (abad, hip, knee), rad
 * @param[in] qd Joint velocities for one leg, rad/s
 * @param[in] RL Leg side sign (-1 for left legs, 1 for right legs)
 * @param[out] F Coriolis, centrifugal, and gravity bias vector
 */
void f_beta(Eigen::Vector3d q, Eigen::Vector3d qd, int RL, Eigen::Vector3d& F);

/**
 * @brief Compute the momentum-observer toe Jacobian for one leg.
 * @param[in] q Joint positions for one leg (abad, hip, knee), rad
 * @param[in] RL Leg side sign (-1 for left legs, 1 for right legs)
 * @param[out] F 3x3 toe Jacobian used to map torque residuals to toe forces
 */
void f_J_MO(Eigen::Vector3d q, int RL, Eigen::Matrix3d& F);

/// Dry friction compensation terms for the momentum observer.
extern double MO_fric[3];

/// Viscous damping compensation terms for the momentum observer.
extern double MO_damp[3];

/// Motor torque calibration gains for the momentum observer.
extern double MO_ktau[3];
}  // namespace force_estimation_dynamics

#endif  // FORCE_ESTIMATOR_DYNAMICS_H
