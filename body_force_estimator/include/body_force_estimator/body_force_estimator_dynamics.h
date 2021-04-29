#ifndef FORCE_ESTIMATOR_DYNAMICS_H
#define FORCE_ESTIMATOR_DYNAMICS_H

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <ros/ros.h>

namespace force_estimation_dynamics {
void f_M(Eigen::Vector3d q, int RL, Eigen::Matrix3d &F);
void f_beta(Eigen::Vector3d q, Eigen::Vector3d qd, int RL, Eigen::Vector3d &F);
void f_J_MO(Eigen::Vector3d q, int RL, Eigen::Matrix3d &F);

extern double MO_fric[3];
extern double MO_damp[3];
extern double MO_ktau[3];
}

#endif // FORCE_ESTIMATOR_DYNAMICS_H
