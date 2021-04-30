// Header file to calculate hand jacobians for each foot
#ifndef FOOT_JACOBIANS_H
#define FOOT_JACOBIANS_H

// Include Files
#include <cstddef>
#include <cstdlib>
// #include <Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <spirit_utils/ros_utils.h>

namespace spirit_utils {
// Function Declarations
void getFootJacobian(int leg_idx, const Eigen::VectorXd &state, Eigen::MatrixXd& jacobian);

void getFootJacobian0(const Eigen::VectorXd &state, Eigen::MatrixXd& foot_jacobian0);

void getFootJacobian1(const Eigen::VectorXd &state, Eigen::MatrixXd& foot_jacobian1);

void getFootJacobian2(const Eigen::VectorXd &state, Eigen::MatrixXd& foot_jacobian2);

void getFootJacobian3(const Eigen::VectorXd &state, Eigen::MatrixXd& foot_jacobian3);

void getJacobian(const Eigen::VectorXd &state, Eigen::MatrixXd& jacobian);


}
#endif