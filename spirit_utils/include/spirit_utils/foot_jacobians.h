// Header file to calculate hand jacobians for each foot
#ifndef FOOT_JACOBIANS_H
#define FOOT_JACOBIANS_H

// Include Files
#include <cstddef>
#include <cstdlib>
// #include <Eigen/Core>
#include <Eigen/Dense>

namespace spirit_utils {
// Function Declarations
void calc_foot_jacobian0(const double state[18], Eigen::MatrixXf& foot_jacobian0);

void calc_foot_jacobian1(const double state[18], Eigen::MatrixXf& foot_jacobian1);

void calc_foot_jacobian2(const double state[18], Eigen::MatrixXf& foot_jacobian2);

void calc_foot_jacobian3(const double state[18], Eigen::MatrixXf& foot_jacobian3);

void calc_jacobian(const double state[18], Eigen::MatrixXf& jacobian);

}
#endif