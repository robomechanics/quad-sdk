// Header file to calculate hand jacobians for each foot
#ifndef FOOT_JACOBIANS_H
#define FOOT_JACOBIANS_H

// Include Files
#include <cstddef>
#include <cstdlib>
// #include <Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <spirit_utils/ros_utils.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace spirit_utils {
// Function Declarations
void getFootJacobian(int leg_idx, const Eigen::VectorXd &state, Eigen::MatrixXd& jacobian);

void getJacobian(const Eigen::VectorXd &state, Eigen::MatrixXd& jacobian);


}
#endif