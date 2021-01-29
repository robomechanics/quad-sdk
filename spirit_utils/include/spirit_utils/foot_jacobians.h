// Header file to calculate hand jacobians for each foot
#ifndef FOOT_JACOBIANS_H
#define FOOT_JACOBIANS_H

// Include Files
#include <cstddef>
#include <cstdlib>
namespace spirit_utils {
// Function Declarations
void calc_foot_jacobian0(const double state[18], const double parameters[6],
  double foot_jacobian0[9]);
void calc_foot_jacobian1(const double state[18], const double parameters[6],
  double foot_jacobian1[9]);
void calc_foot_jacobian2(const double state[18], const double parameters[6],
  double foot_jacobian2[9]);
void calc_foot_jacobian3(const double state[18], const double parameters[6],
  double foot_jacobian3[9]);
}
#endif

