#ifndef SPIRIT_EIGEN_CONTROLLER_TYPES
#define SPIRIT_EIGEN_CONTROLLER_TYPES
#include <Eigen/Dense>

#define MPC_STEPS 10
#define NUM_STATES 12
#define NUM_CONTROLS 13

typedef Eigen::Matrix<double,NUM_STATES,1> body_state_t;
typedef Eigen::Matrix<double,MPC_STEPS,1> traj_var_t;
typedef Eigen::Matrix<double,NUM_STATES*MPC_STEPS,1> stacked_traj_t;
typedef Eigen::Matrix<double,4,3> leg_pos_t;

#endif