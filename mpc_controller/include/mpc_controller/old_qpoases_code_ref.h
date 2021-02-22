#ifndef LEGGED_CONTROLS_MPC_H
#define LEGGED_CONTROLS_MPC_H

#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <vector>
#include <map>
#include <chrono>
#include <iostream>
#include <numeric>
#include <math.h>
#include <assert.h>

#include "eigen_controller_types.h"
#include "trajectory.h"

class LeggedControlsMPC{
public:
	LeggedControlsMPC(bool debug_timing_info = false);

	void updateTrajectory(body_state_t &x0,
						  double xvel_f,
						  double yvel_f,
						  double psivel_f,
						  double zpos_f);

	void setupQP(body_state_t &x0, leg_pos_t &r, double sim_time);

	void solveQP();

	void eigen2qpoases(Eigen::MatrixXd &mat, qpOASES::real_t qp_mat[] );

	Eigen::Matrix3d getSkewSymMat(Eigen::Vector3d &vec);

private:
	void setupGaitSequence();

	void getContinuousDynamics(double psi, leg_pos_t &r);

	void getDiscreteDynamics();

	void getMPCStackedDynamicsDense();

	void getMPCForceConstraintMatrix();

	void getMPCForceConstraintBounds(double t);

	Eigen::Matrix<double,NUM_STATES,MPC_STEPS> getFutureStates();

	Eigen::Matrix<double,NUM_CONTROLS,1> getFirstControl();

	Eigen::Matrix3d getXRotationMatrix(double ang);

	Eigen::Matrix3d getYRotationMatrix(double ang);

	Eigen::Matrix3d getZRotationMatrix(double ang);

	traj_var_t integrateState(traj_var_t &der, double initial_val);

	// qpOASES solver
	qpOASES::int_t _nWSR;
	qpOASES::SQProblem _solver;
	bool _solver_initialized;
	qpOASES::real_t _H[MPC_STEPS*NUM_CONTROLS*MPC_STEPS*NUM_CONTROLS];
	qpOASES::real_t _g[MPC_STEPS*NUM_CONTROLS];
	qpOASES::real_t _uOpt[NUM_CONTROLS*MPC_STEPS];
	qpOASES::real_t _A[2*NUM_CONTROLS*MPC_STEPS*NUM_CONTROLS*MPC_STEPS];
	qpOASES::real_t _ubA[2*NUM_CONTROLS*MPC_STEPS];

	// Force constraints for each possible stance
	Eigen::MatrixXd _stance_sequence;
	std::vector<double> _stance_timings;
	Eigen::MatrixXd _cmax;
	Eigen::MatrixXd _C;

	// Preallocate mpc cost matrices
	Eigen::Matrix<double,NUM_STATES,1> _state_weights;
	Eigen::Matrix<double,NUM_CONTROLS,1> _control_weights;
	Eigen::MatrixXd _L, _K;

	// Preallocate dynamics matrices
	Eigen::Matrix<double,NUM_STATES,NUM_STATES> _Ac, _Ad;
	Eigen::Matrix<double,NUM_STATES,NUM_CONTROLS> _Bc, _Bd;
	Eigen::MatrixXd _Aqp, _Bqp;

	// Dynamics parameters
	double _m; // Robot body mass in kg
	double _dt; // timestep in seconds
	double _mu; // Friction param

	double _min_force;
	double _max_force;

	Eigen::Vector3d _grav; // acceleration from gravity in m/s (x y z)
	Eigen::Matrix3d _bI;

	// Reference Trajectory
	body_state_t _x0;
	Trajectory _traj;

	// Debugging
	bool _debug_timing_info;
};




#endif