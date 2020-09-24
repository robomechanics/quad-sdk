#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <chrono>
#include <random>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <spirit_utils/fast_terrain_map.h>

// Define constants
#define H_MAX 0.6 // 0.6 for Cheetah, 0.59 for ANYmal
#define H_MIN 0.02 // 0.02
#define V_MAX 4.0 // 4.0 for cheetah, 2.5 for anymal - remember that max height jumped is v^2/(2*g)
#define P_MAX 1.0
#define DP_MAX 5.0
#define ANG_ACC_MAX 10.0 // 10
#define MY_PI 3.14159

#define M_CONST 43 		// 43 for cheetah, 30 for anymal
#define G_CONST 9.81
#define F_MAX 800		// 800 or 1800 for cheetah, 500 for anymal
#define MU 1.0			// 0.9 for Cheetah, 0.5 for ANYmal
#define T_S_MAX 0.3		// 0.5
#define T_S_MIN 0.3		// 0.2
#define T_F_MIN 0.0		// 0.5 // should be 0 or many states get trapped!
#define T_F_MAX 0.5		// 0.5
#define V_NOM 1.5 		// 1.5

#define KINEMATICS_RES 0.05 // period of kinematic feasibility check (in s)
#define BACKUP_TIME 0.2
#define BACKUP_RATIO 0.5

#define NUM_GEN_STATES 6
#define GOAL_BOUNDS 0.5 //0.25
#define STANCE 1
#define FLIGHT 0
#define CONNECT_STANCE 2

#define ROBOT_L 0.6		// 0.6 cheetah, 0.554 ANYmal
#define ROBOT_W 0.256	// 0.256 cheetah, 0.232 ANYmal
#define ROBOT_H 0.2		// 0.2 cheetah, 0.08 ANYmal (actual body height is 24cm but this is located 8cm above the motors)
							// so the distance between the motors and the bottom is 24/2-8 = 4 cm, see
							// https://github.com/ANYbotics/anymal_b_simple_description/blob/master/urdf/anymal.urdf

#define INFTY 99999

#define POSEDIM 4
#define STATEDIM 8
#define ACTIONDIM 10
typedef std::array<double, STATEDIM> State;
typedef std::array<double, ACTIONDIM> Action;
typedef std::pair<State, Action> StateActionPair;

#define FORWARD 0
#define REVERSE 1

// typedef struct
// {
// 	std::vector<double> x_data;
// 	std::vector<double> y_data;
// 	std::vector<std::vector<double>> z_data;
// 	std::vector<std::vector<double>> dx_data;
// 	std::vector<std::vector<double>> dy_data;
// 	std::vector<std::vector<double>> dz_data;
// 	int x_size;
// 	int y_size;
// 	int z_size;
// } Ground;
// typedef grid_map::GridMap Ground;

void vectorToArray(State vec, double * new_array);
void printState(State vec);
void printVectorInt(std::vector<int> vec);
void printStateNewline(State vec);
void printVectorIntNewline(std::vector<int> vec);
void printAction(Action a);
void printActionNewline(Action a);

void printStateSequence(std::vector<State> state_sequence);
void printInterpStateSequence(std::vector<State> state_sequence, std::vector<double> interp_t);
void printActionSequence(std::vector<Action> action_sequence);
void interpStateActionPair(State s, Action a,double t0,double dt, std::vector<State> &interp_path, std::vector<double> &interp_t, std::vector<int> &interp_phase);
void getInterpPath(std::vector<State> state_sequence, std::vector<Action> action_sequence,double dt, std::vector<State> &interp_path, std::vector<double> &interp_t, std::vector<int> &interp_phase);


State interp(State q1, State q2, double x);

double poseDistance(State q1, State q2);
double stateDistance(State q1, State q2);
bool isWithinBounds(State s1, State s2);

std::array<double,3> rotate_grf(std::array<double,3> surface_norm, std::array<double,3> grf);
double getGroundHeight(double x, double y);
std::array<double, 3> getSurfaceNormal(double x, double y);

State applyStance(State s, Action a, double t);
State applyStance(State s, Action a);
State applyFlight(State s, double t_f);
State applyAction(State s, Action a);
State applyStanceReverse(State s, Action a, double t);
State applyStanceReverse(State s, Action a);
Action getRandomAction(std::array<double, 3> surf_norm);
bool isValidAction(Action a);
bool isValidState(State s, FastTerrainMap& terrain, int phase);
bool isValidStateActionPair(State s, Action a, FastTerrainMap& terrain, State &s_new, double& t_new);
bool isValidStateActionPair(State s, Action a, FastTerrainMap& terrain);
bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap& terrain, State &s_new, double& t_new);
bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap& terrain);

std::array<double, 4> getAcceleration(State s, Action a, double t);
bool isValidYawRate(State s, Action a, double t);

double arcLengthFunction(State s, Action a, double t, int phase);
double computeArcLength(State s, Action a);

#endif