#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <limits>
#include <chrono>
#include <random>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <spirit_utils/fast_terrain_map.h>

namespace planning_utils {

// Define kinematic constraint parameters
const double H_MAX = 0.6;           // Maximum height of leg base, m
const double H_MIN = 0.02;          // Minimum ground clearance of body corners, m
const double V_MAX = 4.0;           // Maximum robot velocity, m/s (4.0 for cheetah, 2.5 for anymal)
const double V_NOM = 1.5;           // Nominal velocity, m/s (used during connect function)
const double P_MAX = 1.0;           // Maximum pitch, rad
const double DP_MAX = 5.0;          // Maximum angular velocity in pitch, rad/s
const double ANG_ACC_MAX = 10.0;    // Maximum angular acceleration in pitch, rad/s^2
const double ROBOT_L = 0.6;         // Length of robot body, m (0.6 cheetah, 0.554 ANYmal)
const double ROBOT_W = 0.256;       // Width of robot body, m (0.256 cheetah, 0.232 ANYmal)
const double ROBOT_H = 0.1;         // Vertical distance between leg base and bottom of robot, m (0.1 cheetah, 0.04 ANYmal)

// Define dynamic constraint parameters
const double M_CONST = 43;          // Robot mass, kg (43 for cheetah, 30 for anymal)
const double G_CONST = 9.81;        // Gravity constant, m/s^2
const double F_MAX = 800;           // Maximum GRF, N (800 for cheetah, 500 for anymal)
const double MU = 1.0;              // Friction coefficient (1.0 for Cheetah, 0.5 for ANYmal)
const double T_S_MIN = 0.3;         // Minimum stance time, s
const double T_S_MAX = 0.3;         // Maximum stance time, s
const double T_F_MIN = 0.0;         // Minimum flight time, s
const double T_F_MAX = 0.5;         // Maximum stance time, s

// Define planning parameters
const double KINEMATICS_RES = 0.05; // Resolution of kinematic feasibility checks, s
const double BACKUP_TIME = 0.2;     // Duration of backup after finding an invalid state, s
const double BACKUP_RATIO = 0.5;    // Ratio of trajectory to back up after finding an invalid state, s
const int NUM_GEN_STATES = 6;       // Number of actions computed for each extend function
const double GOAL_BOUNDS = 0.5;     // Distance threshold on reaching the goal (only used for vanilla RRT, not RRT-Connect)

// Define phase variable labels
const int FLIGHT = 0;
const int STANCE = 1;
const int CONNECT_STANCE = 2;
const int FORWARD = 0;
const int REVERSE = 1;

// Define the dimensionality and types for states, actions, and pairs
const int POSEDIM = 4;
const int STATEDIM = 8;
const int ACTIONDIM = 10;
typedef std::array<double, STATEDIM> State;
typedef std::array<double, ACTIONDIM> Action;
typedef std::pair<State, Action> StateActionPair;

// Define math parameters
const double INFTY = std::numeric_limits<double>::max();
const double MY_PI = 3.14159;

// Define some useful print statements
void vectorToArray(State vec, double * new_array);
void stdVectorToState(std::vector<double> v, State& s);
void printState(State vec);
void printVectorInt(std::vector<int> vec);
void printStateNewline(State vec);
void printVectorIntNewline(std::vector<int> vec);
void printAction(Action a);
void printActionNewline(Action a);
void printStateSequence(std::vector<State> state_sequence);
void printInterpStateSequence(std::vector<State> state_sequence, std::vector<double> interp_t);
void printActionSequence(std::vector<Action> action_sequence);

// Define some utility functions
void interpStateActionPair(State s, Action a,double t0,double dt, std::vector<State> &interp_path, std::vector<double> &interp_t, std::vector<int> &interp_phase);
void getInterpPath(std::vector<State> state_sequence, std::vector<Action> action_sequence,double dt, std::vector<State> &interp_path, std::vector<double> &interp_t, std::vector<int> &interp_phase);
State interp(State q1, State q2, double x);
double poseDistance(State q1, State q2);
double stateDistance(State q1, State q2);
bool isWithinBounds(State s1, State s2);
std::array<double,3> rotate_grf(std::array<double,3> surface_norm, std::array<double,3> grf);

// Define planning helper functions
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
}

#endif