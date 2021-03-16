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
#include <spirit_utils/math_utils.h>

namespace planning_utils {

struct PlannerConfig {
  FastTerrainMap terrain;

  // Define kinematic constraint parameters
  double H_MAX = 0.375;           // Maximum height of leg base, m
  double H_MIN = 0.125;          // Minimum ground clearance of body corners, m
  double V_MAX = 4.0;           // Maximum robot velocity, m/s (4.0 for cheetah, 2.5 for anymal)
  double V_NOM = 0.25;           // Nominal velocity, m/s (used during connect function)
  double ROBOT_L = 0.4;         // Length of robot body, m (0.6 cheetah, 0.554 ANYmal)
  double ROBOT_W = 0.3;       // Width of robot body, m (0.256 cheetah, 0.232 ANYmal)
  double ROBOT_H = 0.05;         // Vertical distance between leg base and bottom of robot, m (0.1 cheetah, 0.04 ANYmal)

  // Define dynamic constraint parameters
  double M_CONST = 12;          // Robot mass, kg (12 for spirit, 43 for cheetah, 30 for anymal)
  double J_CONST = 1.0;         // Moment of inertia about the robot's y axis (pitch)
  double G_CONST = 9.81;        // Gravity constant, m/s^2
  double F_MAX = 300;           // Maximum GRF, N (800 for cheetah, 500 for anymal)
  double MU = 1.0;              // Friction coefficient (1.0 for Cheetah, 0.5 for ANYmal)
  double T_S_MIN = 0.3;         // Minimum stance time, s
  double T_S_MAX = 0.3;         // Maximum stance time, s
  double T_F_MIN = 0.0;         // Minimum flight time, s
  double T_F_MAX = 0.5;         // Maximum stance time, s

  // Define planning parameters
  double KINEMATICS_RES = 0.05; // Resolution of kinematic feasibility checks, s
  double BACKUP_TIME = 0.2;     // Duration of backup after finding an invalid state, s
  double BACKUP_RATIO = 0.5;    // Ratio of trajectory to back up after finding an invalid state, s
  int NUM_GEN_STATES = 6;       // Number of actions computed for each extend function
  double GOAL_BOUNDS = 0.5;     // Distance threshold on reaching the goal (only used for vanilla RRT, not RRT-Connect)
};

// // Define kinematic constraint parameters
// const double H_MAX = 0.375;           // Maximum height of leg base, m
// const double H_MIN = 0.125;          // Minimum ground clearance of body corners, m
// const double V_MAX = 4.0;           // Maximum robot velocity, m/s (4.0 for cheetah, 2.5 for anymal)
// const double V_NOM = 0.25;           // Nominal velocity, m/s (used during connect function)
// const double ROBOT_L = 0.4;         // Length of robot body, m (0.6 cheetah, 0.554 ANYmal)
// const double ROBOT_W = 0.3;       // Width of robot body, m (0.256 cheetah, 0.232 ANYmal)
// const double ROBOT_H = 0.05;         // Vertical distance between leg base and bottom of robot, m (0.1 cheetah, 0.04 ANYmal)

// // Define dynamic constraint parameters
// const double M_CONST = 12;          // Robot mass, kg (12 for spirit, 43 for cheetah, 30 for anymal)
// const double G_CONST = 9.81;        // Gravity constant, m/s^2
// const double F_MAX = 300;           // Maximum GRF, N (800 for cheetah, 500 for anymal)
// const double MU = 1.0;              // Friction coefficient (1.0 for Cheetah, 0.5 for ANYmal)
// const double T_S_MIN = 0.3;         // Minimum stance time, s
// const double T_S_MAX = 0.3;         // Maximum stance time, s
// const double T_F_MIN = 0.0;         // Minimum flight time, s
// const double T_F_MAX = 0.5;         // Maximum stance time, s

// // Define planning parameters
// const double KINEMATICS_RES = 0.05; // Resolution of kinematic feasibility checks, s
// const double BACKUP_TIME = 0.2;     // Duration of backup after finding an invalid state, s
// const double BACKUP_RATIO = 0.5;    // Ratio of trajectory to back up after finding an invalid state, s
// const int NUM_GEN_STATES = 6;       // Number of actions computed for each extend function
// const double GOAL_BOUNDS = 0.5;     // Distance threshold on reaching the goal (only used for vanilla RRT, not RRT-Connect)

// Define phase variable labels
const int FLIGHT = 0;
const int STANCE = 1;
const int CONNECT_STANCE = 2;
const int FORWARD = 0;
const int REVERSE = 1;

// Define the dimensionality and types for states, actions, and pairs
const int POSEDIM = 3;
const int STATEDIM = 6;
const int FULLSTATEDIM = 12;
const int ACTIONDIM = 8;
typedef std::array<double, STATEDIM> State;
typedef std::array<double, ACTIONDIM> Action;
typedef std::vector<double> FullState;
typedef Eigen::Vector3d GRF;
typedef std::pair<State, Action> StateActionPair;

// Define math parameters
const double INFTY = std::numeric_limits<double>::max();
const double MY_PI = 3.14159;

// State data structure conversions
State fullStateToState(FullState full_state);
FullState stateToFullState(State state, double roll, double pitch, double yaw, 
  double roll_rate, double pitch_rate, double yaw_rate);
void vectorToArray(State vec, double * new_array);

// Define some useful print statements
void printState(State vec);
void printVector(std::vector<double> vec);
void printVectorInt(std::vector<int> vec);
void printStateNewline(State vec);
void printVectorNewline(std::vector<double> vec);
void printVectorIntNewline(std::vector<int> vec);
void printAction(Action a);
void printActionNewline(Action a);
void printStateSequence(std::vector<State> state_sequence);
void printInterpStateSequence(std::vector<State> state_sequence, std::vector<double> interp_t);
void printActionSequence(std::vector<Action> action_sequence);
void plotYaw(std::vector<double> interp_t, std::vector<FullState> interp_full_path);

// Define some utility functions
double poseDistance(State q1, State q2);
double stateDistance(State q1, State q2);
double poseDistance(std::vector<double> v1, std::vector<double> v2);
bool isWithinBounds(State s1, State s2, const PlannerConfig &planner_config);
std::array<double,3> rotateGRF(std::array<double,3> surface_norm, std::array<double,3> grf);

// Define function for obtaining full state/path information
void addFullStates(FullState start_state, std::vector<State> interp_reduced_path, double dt, 
  std::vector<FullState> &interp_full_path, const PlannerConfig &planner_config);
GRF getGRF(Action a,double t, const PlannerConfig &planner_config);
double getHeightFromState(State s, const PlannerConfig &planner_config);
double getPitchFromState(State s, const PlannerConfig &planner_config);
void interpStateActionPair(State s, Action a,double t0,double dt,
  std::vector<State> &interp_plan, std::vector<GRF> &interp_GRF,
  std::vector<double> &interp_t, std::vector<int> &interp_primitive_id,
  const PlannerConfig &planner_config);
void getInterpPlan(FullState start_state, std::vector<State> state_sequence,
  std::vector<Action> action_sequence,double dt, double t0,
  std::vector<FullState> &interp_full_plan, std::vector<GRF> &interp_GRF, 
  std::vector<double> &interp_t, std::vector<int> &interp_primitive_id,
  const PlannerConfig &planner_config);

// Define planning helper functions
State applyStance(State s, Action a, double t, const PlannerConfig &planner_config);
State applyStance(State s, Action a, const PlannerConfig &planner_config);
State applyFlight(State s, double t_f);
State applyAction(State s, Action a);
State applyStanceReverse(State s, Action a, double t, const PlannerConfig &planner_config);
State applyStanceReverse(State s, Action a, const PlannerConfig &planner_config);
Action getRandomAction(std::array<double, 3> surf_norm, const PlannerConfig &planner_config);
bool isValidAction(Action a, const PlannerConfig &planner_config);
bool isValidState(State s, const PlannerConfig &planner_config, int phase);
bool isValidStateActionPair(State s, Action a, const PlannerConfig &planner_config, State &s_new, double& t_new);
bool isValidStateActionPair(State s, Action a, const PlannerConfig &planner_config);
bool isValidStateActionPairReverse(State s, Action a, const PlannerConfig &planner_config, State &s_new, double& t_new);
bool isValidStateActionPairReverse(State s, Action a, const PlannerConfig &planner_config);
}

#endif