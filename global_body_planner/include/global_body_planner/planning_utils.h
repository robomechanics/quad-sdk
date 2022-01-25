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
#include <eigen3/Eigen/Eigen>
#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/math_utils.h>

// Uncomment to add visualization features
// #define VISUALIZE_TREE
// #define VISUALIZE_ALL_CANDIDATE_ACTIONS
// #define PLOT_TRAJECTORIES
#define DEBUG_INVALID_STATE

namespace planning_utils {

struct PlannerConfig {
  FastTerrainMap terrain;

  // Define kinematic constraint parameters
  double H_MAX = 0.375;         // Maximum height of leg base, m
  double H_MIN = 0.125;         // Minimum ground clearance of body corners, m
  double H_NOM = 0.3;           // Nominal ground clearance of body, m
  double V_MAX = 4.0;           // Maximum robot velocity, m/s (4.0 for cheetah, 2.5 for anymal)
  double V_NOM = 0.25;          // Nominal velocity, m/s (used during connect function)
  double DY_MAX = 0;            // Maximum yaw velocity
  double ROBOT_L = 0.4;         // Length of robot body, m (0.6 cheetah, 0.554 ANYmal)
  double ROBOT_W = 0.3;         // Width of robot body, m (0.256 cheetah, 0.232 ANYmal)
  double ROBOT_H = 0.05;        // Vertical distance between leg base and bottom of robot, m (0.1 cheetah, 0.04 ANYmal)

  // Define dynamic constraint parameters
  double M_CONST = 12;          // Robot mass, kg (12 for spirit, 43 for cheetah, 30 for anymal)
  double J_CONST = 1.0;         // Moment of inertia about the robot's y axis (pitch)
  double G_CONST = 9.81;        // Gravity constant, m/s^2
  double F_MIN = 100;           // Minimum GRF
  double F_MAX = 300;           // Maximum GRF, N (800 for cheetah, 500 for anymal)
  double PEAK_GRF_MIN = 3.0;    // Minimum GRF in units of body weight
  double PEAK_GRF_MAX = 3.0;    // Maximum GRF in units of body weight
  double MU = 0.5;              // Friction coefficient (1.0 for Cheetah, 0.5 for ANYmal)
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
  double MAX_TIME = 2.0;        // Maximum planning time allowed
};

// Define phase variable labels
const int CONNECT = 0;
const int LEAP_STANCE = 1;
const int FLIGHT = 2;
const int LAND_STANCE = 3;
const int FORWARD = 0;
const int REVERSE = 1;

// Define exit flags
const int UNSOLVED = 0;
const int VALID = 1;
const int VALID_PARTIAL = 2;
const int INVALID_START_STATE = 3;
const int INVALID_GOAL_STATE = 4;
const int INVALID_START_GOAL_EQUAL = 5;

// Define the dimensionality and types for states, actions, and pairs
const int POSEDIM = 3;
const int STATEDIM = 6;
const int FULLSTATEDIM = 12;
const int ACTIONDIM = 11;
typedef std::array<double, STATEDIM> State;
typedef std::array<double, ACTIONDIM> Action;
typedef std::vector<double> FullState;
typedef Eigen::Vector3d GRF;
typedef std::pair<State, Action> StateActionPair;

struct StateEigen {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

struct FullStateEigen {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d ang;
  Eigen::Vector3d ang_vel;
};

struct ActionEigen {
  Eigen::Vector3d peak_grf_leap;
  Eigen::Vector3d peak_grf_land;
  double t_s_leap;
  double t_f;
  double t_s_land;
  double dz_0;
  double dz_f;
};

struct StateActionResult {
  State s_new;
  Action a_new;
  double t_new = 0;
  double length = 0;
};

// Define math parameters
const double INFTY = std::numeric_limits<double>::max();
const double MY_PI = 3.14159;

// State data structure conversions
State fullStateToState(const FullState &full_state);
FullState stateToFullState(const State &state, double roll, double pitch, double yaw, 
  double roll_rate, double pitch_rate, double yaw_rate);
void flipDirection(State &state);
void flipDirection(Action &action);
void vectorToArray(const State &vec, double * new_array);

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
double poseDistance(const State &q1, const State &q2);
double stateDistance(const State &q1, const State &q2);
double poseDistance(const std::vector<double> &v1, const std::vector<double> &v2);
bool isWithinBounds(const State &s1, const State &s2, const PlannerConfig &planner_config);
std::array<double,3> rotateGRF(const std::array<double,3> &surface_norm,
  const std::array<double,3> &grf);
Eigen::Vector3d rotateGRF(Eigen::Vector3d &surface_norm, const Eigen::Vector3d &grf);

inline double getSpeed(const State &s) {
  return sqrt(pow(s[3], 2) + pow(s[4], 2) + pow(s[5], 2));
}

// Define function for obtaining full state/path information
void addFullStates(const FullState &start_state, std::vector<State> interp_reduced_path, double dt, 
  std::vector<FullState> &interp_full_path, const PlannerConfig &planner_config);
GRF getGRF(const Action &a,double t, const PlannerConfig &planner_config);
Eigen::Vector3d getAcceleration(const State &s,const Action &a, double t, int phase);
double getHeightFromState(const State &s, const PlannerConfig &planner_config);
double getPitchFromState(const State &s, const PlannerConfig &planner_config);
void interpStateActionPair(const State &s, const Action &a,double t0,double dt,
  std::vector<State> &interp_plan, std::vector<GRF> &interp_GRF,
  std::vector<double> &interp_t, std::vector<int> &interp_primitive_id,
  std::vector<double> &interp_length, const PlannerConfig &planner_config);
void getInterpPlan(const FullState &start_state, const std::vector<State> &state_sequence,
  const std::vector<Action> &action_sequence, double dt, double t0,
  std::vector<FullState> &interp_full_plan, std::vector<GRF> &interp_GRF, 
  std::vector<double> &interp_t, std::vector<int> &interp_primitive_id,
  std::vector<double> &interp_length, const PlannerConfig &planner_config);

// Define planning helper functions
State applyStance(const State &s, const Action &a, double t, int phase, const PlannerConfig &planner_config);
State applyStance(const State &s, const Action &a, int phase, const PlannerConfig &planner_config);
State applyFlight(const State &s, double t_f);
State applyAction(const State &s, const Action &a, const PlannerConfig &planner_config);
// State applyActionReverse(State s, Action a, const PlannerConfig &planner_config);
// State applyStanceReverse(State s, Action a, double t, const PlannerConfig &planner_config);
// State applyStanceReverse(State s, Action a, const PlannerConfig &planner_config);
Action getRandomAction(const std::array<double, 3> &surf_norm, const PlannerConfig &planner_config);
Action getRandomLeapAction(const State &s, const std::array<double, 3> &surf_norm,
  const PlannerConfig &planner_config);
bool isValidAction(const Action &a, const PlannerConfig &planner_config);
bool isValidYawRate(const State &s, const Action &a, double t, int phase, const PlannerConfig &planner_config);
bool isValidState(const State &s, const PlannerConfig &planner_config, int phase);
// bool isValidStateActionPair(State s, Action a, const PlannerConfig &planner_config, State &s_new, double& t_new);
bool isValidStateActionPair(const State &s, const Action &a, StateActionResult &result, 
  const PlannerConfig &planner_config);
bool findValidStateActionPair(State &s, Action &a, StateActionResult &result, 
  const PlannerConfig &planner_config);
// bool isValidStateActionPairReverse(State s, Action a, const PlannerConfig &planner_config, State &s_new, double& t_new);
// bool isValidStateActionPairReverse(State s, Action a, StateActionResult &result, 
//   const PlannerConfig &planner_config);

// Define visualization functions
void publishStateActionPair(const State &s, const Action &a, const State &s_goal,
  const PlannerConfig &planner_config, visualization_msgs::MarkerArray &tree_viz_msg,
  ros::Publisher &tree_pub);
}

#endif
