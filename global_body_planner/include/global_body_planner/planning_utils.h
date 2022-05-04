#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <math.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <grid_map_core/grid_map_core.hpp>
#include <iostream>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Uncomment to add visualization features
// #define VISUALIZE_TREE
// #define VISUALIZE_ALL_CANDIDATE_ACTIONS
// #define PLOT_TRAJECTORIES
// #define DEBUG_REFINE_STATE
// #define DEBUG_INVALID_STATE
// #define DEBUG_SOLVE_RESULT

namespace planning_utils {

struct PlannerConfig {
  // Declare the terrain map object
  FastTerrainMap terrain;
  grid_map::GridMap terrain_grid_map;

  // Define kinematic constraint parameters
  double H_MAX;   // Maximum height of leg base, m
  double H_MIN;   // Minimum ground clearance of body corners, m
  double H_NOM;   // Nominal ground clearance of body, m
  double V_MAX;   // Maximum robot velocity, m/s
  double V_NOM;   // Nominal velocity, m/s (used during connect function)
  double DY_MAX;  // Maximum yaw velocity

  // Define dynamic constraint parameters
  double M_CONST;       // Robot mass, kg
  double J_CONST;       // Moment of inertia about the robot's y axis (pitch)
  double G_CONST;       // Gravity constant, m/s^2
  double F_MIN;         // Minimum GRF
  double F_MAX;         // Maximum GRF, N
  double PEAK_GRF_MIN;  // Minimum GRF in units of body weight
  double PEAK_GRF_MAX;  // Maximum GRF in units of body weight
  double MU;            // Friction coefficient
  double T_S_MIN;       // Minimum stance time, s
  double T_S_MAX;       // Maximum stance time, s
  double T_F_MIN;       // Minimum flight time, s
  double T_F_MAX;       // Maximum stance time, s

  // Define planning parameters
  double KINEMATICS_RES;      // Resolution of kinematic feasibility checks, m
  int TRAPPED_BUFFER_FACTOR;  // Number of feasibility that must pass to not
                              // consider a state trapped
  double BACKUP_TIME;    // Duration of backup after finding an invalid state, s
  double BACKUP_RATIO;   // Ratio of trajectory to back up after finding an
                         // invalid state, s
  int NUM_LEAP_SAMPLES;  // Number of actions computed for each extend function
  double GOAL_BOUNDS;    // Distance threshold on reaching the goal (only
                         // used for vanilla RRT, not RRT-Connect)
  double MAX_TIME;       // Maximum planning time allowed
  Eigen::Vector3d G_VEC;  // Maximum planning time allowed

  // Define robot params and declare points used for validity checking
  double ROBOT_L;  // Length of robot body, m
  double ROBOT_W;  // Width of robot body, m
  double ROBOT_H;  // Vertical distance between leg base and bottom of
                   // robot, m

  double body_traversability_threshold;
  double contact_traversability_threshold;
  bool enable_leaping = true;

  static const int num_reachability_points =
      4;  // Number of points on body used to check reachability
  static const int num_collision_points =
      5;  // Number of points on body used to check for collisions

  Eigen::Matrix<double, 3, num_reachability_points>
      reachability_points_body;  // Positions of reachability points in the body
                                 // frame
  Eigen::Matrix<double, 3, num_collision_points>
      collision_points_body;  // Positions of collision points in the body frame

  void loadEigenVectorsFromParams() {
    // Load the gravity vector
    G_VEC << 0, 0, -G_CONST;

    // Load the reachability test points
    reachability_points_body << 0.5 * ROBOT_L, -0.5 * ROBOT_L, 0.5 * ROBOT_L,
        -0.5 * ROBOT_L, 0.5 * ROBOT_W, 0.5 * ROBOT_W, -0.5 * ROBOT_W,
        -0.5 * ROBOT_W, 0, 0, 0, 0;
    // Load the collision test points
    collision_points_body << 0.5 * ROBOT_L, -0.5 * ROBOT_L, 0.5 * ROBOT_L,
        -0.5 * ROBOT_L, 0, 0.5 * ROBOT_W, 0.5 * ROBOT_W, -0.5 * ROBOT_W,
        -0.5 * ROBOT_W, 0, -0.5 * ROBOT_H, -0.5 * ROBOT_H, -0.5 * ROBOT_H,
        -0.5 * ROBOT_H, -0.5 * ROBOT_H;
  }

  void loadParamsFromServer(ros::NodeHandle nh) {
    quad_utils::loadROSParam(nh, "global_body_planner/H_MAX", H_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/H_MIN", H_MIN);
    quad_utils::loadROSParam(nh, "global_body_planner/H_NOM", H_NOM);
    quad_utils::loadROSParam(nh, "global_body_planner/V_MAX", V_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/V_NOM", V_NOM);
    quad_utils::loadROSParam(nh, "global_body_planner/DY_MAX", DY_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/ROBOT_L", ROBOT_L);
    quad_utils::loadROSParam(nh, "global_body_planner/ROBOT_W", ROBOT_W);
    quad_utils::loadROSParam(nh, "global_body_planner/ROBOT_W", ROBOT_W);
    quad_utils::loadROSParam(
        nh, "global_body_planner/BODY_TRAVERSABILITY_THRESHOLD",
        body_traversability_threshold);
    quad_utils::loadROSParam(
        nh, "global_body_planner/CONTACT_TRAVERSABILITY_THRESHOLD",
        contact_traversability_threshold);
    quad_utils::loadROSParam(nh, "global_body_planner/M_CONST", M_CONST);
    quad_utils::loadROSParam(nh, "global_body_planner/J_CONST", J_CONST);
    quad_utils::loadROSParam(nh, "global_body_planner/G_CONST", G_CONST);
    quad_utils::loadROSParam(nh, "global_body_planner/F_MIN", F_MIN);
    quad_utils::loadROSParam(nh, "global_body_planner/F_MAX", F_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/PEAK_GRF_MIN",
                             PEAK_GRF_MIN);
    quad_utils::loadROSParam(nh, "global_body_planner/PEAK_GRF_MAX",
                             PEAK_GRF_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/MU", MU);
    quad_utils::loadROSParam(nh, "global_body_planner/T_S_MIN", T_S_MIN);
    quad_utils::loadROSParam(nh, "global_body_planner/T_S_MAX", T_S_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/T_F_MIN", T_F_MIN);
    quad_utils::loadROSParam(nh, "global_body_planner/T_F_MAX", T_F_MAX);
    quad_utils::loadROSParam(nh, "global_body_planner/KINEMATICS_RES",
                             KINEMATICS_RES);
    quad_utils::loadROSParam(nh, "global_body_planner/BACKUP_TIME",
                             BACKUP_TIME);
    quad_utils::loadROSParam(nh, "global_body_planner/BACKUP_RATIO",
                             BACKUP_RATIO);
    quad_utils::loadROSParam(nh, "global_body_planner/TRAPPED_BUFFER_FACTOR",
                             TRAPPED_BUFFER_FACTOR);
    quad_utils::loadROSParam(nh, "global_body_planner/NUM_LEAP_SAMPLES",
                             NUM_LEAP_SAMPLES);
    quad_utils::loadROSParam(nh, "global_body_planner/GOAL_BOUNDS",
                             GOAL_BOUNDS);
    quad_utils::loadROSParam(nh, "global_body_planner/max_planning_time",
                             MAX_TIME);

    // Load the scalar parameters into Eigen vectors
    loadEigenVectorsFromParams();
  }
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

const grid_map::InterpolationMethods INTER_TYPE =
    grid_map::InterpolationMethods::INTER_NEAREST;

typedef Eigen::Vector3d GRF;

// Define state with Eigen data
struct StateEigen {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;

  bool operator==(const StateEigen rhs) const {
    // Z velocity is overridden by action
    return ((pos == rhs.pos) && (vel.head<2>() == rhs.vel.head<2>()));
  }

  bool operator!=(const StateEigen rhs) const {
    // Z velocity is overridden by action
    return ((pos != rhs.pos) || (vel.head<2>() != rhs.vel.head<2>()));
  }

  bool isApprox(const StateEigen rhs) const {
    // Z velocity is overridden by action
    return ((pos.isApprox(rhs.pos)) &&
            (vel.head<2>().isApprox(rhs.vel.head<2>())));
  }
};

// Define full state with Eigen data
struct FullStateEigen {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d ang;
  Eigen::Vector3d ang_vel;
};

// Define action with Eigen data
struct ActionEigen {
  GRF grf_0;
  GRF grf_f;
  double t_s_leap;
  double t_f;
  double t_s_land;
  double dz_0;
  double dz_f;
};

// Define the dimensionality and types for states, actions, and pairs
const int POSEDIM = 3;
const int STATEDIM = 6;
const int FULLSTATEDIM = 12;
const int ACTIONDIM = 11;
typedef std::array<double, STATEDIM> StateVec;
typedef std::array<double, ACTIONDIM> ActionVec;
typedef std::vector<double> FullStateVec;

typedef StateEigen State;
typedef ActionEigen Action;
typedef FullStateEigen FullState;

typedef std::pair<State, Action> StateActionPair;

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
FullState stateToFullState(const State &state, double roll, double pitch,
                           double yaw, double roll_rate, double pitch_rate,
                           double yaw_rate);
void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s);
Eigen::VectorXd fullStateToEigen(const FullState &s);
void vectorToFullState(const std::vector<double> v, FullState &s);
void flipDirection(State &state);
void flipDirection(Action &action);

// Print statements
void printState(const State &vec);
void printFullState(const FullState &vec);
template <typename T>
void printVector(const std::vector<T> &vec) {
  std::cout << "{";
  for (auto val : vec) std::cout << val << ", ";
  std::cout << "\b\b}";
}
template <typename T>
void printVectorNewline(const std::vector<T> &vec) {
  printVector(vec);
  std::cout << std::endl;
}
void printStateNewline(State vec);
void printAction(Action a);
void printActionNewline(Action a);
void printStateSequence(std::vector<State> state_sequence);
void printInterpStateSequence(std::vector<State> state_sequence,
                              std::vector<double> interp_t);
void printActionSequence(std::vector<Action> action_sequence);
void plotYaw(std::vector<double> interp_t,
             std::vector<FullState> interp_full_path);

// Define some utility functions
double poseDistance(const State &q1, const State &q2);
double poseDistance(const FullState &q1, const FullState &q2);
double stateDistance(const State &q1, const State &q2);
double poseDistance(const std::vector<double> &v1,
                    const std::vector<double> &v2);
bool isWithinBounds(const State &s1, const State &s2,
                    const PlannerConfig &planner_config);
std::array<double, 3> rotateGRF(const std::array<double, 3> &surface_norm,
                                const std::array<double, 3> &grf);
Eigen::Vector3d rotateGRF(const Eigen::Vector3d &surface_norm,
                          const Eigen::Vector3d &grf);

inline double getSpeed(const State &s) { return s.vel.norm(); }

// Define functions for obtaining full state/path information
void addFullStates(const FullState &start_state,
                   std::vector<State> interp_reduced_path, double dt,
                   std::vector<FullState> &interp_full_path,
                   const PlannerConfig &planner_config);
State interpStateActionPair(const State &s, const Action &a, double t0,
                            double dt, std::vector<State> &interp_plan,
                            std::vector<GRF> &interp_GRF,
                            std::vector<double> &interp_t,
                            std::vector<int> &interp_primitive_id,
                            std::vector<double> &interp_length,
                            const PlannerConfig &planner_config);
void getInterpPlan(const FullState &start_state,
                   const std::vector<State> &state_sequence,
                   const std::vector<Action> &action_sequence, double dt,
                   double t0, std::vector<FullState> &interp_full_plan,
                   std::vector<GRF> &interp_GRF, std::vector<double> &interp_t,
                   std::vector<int> &interp_primitive_id,
                   std::vector<double> &interp_length,
                   const PlannerConfig &planner_config);

// Terrain-based heuristics
double getPitchFromState(const State &s, const PlannerConfig &planner_config);
double getDzFromState(const State &s, const PlannerConfig &planner_config);
void setDz(State &s, const PlannerConfig &planner_config);
void setDz(State &s, const Eigen::Vector3d &surf_norm);
inline bool isInMap(const Eigen::Vector3d &pos,
                    const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // return planner_config.terrain_grid_map.isInside(pos.head<2>());
  return planner_config.terrain.isInRange(pos[0], pos[1]);
}
inline bool isInMap(const State &s, const PlannerConfig &planner_config) {
  return isInMap(s.pos, planner_config);
}
inline double getTerrainZ(const Eigen::Vector3d &pos,
                          const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // return planner_config.terrain_grid_map.atPosition("z_inpainted",
  // pos.head<2>(),
  //                                             INTER_TYPE);
  return (planner_config.terrain.getGroundHeight(pos[0], pos[1]));
}
inline double getTerrainZFiltered(const Eigen::Vector3d &pos,
                                  const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // return planner_config.terrain_grid_map.atPosition("z_smooth",
  // pos.head<2>(),
  //                                             INTER_TYPE);
  return (planner_config.terrain.getGroundHeightFiltered(pos[0], pos[1]));
}
inline double getTraversability(const Eigen::Vector3d &pos,
                                const PlannerConfig &planner_config) {
  return planner_config.terrain_grid_map.atPosition("traversability",
                                                    pos.head<2>(), INTER_TYPE);
}
inline bool isBodyTraversable(const Eigen::Vector3d &pos,
                              const PlannerConfig &planner_config) {
  return (getTraversability(pos, planner_config) >=
          planner_config.body_traversability_threshold);
}
inline bool isContactTraversable(const Eigen::Vector3d &pos,
                                 const PlannerConfig &planner_config) {
  return (getTraversability(pos, planner_config) >=
          planner_config.contact_traversability_threshold);
}
inline Eigen::Vector3d getSurfaceNormalFiltered(
    const State &s, const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // Eigen::Vector3d surf_norm;
  // surf_norm.x() = planner_config.terrain_grid_map.atPosition(
  //     "normal_vectors_x", s.pos.head<2>(), INTER_TYPE);
  // surf_norm.y() = planner_config.terrain_grid_map.atPosition(
  //     "normal_vectors_y", s.pos.head<2>(), INTER_TYPE);
  // surf_norm.z() = planner_config.terrain_grid_map.atPosition(
  //     "normal_vectors_z", s.pos.head<2>(), INTER_TYPE);
  // return surf_norm;
  return planner_config.terrain.getSurfaceNormalFilteredEigen(s.pos[0],
                                                              s.pos[1]);
}
inline double getTerrainZFromState(const State &s,
                                   const PlannerConfig &planner_config) {
  return getTerrainZ(s.pos, planner_config);
}
inline double getTerrainZFilteredFromState(
    const State &s, const PlannerConfig &planner_config) {
  return getTerrainZFiltered(s.pos, planner_config);
}
inline double getZRelToTerrain(const Eigen::Vector3d &pos,
                               const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZ(pos, planner_config));
}
inline double getZRelToTerrain(const State &s,
                               const PlannerConfig &planner_config) {
  return getZRelToTerrain(s.pos, planner_config);
}
inline double getZRelToTerrainFiltered(const Eigen::Vector3d &pos,
                                       const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZFiltered(pos, planner_config));
}
inline double getZRelToTerrainFiltered(const State &s,
                                       const PlannerConfig &planner_config) {
  return getZRelToTerrainFiltered(s.pos, planner_config);
}
inline void getMapBounds(const PlannerConfig &planner_config, double &x_min,
                         double &x_max, double &y_min, double &y_max) {
  double eps = 1;
  x_min = planner_config.terrain.getXData().front() + eps;
  x_max = planner_config.terrain.getXData().back() - eps;
  y_min = planner_config.terrain.getYData().front() + eps;
  y_max = planner_config.terrain.getYData().back() - eps;
}

// Kinematics
State applyStance(const State &s, const Action &a, double t, int phase,
                  const PlannerConfig &planner_config);
State applyStance(const State &s, const Action &a, int phase,
                  const PlannerConfig &planner_config);
State applyFlight(const State &s, double t_f,
                  const PlannerConfig &planner_config);
State applyAction(const State &s, const Action &a,
                  const PlannerConfig &planner_config);
GRF getGRF(const Action &a, double t, int phase,
           const PlannerConfig &planner_config);
Eigen::Vector3d getAcceleration(const Action &a, double t, int phase,
                                const PlannerConfig &planner_config);

// Action sampling
Action getRandomAction(const std::array<double, 3> &surf_norm,
                       const PlannerConfig &planner_config);
bool getRandomLeapAction(const State &s, const Eigen::Vector3d &surf_norm,
                         Action &a, const PlannerConfig &planner_config);

// Action refinement (for improved feasiblity)
bool refineAction(const State &s, Action &a,
                  const PlannerConfig &planner_config);
bool refineStance(const State &s, int phase, Action &a,
                  const PlannerConfig &planner_config);
bool refineFlight(const State &s, double &t_f,
                  const PlannerConfig &planner_config);

// Instantaneous validity checking
bool isValidAction(const Action &a, const PlannerConfig &planner_config);
bool isValidYawRate(const State &s, const Action &a, double t, int phase,
                    const PlannerConfig &planner_config);
bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase);
bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase, double &max_height);

// Trajectory validity checking
bool isValidStateActionPair(const State &s, const Action &a,
                            StateActionResult &result,
                            const PlannerConfig &planner_config);

// Define visualization functions
void publishStateActionPair(const State &s, const Action &a,
                            const State &s_goal,
                            const PlannerConfig &planner_config,
                            visualization_msgs::MarkerArray &tree_viz_msg,
                            ros::Publisher &tree_pub);
}  // namespace planning_utils

#endif
