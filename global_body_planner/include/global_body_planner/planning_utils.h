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

// TODO(anyone): Format all of these for Doxygen
struct PlannerConfig {
  // Declare the terrain map object
  FastTerrainMap terrain;
  grid_map::GridMap terrain_grid_map;

  // Define kinematic constraint parameters
  double h_max;  // Maximum height of leg base, m
  double h_min;  // Minimum ground clearance of body corners, m
  double h_nom;  // Nominal ground clearance of body, m
  double v_max;  // Maximum robot velocity, m/s
  double v_nom;  // Nominal velocity, m/s (used during connect function)

  // Define dynamic constraint parameters
  double mass;     // Robot mass, kg
  double g;        // Gravity constant, m/s^2
  double grf_min;  // Minimum GRF in units of body weight
  double grf_max;  // Maximum GRF in units of body weight
  double mu;       // Friction coefficient
  double t_s_min;  // Minimum stance time, s
  double t_s_max;  // Maximum stance time, s
  double dz0_min;  // Minimum vertical velocity impulse, m/s
  double dz0_max;  // Maximum vertical velocity impulse, m/s

  // Define planning parameters
  double dt;                  // Resolution of kinematic feasibility checks, m
  int trapped_buffer_factor;  // Number of feasibility that must pass to not
                              // consider a state trapped
  double backup_ratio;        // Ratio of trajectory to back up after finding an
                              // invalid state, s
  int num_leap_samples;  // Number of actions computed for each extend function
  double max_planning_time;  // Maximum planning time allowed
  Eigen::Vector3d g_vec;     // Maximum planning time allowed

  // Define robot params and declare points used for validity checking
  double robot_l;  // Length of robot body, m
  double robot_w;  // Width of robot body, m
  double robot_h;  // Vertical distance between leg base and bottom of
                   // robot, m

  double body_traversability_threshold;  // Min traversability for body
                                         // (requires the body to not be over a
                                         // hole unless leaping)
  double contact_traversability_threshold;  // Min traversability for contact
                                            // location cannot step on rough
                                            // surfaces unless leaping
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
    g_vec << 0, 0, -g;

    // Load the reachability test points
    reachability_points_body << 0.5 * robot_l, -0.5 * robot_l, 0.5 * robot_l,
        -0.5 * robot_l, 0.5 * robot_w, 0.5 * robot_w, -0.5 * robot_w,
        -0.5 * robot_w, 0, 0, 0, 0;
    // Load the collision test points
    collision_points_body << 0.5 * robot_l, -0.5 * robot_l, 0.5 * robot_l,
        -0.5 * robot_l, 0, 0.5 * robot_w, 0.5 * robot_w, -0.5 * robot_w,
        -0.5 * robot_w, 0, -0.5 * robot_h, -0.5 * robot_h, -0.5 * robot_h,
        -0.5 * robot_h, -0.5 * robot_h;
  }

  void loadParamsFromServer(ros::NodeHandle nh) {
    quad_utils::loadROSParam(nh, "global_body_planner/h_max", h_max);
    quad_utils::loadROSParam(nh, "global_body_planner/h_min", h_min);
    quad_utils::loadROSParam(nh, "global_body_planner/h_nom", h_nom);
    quad_utils::loadROSParam(nh, "global_body_planner/v_max", v_max);
    quad_utils::loadROSParam(nh, "global_body_planner/v_nom", v_nom);
    quad_utils::loadROSParam(nh, "global_body_planner/robot_l", robot_l);
    quad_utils::loadROSParam(nh, "global_body_planner/robot_w", robot_w);
    quad_utils::loadROSParam(nh, "global_body_planner/robot_h", robot_h);
    quad_utils::loadROSParam(
        nh, "global_body_planner/body_traversability_threshold",
        body_traversability_threshold);
    quad_utils::loadROSParam(
        nh, "global_body_planner/contact_traversability_threshold",
        contact_traversability_threshold);
    quad_utils::loadROSParam(nh, "global_body_planner/mass", mass);
    quad_utils::loadROSParam(nh, "global_body_planner/g", g);
    quad_utils::loadROSParam(nh, "global_body_planner/grf_min", grf_min);
    quad_utils::loadROSParam(nh, "global_body_planner/grf_max", grf_max);
    quad_utils::loadROSParam(nh, "global_body_planner/mu", mu);
    quad_utils::loadROSParam(nh, "global_body_planner/t_s_min", t_s_min);
    quad_utils::loadROSParam(nh, "global_body_planner/t_s_max", t_s_max);
    quad_utils::loadROSParam(nh, "global_body_planner/dz0_min", dz0_min);
    quad_utils::loadROSParam(nh, "global_body_planner/dz0_max", dz0_max);
    quad_utils::loadROSParam(nh, "global_body_planner/dt", dt);
    quad_utils::loadROSParam(nh, "global_body_planner/backup_ratio",
                             backup_ratio);
    quad_utils::loadROSParam(nh, "global_body_planner/trapped_buffer_factor",
                             trapped_buffer_factor);
    quad_utils::loadROSParam(nh, "global_body_planner/num_leap_samples",
                             num_leap_samples);
    quad_utils::loadROSParam(nh, "global_body_planner/max_planning_time",
                             max_planning_time);

    // Load the scalar parameters into Eigen vectors
    loadEigenVectorsFromParams();
  }
};

// Define phase variable labels
enum Phase{
  CONNECT,
  LEAP_STANCE,
  FLIGHT,
  LAND_STANCE
};
const int FORWARD = 0;
const int REVERSE = 1;

// Define exit flags
enum ExitFlag{
  UNSOLVED,
  VALID,
  ALID_PARTIAL,
  INVALID_START_STATE,
  INVALID_GOAL_STATE,
  INVALID_START_GOAL_EQUAL
};

const grid_map::InterpolationMethods INTER_TYPE =
    grid_map::InterpolationMethods::INTER_NEAREST;

typedef Eigen::Vector3d GRF;

// Define state with Eigen data
struct State {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;

  bool operator==(const State rhs) const {
    // Z velocity is overridden by action
    return ((pos == rhs.pos) && (vel.head<2>() == rhs.vel.head<2>()));
  }

  bool operator!=(const State rhs) const {
    // Z velocity is overridden by action
    return ((pos != rhs.pos) || (vel.head<2>() != rhs.vel.head<2>()));
  }

  bool isApprox(const State rhs) const {
    // Z velocity is overridden by action
    return ((pos.isApprox(rhs.pos)) &&
            (vel.head<2>().isApprox(rhs.vel.head<2>())));
  }
};

// Define full state with Eigen data
struct FullState {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d ang;
  Eigen::Vector3d ang_vel;
};

// Define action with Eigen data
struct Action {
  GRF grf_0;
  GRF grf_f;
  double t_s_leap; //
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

/**
 * @brief Truncate FullState to State
 * @param[in] full_state FullState
 * @return State
 */
State fullStateToState(const FullState &full_state);

/**
 * @brief Extend State to FullState, adding body orientation and angular speed
 * @param[in] state State
 * @param[in] roll roll
 * @param[in] pitch pitch
 * @param[in] yaw yaw
 * @param[in] roll_rate roll_rate
 * @param[in] pitch pitch_rate
 * @param[in] yaw yaw_rate
 * @return FullState
 */
FullState stateToFullState(const State &state, double roll, double pitch,
                           double yaw, double roll_rate, double pitch_rate,
                           double yaw_rate);

/**
 * @brief Get FullState from Eigen vector
 * @param[in] s_eig FullState in Eigen vector form
 * @param[out] s FullState
 */
void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s);

/**
 * @brief Turn FullState to Eigen vector form
 * @param[in] s FullState
 * @return FullState in Eigen vector form
 */
Eigen::VectorXd fullStateToEigen(const FullState &s);

/**
 * @brief Get FullState from std vector form
 * @param[in] v FullState in std vector form
 * @param[in] s FullState
 */
void vectorToFullState(const std::vector<double> v, FullState &s);

/**
 * @brief Reverse State velocity direction
 */
void flipDirection(State &state);

/**
 * @brief Reverse Action GRF, vertical velocities, 
 * and stance times for landing and leaping if this is an actual leap
 */
void flipDirection(Action &action);

/**
 * @brief Print State
 * @param[in] s State
 */
void printState(const State &s);

/**
 * @brief Print FullState
 * @param[in] s FullState
 */
void printFullState(const FullState &s);

/**
 * @brief Print FullState with a separate line
 * @param[in] s FullState
 */
void printStateNewline(State &s);

/**
 * @brief Print Action
 * @param[in] a Action
 */
void printAction(Action &a);

/**
 * @brief Print Action with a separate line
 * @param[in] a Action
 */
void printActionNewline(Action &a);

/**
 * @brief Print a sequence of State
 * @param[in] state_sequence A sequence of State
 */
void printStateSequence(std::vector<State> &state_sequence);

/**
 * @brief Print a sequence of State
 * @param[in] state_sequence A sequence of State
 * @param[in] state_sequence A sequence of time at which particular 
 * interpolated States occur
 */
void printInterpStateSequence(std::vector<State> &state_sequence,
                              std::vector<double> interp_t);

/**
 * @brief Print a sequence of Action
 * @param[in] state_sequence A sequence of Action
 */
void printActionSequence(std::vector<Action> action_sequence);

/**
 * @brief Calculate the distance between two State positions
 * @param[in] q1 State 1
 * @param[in] q2 State 2
 */
double poseDistance(const State &q1, const State &q2);

/**
 * @brief Calculate the distance between two FullState positions
 * @param[in] q1 FullState 1
 * @param[in] q2 FullState 2
 */
double poseDistance(const FullState &q1, const FullState &q2);

/**
 * @brief Calculate the distance between two positions in std vector form
 * @param[in] q1 Vector 1
 * @param[in] q2 Vector 2
 */
double poseDistance(const std::vector<double> &v1,
                    const std::vector<double> &v2);

/**
 * @brief Calculate the position and veocity distance between two States
 * @param[in] q1 State 1
 * @param[in] q2 State 2
 */
double stateDistance(const State &q1, const State &q2);

/**
 * @brief Rotate GRF from contact frame to spatial frame
 * @param[in] surface_norm Contact point surface normal
 * @param[in] grf Ground reaction force in contact frame
 * @return Ground reaction force in spatial fram
 */
Eigen::Vector3d rotateGRF(const Eigen::Vector3d &surface_norm,
                          const Eigen::Vector3d &grf);

/**
 * @brief Inline function to get State speed
 * @param[in] s State
 * @return State speed
 */
inline double getSpeed(const State &s) { return s.vel.norm(); }

/**
 * @brief Obtain full state/path information
 * @param[in] start_state Initial FullState
 * @param[in] interp_reduced_path A sequence of interpolated State 
 * @param[in] dt  sequence of time at which particular interpolated 
 * States occur
 * @param[in] interp_full_path A sequence of interpolated FullState 
 * @param[in] planner_config planner config
 */
void addFullStates(const FullState &start_state,
                   std::vector<State> interp_reduced_path, double dt,
                   std::vector<FullState> &interp_full_path,
                   const PlannerConfig &planner_config);

/**
 * @brief Interpolate State and Action Pair
 * @param[in] s State
 * @param[in] a Action
 * @param[in] t0 The initial timestep
 * @param[in] dt 
 * @param[in] interp_plan
 * @param[in] inter_GRF
 * @param[in] interp_t
 * @param[in] interp_primitive_id
 * @param[in] interp_length
 * @param[in] planner_config Configuration parameters
 * @return 
 */
State interpStateActionPair(const State &s, const Action &a, double t0,
                            double dt, std::vector<State> &interp_plan,
                            std::vector<GRF> &interp_GRF,
                            std::vector<double> &interp_t,
                            std::vector<int> &interp_primitive_id,
                            std::vector<double> &interp_length,
                            const PlannerConfig &planner_config);

/**
 * @brief Interpolate State and Action Pair
 * @param[in] start_state The start state of the planner
 * @param[in] state_sequence The sequence of states in the path
 * @param[in] action_sequence The sequence of actions in the path
 * @param[in] dt Time interval for interpolation
 * @param[in] t0 The initial timestep
 * @param[in] inter_full_plan Interpolated FullState
 * @param[in] interp_GRF Interpolated ground reaction force
 * @param[in] interp_t Interpolated timestep
 * @param[in] interp_primitive_id
 * @param[in] interp_length The distance of interpolated
 * @param[in] planner_config
 * @return 
 */
void getInterpPlan(const FullState &start_state,
                   const std::vector<State> &state_sequence,
                   const std::vector<Action> &action_sequence, double dt,
                   double t0, std::vector<FullState> &interp_full_plan,
                   std::vector<GRF> &interp_GRF, std::vector<double> &interp_t,
                   std::vector<int> &interp_primitive_id,
                   std::vector<double> &interp_length,
                   const PlannerConfig &planner_config);

/**
 * @brief Obtain body pitch from State
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Pitch of current State
 */
double getPitchFromState(const State &s, const PlannerConfig &planner_config);

/**
 * @brief Align lateral velocity along surface normal
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return delta z
 */
double getDzFromState(const State &s, const PlannerConfig &planner_config);


void setDz(State &s, const PlannerConfig &planner_config);

void setDz(State &s, const Eigen::Vector3d &surf_norm);

/**
 * @brief Inline function to check if State pos is inside 
 * map range or not
 * @param[in] pos State pos in Eigen vector
 * @param[in] planner_config Configuration parameters
 * @return Whether the State pos is inside map
 */
inline bool isInMap(const Eigen::Vector3d &pos,
                    const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  return planner_config.terrain_grid_map.isInside(pos.head<2>());
  // return planner_config.terrain.isInRange(pos[0], pos[1]);
}

/**
 * @brief Inline function to check if State pos is inside map range or not
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Whether the State pos is inside map
 */
inline bool isInMap(const State &s, const PlannerConfig &planner_config) {
  return isInMap(s.pos, planner_config);
}

/**
 * @brief Inline function to get the terrain height at a point
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Terrain height at the location
 */
inline double getTerrainZ(const Eigen::Vector3d &pos,
                          const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // return planner_config.terrain_grid_map.atPosition("z_inpainted",
  // pos.head<2>(),
  //                                             INTER_TYPE);
  return (planner_config.terrain.getGroundHeight(pos[0], pos[1]));
}

/**
 * @brief Inline function to get the filtered terrain height at a point
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Filtered terrain height at the location
 */
inline double getTerrainZFiltered(const Eigen::Vector3d &pos,
                                  const PlannerConfig &planner_config) {
  // Uncomment to use grid_map
  // return planner_config.terrain_grid_map.atPosition("z_smooth",
  // pos.head<2>(),
  //                                             INTER_TYPE);
  return (planner_config.terrain.getGroundHeightFiltered(pos[0], pos[1]));
}

/**
 * @brief Inline function to get the traversability at a point
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Filtered terrain height at the location
 */
inline double getTraversability(const Eigen::Vector3d &pos,
                                const PlannerConfig &planner_config) {
  return planner_config.terrain_grid_map.atPosition("traversability",
                                                    pos.head<2>(), INTER_TYPE);
}

/**
 * @brief Inline function to check whether body is traversable above a threshold
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Whether body is traversable
 */
inline bool isBodyTraversable(const Eigen::Vector3d &pos,
                              const PlannerConfig &planner_config) {
  return (getTraversability(pos, planner_config) >=
          planner_config.body_traversability_threshold);
}

/**
 * @brief Inline function to check whether contact is traversable above a threshold
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Whether body is traversable
 */
inline bool isContactTraversable(const Eigen::Vector3d &pos,
                                 const PlannerConfig &planner_config) {
  return (getTraversability(pos, planner_config) >=
          planner_config.contact_traversability_threshold);
}

/**
 * @brief Inline function to obtain the surface normal filter
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Whether body is traversable
 */
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

/**
 * @brief Inline function to get the terrain height of State pos
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Terrain height at the State pos
 */
inline double getTerrainZFromState(const State &s,
                                   const PlannerConfig &planner_config) {
  return getTerrainZ(s.pos, planner_config);
}

/**
 * @brief Inline function to get the filtered terrain height at a point
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Filtered terrain height at the location
 */
inline double getTerrainZFilteredFromState(
    const State &s, const PlannerConfig &planner_config) {
  return getTerrainZFiltered(s.pos, planner_config);
}

/**
 * @brief Inline function to get the relative difference between 
 *  terrain height and body pos
 * @param[in] pos location in Eigen vector to check height
 * @param[in] planner_config Configuration parameters
 * @return The relative difference between terrain height and body pos
 */
inline double getZRelToTerrain(const Eigen::Vector3d &pos,
                               const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZ(pos, planner_config));
}

/**
 * @brief Inline function to get the relative difference between 
 * terrain height and body pos
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return The relative difference between terrain height and body pos
 */
inline double getZRelToTerrain(const State &s,
                               const PlannerConfig &planner_config) {
  return getZRelToTerrain(s.pos, planner_config);
}

/**
 * @brief Inline function to get the relative difference between 
 * filtered terrain height and body pos
 * @param[in] pos location to check height
 * @param[in] planner_config Configuration parameters
 * @return Filtered terrain height at the location
 */
inline double getZRelToTerrainFiltered(const Eigen::Vector3d &pos,
                                       const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZFiltered(pos, planner_config));
}

/**
 * @brief Inline function to get the relative difference between 
 * filtered terrain height and body pos
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Filtered terrain height at the location
 */
inline double getZRelToTerrainFiltered(const State &s,
                                       const PlannerConfig &planner_config) {
  return getZRelToTerrainFiltered(s.pos, planner_config);
}

/**
 * @brief Inline function to get the map boundary
 * filtered terrain height and body pos
 * @param[in] planner_config Configuration parameters
 * @param[in] x_min The minimal x bounday
 * @param[in] x_max The maximal x bounday
 * @param[in] y_min The minimal y bounday
 * @param[in] y_max The maximal x bounday
 */
inline void getMapBounds(const PlannerConfig &planner_config, double &x_min,
                         double &x_max, double &y_min, double &y_max) {
  double eps = 0.5;
  x_min = planner_config.terrain.getXData().front() + eps;
  x_max = planner_config.terrain.getXData().back() - eps;
  y_min = planner_config.terrain.getYData().front() + eps;
  y_max = planner_config.terrain.getYData().back() - eps;
}

/**
 * @brief Obtain new State height after applying the ground reaction force
 * in Action
 * @param[in] s State
 * @param[in] a Action
 * @param[in] t 
 * @param[in] phase Phase variable, CONNECT or others？
 * @param[in] planner_config Configuration parameters
 */ 
State applyStance(const State &s, const Action &a, double t, int phase,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State height after applying the ground reaction force
 * in Action
 * @param[in] s State
 * @param[in] a Action
 * @param[in] phase Phase variable, CONNECT or others？
 * @param[in] planner_config Configuration parameters
 */ 
State applyStance(const State &s, const Action &a, int phase,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State height after applying new flight
 * @param[in] s State
 * @param[in] t_f
 * @param[in] planner_config Configuration parameters
 * @return New State after Flight phase
 */
State applyFlight(const State &s, double t_f,
                  const PlannerConfig &planner_config);


State applyAction(const State &s, const Action &a,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State height after applying new flight
 * @param[in] s State
 * @param[in] t_f
 * @param[in] planner_config Configuration parameters
 * @return New State after Flight phase
 */
GRF getGRF(const Action &a, double t, int phase,
           const PlannerConfig &planner_config);

Eigen::Vector3d getAcceleration(const Action &a, double t, int phase,
                                const PlannerConfig &planner_config);

// Action sampling
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
