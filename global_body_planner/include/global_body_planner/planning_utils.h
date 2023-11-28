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
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
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
<<<<<<< HEAD
=======
=======
#include <ros/ros.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/math_utils.h>
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

namespace planning_utils {

/**
 * @brief Planner Configuration
 */
struct PlannerConfig {
  // Declare the terrain map object
  FastTerrainMap terrain;              // Terrain in FastTerrainMap format
  grid_map::GridMap terrain_grid_map;  // Terrain in grid_map format

  // Define kinematic constraint parameters
  double h_max;  // Maximum height of leg base, m
  double h_min;  // Minimum ground clearance of body corners, m
  double h_nom;  // Nominal ground clearance of body, m
  double v_max;  // Maximum robot velocity, m/s
  double v_nom;  // Nominal velocity, m/s (used during connect function)

  // Define dynamic constraint parameters
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  double mass;     // Robot mass, kg
  double g;        // Gravity constant, m/s^2
  double grf_min;  // Minimum GRF in units of body weight
  double grf_max;  // Maximum GRF in units of body weight
  double mu;       // Friction coefficient
  double t_s_min;  // Minimum stance time, s
  double t_s_max;  //  Maximum stance time, s
  double dz0_min;  //  Minimum vertical velocity impulse, m/s
  double dz0_max;  //  Maximum vertical velocity impulse, m/s
<<<<<<< HEAD
=======
=======
  double M_CONST = 12;          // Robot mass, kg (12 for quad, 43 for cheetah, 30 for anymal)
  double J_CONST = 1.0;         // Moment of inertia about the robot's y axis (pitch)
  double G_CONST = 9.81;        // Gravity constant, m/s^2
  double F_MAX = 300;           // Maximum GRF, N (800 for cheetah, 500 for anymal)
  double MU = 1.0;              // Friction coefficient (1.0 for Cheetah, 0.5 for ANYmal)
  double T_S_MIN = 0.3;         // Minimum stance time, s
  double T_S_MAX = 0.3;         // Maximum stance time, s
  double T_F_MIN = 0.0;         // Minimum flight time, s
  double T_F_MAX = 0.5;         // Maximum stance time, s
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

  // Define planning parameters
  double dt;                  //  Resolution of kinematic feasibility checks, m
  int trapped_buffer_factor;  //  Number of feasibility that must pass to not a
                              //  state trapped
  double backup_ratio;   //  Ratio of trajectory to back up after finding an
                         //  invalid state, s
  int num_leap_samples;  //  Number of actions computed for each extend function
  double max_planning_time;  //  Maximum planning time allowed
  Eigen::Vector3d g_vec;     //  Maximum planning time allowed

  // Define robot params and declare points used for validity checking
  double robot_l;  //  Length of robot body, m
  double robot_w;  //  Width of robot body, m
  double robot_h;  //  Vertical distance between leg base and bottom of robot, m

  double traversability_threshold;  // Min traversability for contact location
                                    // unless in flight

  bool enable_leaping = true;  // Leaping mode switch
  static const int num_reachability_points =
      4;  // Number of points on body used to check reachability
  static const int num_collision_points =
      5;  // Number of points on body used to check for collisions

  Eigen::Matrix<double, 3, num_reachability_points>
      reachability_points_body;  // Positions of reachability points in thebody
                                 // frame
  Eigen::Matrix<double, 3, num_collision_points>
      collision_points_body;  // Positions of collision points in the bodyframe

  /**
   * Load the vector of reachability test points and collision test
   * points in robot frame
   */
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

  /**
   * @brief Load the Global Body Planner parameters from ROS server
   */
  void loadParamsFromServer(ros::NodeHandle nh) {
    // Load robot parameters
    quad_utils::loadROSParam(nh, "global_body_planner/h_max", h_max);
    quad_utils::loadROSParam(nh, "global_body_planner/h_min", h_min);
    quad_utils::loadROSParam(nh, "global_body_planner/h_nom", h_nom);
    quad_utils::loadROSParam(nh, "global_body_planner/v_max", v_max);
    quad_utils::loadROSParam(nh, "global_body_planner/v_nom", v_nom);
    quad_utils::loadROSParam(nh, "global_body_planner/robot_l", robot_l);
    quad_utils::loadROSParam(nh, "global_body_planner/robot_w", robot_w);
    quad_utils::loadROSParam(nh, "global_body_planner/robot_h", robot_h);

    quad_utils::loadROSParam(nh, "global_body_planner/mass", mass);
    quad_utils::loadROSParam(nh, "global_body_planner/grf_min", grf_min);
    quad_utils::loadROSParam(nh, "global_body_planner/grf_max", grf_max);
    quad_utils::loadROSParam(nh,
                             "/global_body_planner/traversability_threshold",
                             traversability_threshold);

    // Load global parameters
    quad_utils::loadROSParam(nh, "/global_body_planner/g", g);
    quad_utils::loadROSParam(nh, "/global_body_planner/mu", mu);
    quad_utils::loadROSParam(nh, "/global_body_planner/t_s_min", t_s_min);
    quad_utils::loadROSParam(nh, "/global_body_planner/t_s_max", t_s_max);
    quad_utils::loadROSParam(nh, "/global_body_planner/dz0_min", dz0_min);
    quad_utils::loadROSParam(nh, "/global_body_planner/dz0_max", dz0_max);
    quad_utils::loadROSParam(nh, "/global_body_planner/dt", dt);
    quad_utils::loadROSParam(nh, "/global_body_planner/backup_ratio",
                             backup_ratio);
    quad_utils::loadROSParam(nh, "/global_body_planner/trapped_buffer_factor",
                             trapped_buffer_factor);
    quad_utils::loadROSParam(nh, "/global_body_planner/num_leap_samples",
                             num_leap_samples);
    quad_utils::loadROSParam(nh, "/global_body_planner/max_planning_time",
                             max_planning_time);

    // Load the scalar parameters into Eigen vectors
    loadEigenVectorsFromParams();
  }
<<<<<<< HEAD
};

/**
 * @brief Define phase labels
 */
enum Phase { CONNECT, LEAP_STANCE, FLIGHT, LAND_STANCE };

/**
 * @brief Define tree growing direction labels
 * (FORWARD to go away from the root vertex, REVERSE to go towards it)
 */
enum TreeDirection { FORWARD, REVERSE };

/**
 * @brief Define exit flags
 */
enum ExitFlag {
  UNSOLVED,
  VALID,
  VALID_PARTIAL,
  INVALID_START_STATE,
  INVALID_GOAL_STATE,
  INVALID_START_GOAL_EQUAL
};

=======
};

/**
 * @brief Define phase labels
 */
enum Phase { CONNECT, LEAP_STANCE, FLIGHT, LAND_STANCE };

/**
 * @brief Define tree growing direction labels
 * (FORWARD to go away from the root vertex, REVERSE to go towards it)
 */
enum TreeDirection { FORWARD, REVERSE };

/**
 * @brief Define exit flags
 */
enum ExitFlag {
  UNSOLVED,
  VALID,
  VALID_PARTIAL,
  INVALID_START_STATE,
  INVALID_GOAL_STATE,
  INVALID_START_GOAL_EQUAL
};

>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
/// Interpolation typ
const grid_map::InterpolationMethods INTER_TYPE =
    grid_map::InterpolationMethods::INTER_NEAREST;

/// Ground reaction force
typedef Eigen::Vector3d GRF;

/**
 * @brief Define state with Eigen data
 */
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

/**
 * @brief Define full state with Eigen data
 */
struct FullState {
  Eigen::Vector3d pos;      // Position
  Eigen::Vector3d vel;      // Velocity
  Eigen::Vector3d ang;      // Linear Velocity
  Eigen::Vector3d ang_vel;  // Angular Velocity
};

/**
 * @brief Define action with Eigen data
 */
struct Action {
  GRF grf_0;        // Ground reaction force at the beginning of leaping phase
  GRF grf_f;        // Ground reaction froce at the end of landing phase
  double t_s_leap;  // Time length of leaping phase
  double t_f;       // Time length of flight phase
  double t_s_land;  // Time length of landing phase
  double dz_0;      // Velocity at the beginning of leaping phase
  double dz_f;      // Velocity at the end of landing phase
};

struct StateActionResult {
  State s_new;        // New State
  Action a_new;       // New Action
  double t_new = 0;   // !!!
  double length = 0;  // Distance between new State and previous State
};

/**
 * @brief Truncate FullState to State
 * @param[in] full_state FullState
 * @return State after truncation
 */
State fullStateToState(const FullState &full_state);

/**
 * @brief Extend State to FullState, adding body orientation and angular speed
 * @param[in] state State
 * @param[in] roll Roll
 * @param[in] pitch Pitch
 * @param[in] yaw Yaw
 * @param[in] roll_rate Change rate of roll
 * @param[in] pitch Change rate of Pitch
 * @param[in] yaw Change rate of Yaw
 * @return FullState after extension
 */
FullState stateToFullState(const State &state, double roll, double pitch,
                           double yaw, double roll_rate, double pitch_rate,
                           double yaw_rate);

/**
 * @brief Reformat Eigen vector to FullState
 * @param[in] s_eig Eigen vector
 * @param[out] s FullState obtained from Eigen vector
 */
void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s);

/**
 * @brief Reformat FullState to Eigen vector
 * @param[in] s FullState
 * @return Eigen vector obtained from FullState
 */
Eigen::VectorXd fullStateToEigen(const FullState &s);

/**
 * @brief Refromat STL vector to FulState
 * @param[in] v STL vector
 * @param[out] s FullState obtained from STL vector
 */
void vectorToFullState(const std::vector<double> &v, FullState &s);

/**
 * @brief Reverse State velocity direction
 * @param[out] s State
 */
void flipDirection(State &s);

/**
 * @brief Reverse GRF, vertical velocities,and stance times of landing and
 * leaping if there is an actual leap
 * @param[out] a Action
 */
void flipDirection(Action &a);

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
 * @brief Print FullState along with a separate line
 * @param[in] s FullState
 */
void printStateNewline(const State &s);

/**
 * @brief Print Action
 * @param[in] a Action
 */
void printAction(const Action &a);

/**
 * @brief Print Action along with a separate line
 * @param[in] a Action
 */
void printActionNewline(const Action &a);

/**
 * @brief Print The sequence of States
 * @param[in] state_sequence The sequence of States
 */
void printStateSequence(const std::vector<State> &state_sequence);

/**
 * @brief Print The sequence of States along with corresponding timesteps
 * @param[in] state_sequence The sequence of State
 * @param[in] state_sequence The sequence of timestep at which particular
 * interpolated States occur
 */
void printInterpStateSequence(const std::vector<State> &state_sequence,
                              const std::vector<double> &interp_t);

/**
 * @brief Print The sequence of Actions
 * @param[in] state_sequence The sequence of Actions
 */
void printActionSequence(const std::vector<Action> &action_sequence);

/**
 * @brief Calculate the distance between two State' positions
 * @param[in] q1 State 1
 * @param[in] q2 State 2
 * @return The distance between two State' positions
 */
double poseDistance(const State &q1, const State &q2);

/**
 * @brief Calculate the distance between two State' positions
 * @param[in] q1 FullState 1
 * @param[in] q2 FullState 2
 * @return The distance between two State' positions
 */
double poseDistance(const FullState &q1, const FullState &q2);

/**
 * @brief Calculate the Euclidean distance between two positions in STL vector
 * form
 * @param[in] q1 STL vector 1
 * @param[in] q2 STL vector 2
 * @return The Euclidean distance between two positions in std vectorform
 */
double poseDistance(const std::vector<double> &v1,
                    const std::vector<double> &v2);

/**
 * @brief Calculate the Euclidean distance between positions plus Euclidean
 * distance between velocity vectors of two States
 * @param[in] q1 State 1
 * @param[in] q2 State 2
 * @return The Euclidean distance between positions plus Euclidean
 * distance between velocity vectors of two States
 */
double stateDistance(const State &q1, const State &q2);

/**
 * @brief Transform GRF from contact frame to spatial frame
 * @param[in] surface_norm Surface normal vector at contact point
 * @param[in] grf Ground reaction force at contact point
 * @return Ground reaction force in spatial frame
 */
Eigen::Vector3d rotateGRF(const Eigen::Vector3d &surface_norm,
                          const Eigen::Vector3d &grf);

/**
 * @brief Inline function to get State speed (scalar)
 * @param[in] s State
 * @return State speed (scalar)
 */
inline double getSpeed(const State &s) { return s.vel.norm(); }

/**
 * @brief Append FullState to State and FullState arrays
 * @param[in] start_state FullState to be appended
 * @param[out] interp_reduced_path The sequence of States to be appended upon
 * @param[in] dt Time resolution
 * @param[out] interp_full_path The sequence of FullStates to be appended upon
 * @param[in] planner_config Configuration parameters
 */
void addFullStates(const FullState &start_state,
                   std::vector<State> interp_reduced_path, double dt,
                   std::vector<FullState> &interp_full_path,
                   const PlannerConfig &planner_config);

/**
 * @brief Interpolating States based on Action
 * @param[in] s The initial State
 * @param[in] a Action to be applied on States
 * @param[in] t0 The initial timestep
 * @param[in] dt Time resolution
 * @param[out] interp_plan The sequence of interpolated States on timesteps
 * throughout stance-flight-stance phases
 * @param[out] inter_GRF The sequence of ground reaction force along the path
 * @param[out] interp_t The sequence of timesteps along the path
 * @param[out] interp_primitive_id The sequence of corresponding Phase label
 * along the path
 * @param[out] interp_length The sequence of accumulated distances from inital
 * State to each interpolated State position
 * @param[in] planner_config Configuration parameters
 * @return The final State along the path
 */
State interpStateActionPair(const State &s, const Action &a, double t0,
                            double dt, std::vector<State> &interp_plan,
                            std::vector<GRF> &interp_GRF,
                            std::vector<double> &interp_t,
                            std::vector<int> &interp_primitive_id,
                            std::vector<double> &interp_length,
                            const PlannerConfig &planner_config);

/**
 * @brief Retrieve the path of FullState by interpolating States based on Action
 * @param[in] start_state The initial State
 * @param[in] state_sequence The sequence of states in the path
 * @param[in] action_sequence The sequence of actions in the path
 * @param[in] dt Time resolution
 * @param[in] t0 The initial timestep
 * @param[out] inter_full_plan The interpolated FullStates along the path
 * @param[out] interp_GRF The sequence of ground reaction force along the path
 * @param[out] interp_t The sequence of timesteps along the path
 * @param[out] interp_primitive_id The sequence of corresponding Phase label
 * along the path
 * @param[out] interp_length The sequence of accumulated distances from inital
 * State to each interpolated State position
 * @param[in] planner_config Configuration parameters
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
 * @brief Get body pitch from State
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Pitch of current State
 */
double getPitchFromState(const State &s, const PlannerConfig &planner_config);

/**
 * @brief Align lateral velocity along surface normal of filtered map
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return delta z
 */
double getDzFromState(const State &s, const PlannerConfig &planner_config);

/**
 * @brief Align lateral velocity along surface normal of filtered map
 * @param[out] s State
 * @param[in] planner_config Configuration parameters
 * @return delta z
 */
void setDz(State &s, const PlannerConfig &planner_config);

/**
 * @brief Align lateral velocity along surface normal of filtered map
 * @param[out] s State
 * @param[in] planner_config Configuration parameters
 * @return delta z
 */
void setDz(State &s, const Eigen::Vector3d &surf_norm);

/**
 * @brief Inline function to check if State pos is inside
 * map range or not
 * @param[in] pos State pos in Eigen vector form
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
 * @brief Inline function to get the terrain height at State position
 * @param[in] pos State position
 * @param[in] planner_config Configuration parameters
 * @return The terrain height at State position
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
 * @brief Inline function to get the filtered terrain height at State position
 * @param[in] pos State position
 * @param[in] planner_config Configuration parameters
 * @return The filtered terrain height at State position
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
 * @brief Inline function to get the traversability at State position
 * @param[in] pos State position
 * @param[in] planner_config Configuration parameters
 * @return The traversability at State position
 */
inline double getTraversability(const Eigen::Vector3d &pos,
                                const PlannerConfig &planner_config) {
  return planner_config.terrain_grid_map.atPosition("traversability",
                                                    pos.head<2>(), INTER_TYPE);
}

/**
 * @brief Inline function to check whether position is traversable above a
 * threshold
 * @param[in] pos Position for checking
 * @param[in] planner_config Configuration parameters
 * @return Whether positon is traversable at position on map
 */
inline bool isTraversable(const Eigen::Vector3d &pos,
                          const PlannerConfig &planner_config) {
  return (getTraversability(pos, planner_config) >=
          planner_config.traversability_threshold);
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
<<<<<<< HEAD
}

/**
 * @brief Inline function to get the terrain height of State position
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Terrain height at the State position
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
 *  terrain height and  State +z position
 * @param[in] pos State position in Eigen vector form
 * @param[in] planner_config Configuration parameters
 * @return The relative difference between terrain height and  State +z position
 */
inline double getZRelToTerrain(const Eigen::Vector3d &pos,
                               const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZ(pos, planner_config));
}

/**
 * @brief Inline function to get the relative difference between
 * terrain height and State +z position
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return The relative difference between terrain height and State +z position
 */
inline double getZRelToTerrain(const State &s,
                               const PlannerConfig &planner_config) {
  return getZRelToTerrain(s.pos, planner_config);
}

/**
 * @brief Inline function to get the relative difference between
 * filtered terrain height and State +z position
 * @param[in] pos State position
 * @param[in] planner_config Configuration parameters
 * @return the relative difference between filtered terrain height and State +z
 * position
 */
inline double getZRelToTerrainFiltered(const Eigen::Vector3d &pos,
                                       const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZFiltered(pos, planner_config));
}

/**
 * @brief Inline function to get the relative difference between
 * filtered terrain height and State position
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
 * @param[in] planner_config Configuration parameters
 * @param[in] x_min The minimal x
 * @param[in] x_max The maximal x
 * @param[in] y_min The minimal y
 * @param[in] y_max The maximal x
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
=======
}

/**
 * @brief Inline function to get the terrain height of State position
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return Terrain height at the State position
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
 *  terrain height and  State +z position
 * @param[in] pos State position in Eigen vector form
 * @param[in] planner_config Configuration parameters
 * @return The relative difference between terrain height and  State +z position
 */
inline double getZRelToTerrain(const Eigen::Vector3d &pos,
                               const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZ(pos, planner_config));
}

/**
 * @brief Inline function to get the relative difference between
 * terrain height and State +z position
 * @param[in] s State
 * @param[in] planner_config Configuration parameters
 * @return The relative difference between terrain height and State +z position
 */
inline double getZRelToTerrain(const State &s,
                               const PlannerConfig &planner_config) {
  return getZRelToTerrain(s.pos, planner_config);
}

/**
 * @brief Inline function to get the relative difference between
 * filtered terrain height and State +z position
 * @param[in] pos State position
 * @param[in] planner_config Configuration parameters
 * @return the relative difference between filtered terrain height and State +z
 * position
 */
inline double getZRelToTerrainFiltered(const Eigen::Vector3d &pos,
                                       const PlannerConfig &planner_config) {
  return (pos[2] - getTerrainZFiltered(pos, planner_config));
}

/**
 * @brief Inline function to get the relative difference between
 * filtered terrain height and State position
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
 * @param[in] planner_config Configuration parameters
 * @param[in] x_min The minimal x
 * @param[in] x_max The maximal x
 * @param[in] y_min The minimal y
 * @param[in] y_max The maximal x
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
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
 * @brief Obtain new State after applying Action
 * @param[in] s The initial State
 * @param[in] a The Action applied
 * @param[in] t time last for Action
 * @param[in] phase Phase variable which the State is under, CONNECT or others？
 * @param[in] planner_config Configuration parameters
 * @return New State after Action appiled for the period of time
 */
State applyStance(const State &s, const Action &a, double t, int phase,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State after applying Action during the leaping and landing
 * phase
 * @param[in] s The initial State
 * @param[in] a The Action applied
 * @param[in] phase Phase variable which the State is under, CONNECT or
 * LEAP_STANCE？
 * @param[in] planner_config Configuration parameters
 * @return New State after Action appiled for the period of time
 */
State applyStance(const State &s, const Action &a, int phase,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State after the flight phase
 * @param[in] s The initial State
 * @param[in] t_f Time last in the flight phase
 * @param[in] planner_config Configuration parameters
 * @return New State after the flight phase
 */
State applyFlight(const State &s, double t_f,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State by applying Action
 * @param[in] s The initial State
 * @param[in] t_f Time last in the flight phase
 * @param[in] planner_config Configuration parameters
 * @return New State after the flight phase
 */
State applyAction(const State &s, const Action &a,
                  const PlannerConfig &planner_config);

/**
 * @brief Obtain new State height after applying new flight
 * @param[in] s State
 * @param[in] t
 * @param[in] phase The phase the State is under
 * @param[in] planner_config Configuration parameters
 * @return The ground reaction force
 */
GRF getGRF(const Action &a, double t, int phase,
           const PlannerConfig &planner_config);

/**
 * @brief Get the acceleration after applying the Action
 * @param[in] t
 * @param[in] phase The phase the State is under
 * @param[in] planner_config Configuration parameters
 * @return The acceleration
 */
Eigen::Vector3d getAcceleration(const Action &a, double t, int phase,
                                const PlannerConfig &planner_config);

// Action sampling
/**
 * @brief Obtain new State height after applying new flight
 * @param[in] s The State
 * @param[in] surf_norm Surface normal
 * @param[in] a The Action
 * @param[in] planner_config Configuration parameters
 */
bool getRandomLeapAction(const State &s, const Eigen::Vector3d &surf_norm,
                         Action &a, const PlannerConfig &planner_config);

// Action refinement (for improved feasiblity)
/**
 * @brief Refine Action throughout all phases
 * @param[in] s State
 * @param[out] a Action to be refined
 * @param[in] planner_config Configuration parameters
 * @return Whether refinement is successfuls or not
 */
bool refineAction(const State &s, Action &a,
                  const PlannerConfig &planner_config);
/**
 * @brief Refine stance Action to validify ground reaction force, friction cone,
 * final state and midstance state
 * @param[in] s The initial State
 * @param[in] phase LEAP_STANCE or LAND_STANCE
 * @param[out] a Action in the stance phase to be refined
 * @param[in] planner_config Configuration parameters
 * @return Whether refinement is successful or not
 */
bool refineStance(const State &s, int phase, Action &a,
                  const PlannerConfig &planner_config);
/**
 * @brief Refine flight Action by extending the flight phase length
 * @param[in] s The initial State
 * @param[out] t_f Time last in the flight phase
 * @param[in] planner_config Configuration parameters
 * @return Whether refinement is successful or not
 */
bool refineFlight(const State &s, double &t_f,
                  const PlannerConfig &planner_config);

// Instantaneous validity checking
/**
 * @brief Check the Action is valid or not in terms of force limits and friction
 * cone
 * @param[in] a The Action to be checked
 * @param[in] planner_config Configuration parameters
 * @return Whether the Action is valid or not
 */
bool isValidAction(const Action &a, const PlannerConfig &planner_config);

/**
 * @brief Check the State is valid or not in terms of traversability,
 * reachability, collision, etc.
 * @param[in] s The State to be checked
 * @param[in] planner_config Configuration parameters
 * @param[in] phase The phase the State is under
 * @param[in] max_height Maximum height
 * @return Whether the State is valid or not
 */
bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase);

/**
 * @brief Check the State is valid or not in terms of traversability,
 * reachability, collision, etc.
 * @param[in] s The State to be checked
 * @param[in] planner_config Configuration parameters
 * @param[in] phase The phase the State is under
 * @param[in] max_height Maximum height
 * @return Whether the State is valid or not
 */
bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase, double &max_height);

// Trajectory validity checking
/**
 * @brief Check the validity of the whole trajectory
 * @param[in] s The State to be checked
 * @param[in] a The Action to be checked
 * @param[in] result The StateActionResult of the trajeccotry
 * @param[in] planner_config Configuration parameters
 * @return Whether the StateActionPair is valid or not
 */
bool isValidStateActionPair(const State &s, const Action &a,
                            StateActionResult &result,
                            const PlannerConfig &planner_config);

// Define visualization functions
/**
 * @brief Publish StateAction Pair
 * @param[in] s The State to be checked
 * @param[in] a The Action to be checked
 * @param[in] result The StateActionResult of the trajeccotry
 * @param[in] planner_config Configuration parameters
 * @return Whether the StateActionPair is valid or not
 */
void publishStateActionPair(const State &s, const Action &a,
                            const State &s_goal,
                            const PlannerConfig &planner_config,
                            visualization_msgs::MarkerArray &tree_viz_msg,
                            ros::Publisher &tree_pub);
}  // namespace planning_utils

#endif
