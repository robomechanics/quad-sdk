#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <quad_msgs/FootPlanDiscrete.h>
#include <quad_msgs/FootState.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <quad_msgs/MultiFootPlanDiscrete.h>
#include <quad_msgs/MultiFootState.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/function_timer.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

//! A local footstep planning class for quad
/*!
   FootstepPlanner is a container for all of the logic utilized in the local
   footstep planning node. The implementation must provide a clean and high
   level interface to the core algorithm
*/
class LocalFootstepPlanner {
 public:
  // Time difference between two calls
  double time_diff_;

  /**
   * @brief Constructor for LocalFootstepPlanner Class
   * @return Constructed object of type LocalFootstepPlanner
   */
  LocalFootstepPlanner();

  /**
   * @brief Set the temporal parameters of this object
   * @param[in] dt The duration of one timestep in the plan
   * @param[in] period The period of a gait cycle in number of timesteps
   * @param[in] horizon_length The length of the planning horizon in number of
   * timesteps
   */
  void setTemporalParams(double dt, int period, int horizon_length,
                         const std::vector<double> &duty_cycles,
                         const std::vector<double> &phase_offsets);

  /**
   * @brief Set the spatial parameters of this object
   * @param[in] ground_clearance The foot clearance over adjacent footholds in m
   * @param[in] hip_clearance The foot clearance under hip in m
   * @param[in] standing_error_threshold Threshold of body error from desired
   * goal to start stepping
   * @param[in] grf_weight Weight on GRF projection (0 to 1)
   * @param[in] kinematics Kinematics class for computations
   * @param[in] foothold_search_radius Radius to locally search for valid
   * footholds (m)
   * @param[in] foothold_obj_threshold Minimum objective function value for
   * valid foothold
   * @param[in] obj_fun_layer Terrain layer for foothold search
   */
  void setSpatialParams(double ground_clearance, double hip_clearance,
                        double grf_weight, double standing_error_threshold,
                        std::shared_ptr<quad_utils::QuadKD> kinematics,
                        double foothold_search_radius,
                        double foothold_obj_threshold,
                        std::string obj_fun_layer);

  /**
   * @brief Transform a vector of foot positions from the world to the body
   * frame
   * @param[in] body_plan Current body plan
   * @param[in] foot_positions_world Foot positions in the world frame
   * @param[out] foot_positions_body Foot positions in the body frame
   */
  void getFootPositionsBodyFrame(const Eigen::VectorXd &body_plan,
                                 const Eigen::VectorXd &foot_positions_world,
                                 Eigen::VectorXd &foot_positions_body);

  /**
   * @brief Transform the entire foot plan from the world to the body frame
   * @param[in] body_plan Current body plan
   * @param[in] foot_positions_world Foot positions in the world frame
   * @param[out] foot_positions_body Foot positions in the body frame
   */
  void getFootPositionsBodyFrame(const Eigen::MatrixXd &body_plan,
                                 const Eigen::MatrixXd &foot_positions_world,
                                 Eigen::MatrixXd &foot_positions_body);

  /**
   * @brief Update the map of this object
   * @param[in] terrain The map of the terrain
   */
  void updateMap(const FastTerrainMap &terrain);

  /**
   * @brief Update the map of this object
   * @param[in] terrain The map of the terrain
   */
  void updateMap(const grid_map::GridMap &terrain);

  /**
   * @brief Compute the contact schedule based on the current phase
   * @param[in] current_plan_index_ current index in the plan
   * @param[out] contact_schedule 2D array of contact states
   */
  void computeStanceContactSchedule(
      int current_plan_index_,
      std::vector<std::vector<bool>> &contact_schedule);

  /**
   * @brief Compute the contact schedule based on the current phase
   * @param[in] current_plan_index_ Current index in the plan
   * @param[in] current_state Current robot state
   * @param[in] ref_body_plan Reference bode plan
   * @param[out] contact_schedule 2D array of contact states
   */
  void computeContactSchedule(int current_plan_index,
                              Eigen::VectorXd current_state,
                              Eigen::MatrixXd ref_body_plan,
                              std::vector<std::vector<bool>> &contact_schedule);

  /**
   * @brief Update the discrete footstep plan with the current plan
   * @param[in] state Current robot state
   * @param[in] body_plan Current body plan
   * @param[in] grf_plan Current grf plan
   * @param[in] contact_schedule Current contact schedule
   * @param[in] current_state Current state of the robot body
   * @param[out] foot_positions Foot positions over the horizon
   */
  void computeFootPositions(
      const Eigen::MatrixXd &body_plan, const Eigen::MatrixXd &grf_plan,
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::MatrixXd &ref_body_plan, Eigen::MatrixXd &foot_positions);

  /**
   * @brief Convert the foot positions and contact schedule into ros messages
   * for the foot plan
   * @param[in] contact_schedule Current contact schedule
   * @param[in] foot_positions Foot positions over the horizon
   * @param[in] current_foot_position Current foot position
   * @param[in] current_foot_velocity Current foot velocity
   * @param[in] current_plan_index Current index in the global plan
   * @param[in] body_plan Body plan from MPC
   * @param[in] first_element_duration Time duration to the next plan index
   * @param[out] past_footholds_msg Message for previous footholds
   * @param[out] future_footholds_msg Message for future (planned) footholds
   * @param[out] foot_plan_continuous_msg Message for continuous foot
   * trajectories
   */
  void computeFootPlanMsgs(
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::MatrixXd &foot_positions,
      const Eigen::VectorXd &current_foot_position,
      const Eigen::VectorXd &current_foot_velocity, int current_plan_index,
      const Eigen::MatrixXd &body_plan, const double &first_element_duration,
      quad_msgs::MultiFootPlanDiscrete &past_footholds_msg,
      quad_msgs::MultiFootPlanDiscrete &future_footholds_msg,
      quad_msgs::MultiFootPlanContinuous &foot_plan_continuous_msg);

  inline void printContactSchedule(
      const std::vector<std::vector<bool>> &contact_schedule) {
    for (int i = 0; i < contact_schedule.size(); i++) {
      for (int j = 0; j < contact_schedule.at(i).size(); j++) {
        if (contact_schedule[i][j]) {
          printf("1 ");
        } else {
          printf("0 ");
        }
      }
      printf("\n");
    }
  }

  inline double getTerrainHeight(double x, double y) {
    grid_map::Position pos = {x, y};
    double height = this->terrain_grid_.atPosition(
        "z_smooth", pos, grid_map::InterpolationMethods::INTER_LINEAR);
    return (height);
  }

  inline double getTerrainHeight(Eigen::Vector3d body_pos,
                                 Eigen::Vector3d body_rpy) {
    // Compute terrain height based on the projection on four hip positions
    std::vector<double> height;
    for (size_t i = 0; i < 4; i++) {
      Eigen::Vector3d hip_pos;

      // Compute nominal foot positions for kinematic and grf-projection
      // measures
      quadKD_->worldToNominalHipFKWorldFrame(i, body_pos, body_rpy, hip_pos);
      grid_map::Position hip_position_grid_map = {hip_pos.x(), hip_pos.y()};
      double hip_height = terrain_grid_.atPosition(
          "z_smooth", hip_position_grid_map,
          grid_map::InterpolationMethods::INTER_LINEAR);
      height.push_back(hip_height);
    }

    // Use the minimum
    return *std::min_element(height.begin(), height.end());
    // Use the average
    // return std::accumulate(height.begin(), height.end(), 0.0)/4;
  }

  inline double getTerrainSlope(double x, double y, double dx, double dy) {
    // std::array<double, 3> surf_norm =
    // this->terrain_.getSurfaceNormalFiltered(x, y); Use estimated grid map
    grid_map::Position pos = {x, y};
    Eigen::Vector3d surf_norm;
    surf_norm << terrain_grid_.atPosition(
        "normal_vectors_x", pos, grid_map::InterpolationMethods::INTER_NEAREST),
        terrain_grid_.atPosition("normal_vectors_y", pos,
                                 grid_map::InterpolationMethods::INTER_NEAREST),
        terrain_grid_.atPosition("normal_vectors_z", pos,
                                 grid_map::InterpolationMethods::INTER_NEAREST);

    double denom = dx * dx + dy * dy;
    if (denom <= 0 || surf_norm[2] <= 0) {
      double default_pitch = 0;
      return default_pitch;
    } else {
      double v_proj = (dx * surf_norm[0] + dy * surf_norm[1]) / sqrt(denom);
      double pitch = atan2(v_proj, surf_norm[2]);

      return pitch;
    }
  }

  inline void getTerrainSlope(double x, double y, double yaw, double &roll,
                              double &pitch) {
    // std::array<double, 3> surf_norm =
    // this->terrain_.getSurfaceNormalFiltered(x, y); Use estimated grid map
    grid_map::Position pos = {x, y};
    Eigen::Vector3d surf_norm;
    surf_norm << terrain_grid_.atPosition(
        "normal_vectors_x", pos, grid_map::InterpolationMethods::INTER_NEAREST),
        terrain_grid_.atPosition("normal_vectors_y", pos,
                                 grid_map::InterpolationMethods::INTER_NEAREST),
        terrain_grid_.atPosition("normal_vectors_z", pos,
                                 grid_map::InterpolationMethods::INTER_NEAREST);

    Eigen::Vector3d norm_vec(surf_norm.data());
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(norm_vec);
    double ang = acos(
        std::max(std::min(Eigen::Vector3d::UnitZ().dot(norm_vec), 1.), -1.));

    if (ang < 1e-3) {
      roll = 0;
      pitch = 0;
      return;
    } else {
      Eigen::Matrix3d rot(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
      axis = rot.transpose() * (axis / axis.norm());
      tf2::Quaternion quat(tf2::Vector3(axis(0), axis(1), axis(2)), ang);
      tf2::Matrix3x3 m(quat);
      double tmp;
      m.getRPY(roll, pitch, tmp);
    }
  }

 private:
  /**
   * @brief Update the continuous foot plan to match the discrete
   */
  void updateContinuousPlan();

  /**
   * @brief Compute the cubic hermite spline
   * @param[in] pos_prev Previous position
   * @param[in] vel_prev Previous velocity
   * @param[in] pos_next Next position
   * @param[in] vel_next Next velocity
   * @param[in] phase Interplation phase
   * @param[in] duration Interplation duration
   * @param[out] pos Interplated position
   * @param[out] vel Interplated velocity
   * @param[out] acc Interplated accleration
   */
  void cubicHermiteSpline(double pos_prev, double vel_prev, double pos_next,
                          double vel_next, double phase, double duration,
                          double &pos, double &vel, double &acc);

  /**
   * @brief Search locally around foothold for optimal location
   * @param[in] foot_position_prev Foothold to optimize around
   * @return Optimized foothold
   */
  Eigen::Vector3d getNearestValidFoothold(const Eigen::Vector3d &foot_position);

  /**
   * @brief Extract foot data from the matrix
   */
  inline Eigen::Vector3d getFootData(const Eigen::MatrixXd &foot_state_vars,
                                     int horizon_index, int foot_index) {
    return foot_state_vars.block<1, 3>(horizon_index, 3 * foot_index);
  }

  /**
   * @brief Check if a foot is in contact at a given index
   */
  inline bool isContact(const std::vector<std::vector<bool>> &contact_schedule,
                        int horizon_index, int foot_index) {
    return (contact_schedule.at(horizon_index).at(foot_index));
  }

  /**
   * @brief Check if a foot is newly in contact at a given index
   */
  inline bool isNewContact(
      const std::vector<std::vector<bool>> &contact_schedule, int horizon_index,
      int foot_index) {
    if (horizon_index == 0) return false;

    return (!isContact(contact_schedule, horizon_index - 1, foot_index) &&
            isContact(contact_schedule, horizon_index, foot_index));
  }

  /**
   * @brief Check if a foot is newly in swing at a given index
   */
  inline bool isNewLiftoff(
      const std::vector<std::vector<bool>> &contact_schedule, int horizon_index,
      int foot_index) {
    if (horizon_index == 0) return false;

    return (isContact(contact_schedule, horizon_index - 1, foot_index) &&
            !isContact(contact_schedule, horizon_index, foot_index));
  }

  /**
   * @brief Compute the index of the next contact for a foot. If none exist
   * return the last.
   */
  inline int getNextContactIndex(
      const std::vector<std::vector<bool>> &contact_schedule, int horizon_index,
      int foot_index) {
    // Loop through the rest of this contact schedule, if a new contact is found
    // return its index
    for (int i_touchdown = horizon_index; i_touchdown < horizon_length_;
         i_touchdown++) {
      if (isNewContact(contact_schedule, i_touchdown, foot_index)) {
        return i_touchdown;
      }
    }

    // If no contact is found, return the last index in the horizon
    return (horizon_length_ - 1);
  }

  /// Handle for the map frame
  std::string map_frame_;

  /// Struct for terrain map data
  FastTerrainMap terrain_;

  /// GridMap for terrain map data
  grid_map::GridMap terrain_grid_;

  /// Current continuous footstep plan
  quad_msgs::MultiFootPlanContinuous multi_foot_plan_continuous_msg_;

  /// Number of feet
  const int num_feet_ = 4;

  /// Number of cycles to plan
  int num_cycles_;

  /// Timestep for one finite element
  double dt_;

  /// Gait period in timesteps
  int period_;

  /// Horizon length in timesteps
  int horizon_length_;

  /// Phase offsets for the touchdown of each foot
  std::vector<double> phase_offsets_ = {0, 0.5, 0.5, 0.0};

  /// Duty cycles for the stance duration of each foot
  std::vector<double> duty_cycles_ = {0.5, 0.5, 0.5, 0.5};

  /// Nominal contact schedule
  std::vector<std::vector<bool>> nominal_contact_schedule_;

  /// Ground clearance
  double ground_clearance_;

  /// Hip clearance
  double hip_clearance_;

  /// Weighting on the projection of the grf
  double grf_weight_;

  /// Primitive ids - FLIGHT
  const int FLIGHT = 0;

  /// Primitive ids - STANCE
  const int STANCE = 1;

  /// Primitive ids - CONNECT_STANCE
  const int CONNECT_STANCE = 2;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// Threshold of body error from desired goal to start stepping
  double standing_error_threshold_ = 0;

  /// Radius to locally search for valid footholds (m)
  double foothold_search_radius_;

  /// Minimum objective function value for valid foothold
  double foothold_obj_threshold_;

  /// Terrain layer for foothold search
  std::string obj_fun_layer_;

  /// Toe radius
  double toe_radius = 0.02;

  /// Estimated ground height
  double ground_height_;
};

#endif  // LOCAL_FOOTSTEP_PLANNER_H
