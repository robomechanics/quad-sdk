#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/RobotPlan.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/FootState.h>
#include <spirit_msgs/MultiFootState.h>
#include <spirit_msgs/FootPlanDiscrete.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_utils/fast_terrain_map.h>
#include <spirit_utils/function_timer.h>
#include <spirit_utils/math_utils.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/kinematics.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

//! A local footstep planning class for spirit
/*!
   FootstepPlanner is a container for all of the logic utilized in the local footstep planning node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class LocalFootstepPlanner {
  public:
    /**
     * @brief Constructor for LocalFootstepPlanner Class
     * @return Constructed object of type LocalFootstepPlanner
     */
    LocalFootstepPlanner();

    /**
     * @brief Set the temporal parameters of this object
     * @param[in] dt The duration of one timestep in the plan
     * @param[in] period The period of a gait cycle in number of timesteps
     * @param[in] horizon_length The length of the planning horizon in number of timesteps
     */
    void setTemporalParams(double dt, int period, int horizon_length);

    /**
     * @brief Set the spatial parameters of this object
     * @param[in] ground_clearance The foot clearance over adjacent footholds in cm
     * @param[in] standing_error_threshold Threshold of body error from desired goal to start stepping
     * @param[in] grf_weight Weight on GRF projection (0 to 1)
     * @param[in] kinematics Kinematics class for computations
     */
    void setSpatialParams(double ground_clearance, double grf_weight,double standing_error_threshold,
      std::shared_ptr<spirit_utils::SpiritKinematics> kinematics);

    /**
     * @brief Transform a vector of foot positions from the world to the body frame
     * @param[in] body_plan Current body plan
     * @param[in] foot_positions_world Foot positions in the world frame
     * @param[out] foot_positions_body Foot positions in the body frame
     */
    void getFootPositionsBodyFrame(const Eigen::VectorXd &body_plan,
      const Eigen::VectorXd &foot_positions_world, Eigen::VectorXd &foot_positions_body);

    /**
     * @brief Transform the entire foot plan from the world to the body frame
     * @param[in] body_plan Current body plan
     * @param[in] foot_positions_world Foot positions in the world frame
     * @param[out] foot_positions_body Foot positions in the body frame
     */
    void getFootPositionsBodyFrame(const Eigen::MatrixXd &body_plan,
      const Eigen::MatrixXd &foot_positions_world, Eigen::MatrixXd &foot_positions_body);

    /**
     * @brief Update the map of this object
     * @param[in] terrain The map of the terrain
     */
    void updateMap(const FastTerrainMap &terrain);
    
    /**
     * @brief Compute the contact schedule based on the current phase
     * @param[in] current_plan_index_ current index in the plan
     * @param[out] contact_schedule 2D array of contact states
     */
    void computeStanceContactSchedule(int current_plan_index_, 
      std::vector<std::vector<bool>> &contact_schedule);

    /**
     * @brief Compute the contact schedule based on the current phase
     * @param[in] current_plan_index_ Current index in the plan
     * @param[in] current_state Current robot state
     * @param[in] ref_body_plan Reference bode plan
     * @param[out] contact_schedule 2D array of contact states
     */
    void computeContactSchedule(int current_plan_index,
      Eigen::VectorXd current_state, Eigen::MatrixXd ref_body_plan,
      std::vector<std::vector<bool>> &contact_schedule) ;

    /**
     * @brief Update the discrete footstep plan with the current plan
     * @param[in] state Current robot state
     * @param[in] body_plan Current body plan
     * @param[in] grf_plan Current grf plan
     * @param[in] contact_schedule Current contact schedule
     * @param[in] current_state Current state of the robot body
     * @param[out] foot_positions Foot positions over the horizon
     */
    void computeFootPositions(const Eigen::MatrixXd &body_plan, const Eigen::MatrixXd &grf_plan,
      const std::vector<std::vector<bool>> &contact_schedule, const Eigen::MatrixXd &ref_body_plan,
      Eigen::MatrixXd &foot_positions);

    /**
     * @brief Convert the foot positions and contact schedule into ros messages for the foot plan
     * @param[in] contact_schedule Current contact schedule
     * @param[in] foot_positions Foot positions over the horizon
     * @param[in] current_plan_index Current index in the global plan
     * @param[out] past_footholds_msg Message for previous footholds
     * @param[out] future_footholds_msg Message for future (planned) footholds
     * @param[out] foot_plan_continuous_msg Message for continuous foot trajectories
     */
    void computeFootPlanMsgs(
      const std::vector<std::vector<bool>> &contact_schedule, const Eigen::MatrixXd &foot_positions,
      int current_plan_index, spirit_msgs::MultiFootPlanDiscrete &past_footholds_msg,
      spirit_msgs::MultiFootPlanDiscrete &future_footholds_msg,
      spirit_msgs::MultiFootPlanContinuous &foot_plan_continuous_msg);

    inline void printContactSchedule(const std::vector<std::vector<bool>> &contact_schedule) {
      
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

  private:

    /**
     * @brief Update the continuous foot plan to match the discrete
     */
    void updateContinuousPlan();

    /**
     * @brief Compute the foot state for a swing foot as a function of the previous and next steps
     * @param[in] foot_position_prev Previous foothold
     * @param[in] foot_position_next Next foothold
     * @param[in] swing_phase Phase variable for swing phase (as a fraction)
     * @param[in] swing_duration Duration of swing (in timesteps)
     * @param[out] foot_position Position of swing foot
     * @param[out] foot_velocity Velocity of swing foot
     */
    void computeSwingFootState(const Eigen::Vector3d &foot_position_prev,
      const Eigen::Vector3d &foot_position_next, double swing_phase, int swing_duration,
      Eigen::Vector3d &foot_position, Eigen::Vector3d &foot_velocity);

    /**
     * @brief Extract foot data from the matrix
     */
    inline Eigen::Vector3d getFootData(const Eigen::MatrixXd &foot_state_vars,
      int horizon_index, int foot_index) {

      return foot_state_vars.block<1,3>(horizon_index,3*foot_index);
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
    inline bool isNewContact(const std::vector<std::vector<bool>> &contact_schedule,
      int horizon_index, int foot_index) {
      
      if (horizon_index == 0)
        return false;

      return (!isContact(contact_schedule, horizon_index-1, foot_index) && 
        isContact(contact_schedule, horizon_index, foot_index));
    }

    /**
     * @brief Check if a foot is newly in swing at a given index
     */
    inline bool isNewLiftoff(const std::vector<std::vector<bool>> &contact_schedule,
      int horizon_index, int foot_index) {

      if (horizon_index == 0)
        return false;

      return (isContact(contact_schedule, horizon_index-1, foot_index) && 
        !isContact(contact_schedule, horizon_index, foot_index));
    }

    /**
     * @brief Compute the index of the next contact for a foot. If none exist return the last.
     */
    inline int getNextContactIndex(const std::vector<std::vector<bool>> &contact_schedule,
      int horizon_index, int foot_index) {

      // Loop through the rest of this contact schedule, if a new contact is found return its index
      for (int i_touchdown = horizon_index; i_touchdown < horizon_length_; i_touchdown++) {
        if (isNewContact(contact_schedule, i_touchdown, foot_index)) {
          return i_touchdown;
        }
      }

      // If no contact is found, return the last index in the horizon
      return (horizon_length_-1);
    }

    /// Handle for the map frame
    std::string map_frame_;

    /// Struct for terrain map data
    FastTerrainMap terrain_;

    /// Current continuous footstep plan
    spirit_msgs::MultiFootPlanContinuous multi_foot_plan_continuous_msg_;

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
    std::vector<double> phase_offsets_ = {0,0.5,0.5,0};

    /// Duty cycles for the stance duration of each foot
    std::vector<double> duty_cycles_ = {0.5,0.5,0.5,0.5};

    /// Nominal contact schedule
    std::vector<std::vector<bool>> nominal_contact_schedule_;

    /// Ground clearance
    double ground_clearance_;

    /// Weighting on the projection of the grf
    double grf_weight_;

    /// Primitive ids - FLIGHT
    const int FLIGHT = 0;

    /// Primitive ids - STANCE
    const int STANCE = 1;

    /// Primitive ids - CONNECT_STANCE
    const int CONNECT_STANCE = 2;

    /// Spirit Kinematics class
    std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;

    /// Threshold of body error from desired goal to start stepping
    double standing_error_threshold_ = 0;

};


#endif // LOCAL_FOOTSTEP_PLANNER_H
