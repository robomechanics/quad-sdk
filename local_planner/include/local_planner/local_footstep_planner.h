#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/BodyPlan.h>
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
     * @param[in] grf_weight Weight on GRF projection (0 to 1)
     * @param[in] kinematics Kinematics class for computations
     */
    void setSpatialParams(double ground_clearance, double grf_weight,
      std::shared_ptr<spirit_utils::SpiritKinematics> kinematics);
    
    /**
     * @brief Compute the contact schedule based on the current phase
     * @param[in] phase current phase as index
     */
    void computeContactSchedule(int phase, std::vector<std::vector<bool>> &contact_schedule);

    /**
     * @brief Compute the contact schedule based on the current phase
     * @param[in] t Current time in trajectory
     */
    void computeContactSchedule(double t, std::vector<std::vector<bool>> &contact_schedule);

    /**
     * @brief Update the discrete footstep plan with the current plan
     * @param[in] state Current robot state
     * @param[in] body_plan Current body plan
     * @param[in] grf_plan Current grf plan
     * @param[in] contact_schedule Current contact schedule
     * @param[out] foot_positions Foot positions over the horizon
     */
    void computeFootPositions(const Eigen::MatrixXd &body_plan, const Eigen::MatrixXd &grf_plan,
      const std::vector<std::vector<bool>> &contact_schedule, Eigen::MatrixXd &foot_positions);

    /**
     * @brief Convert the foot positions and contact schedule into ros messages for the foot plan
     * @param[in] contact_schedule Current contact schedule
     * @param[in] foot_positions Foot positions over the horizon
     * @param[out] multi_foot_plan_discrete_msg Message for discrete footholds
     * @param[out] multi_foot_plan_continuous_msg Message for continuous foot trajectories
     */
    void computeFootPlanMsgs(
      const std::vector<std::vector<bool>> &contact_schedule, const Eigen::MatrixXd &foot_positions,
      spirit_msgs::MultiFootPlanDiscrete &multi_foot_plan_discrete_msg,
      spirit_msgs::MultiFootPlanContinuous &multi_foot_plan_continuous_msg);

  private:

    /**
     * @brief Update the continuous foot plan to match the discrete
     */
    void updateContinuousPlan();

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

};


#endif // LOCAL_FOOTSTEP_PLANNER_H
