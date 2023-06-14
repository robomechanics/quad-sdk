#ifndef GLOBAL_BODY_PLAN_H
#define GLOBAL_BODY_PLAN_H

#include <quad_utils/ros_utils.h>

#include "global_body_planner/planning_utils.h"

using namespace planning_utils;

//! A class to contain global plan data along with helper functions
/*!
   This class inherits GraphClass, and adds method to add random states to the
   graph and search for neighbors. These functions are useful for sample-based
   planners such as RRTs or PRMs.
*/
class GlobalBodyPlan {
 public:
  /**
   * @brief Constructor for GlobalBodyPlan
   * @return Constructed object of type GlobalBodyPlan
   */
  GlobalBodyPlan();

  bool operator==(GlobalBodyPlan p1) const {
    return (p1.getComputedTimestamp() == computed_timestamp_);
  }

  /**
   * @brief Check if this plan is empty
   * @return Plan emptiness
   */
  inline bool isEmpty() const { return t_plan_.empty(); }

  /**
   * @brief Get the status of this plan
   * @return Plan status ID
   */
  inline int getStatus() const { return plan_status_; }

  /**
   * @brief Set the status of this plan
   * @param[in] status Plan status ID
   */
  inline void setStatus(int status) { plan_status_ = status; }

  /**
   * @brief Get the size of the plan (number of finite elements)
   * @return Plan size
   */
  inline int getSize() const { return t_plan_.size(); }

  /**
   * @brief Get the duration of the plan in seconds.
   * @return Plan duration
   */
  inline double getDuration() const { return t_plan_.back(); }

  /**
   * @brief Get the length of the plan in meters.
   * @return Plan length
   */
  inline double getLength() const { return length_plan_.back(); }

  /**
   * @brief Get the length of the plan at a particular index in meters.
   * @return Plan length at index
   */
  inline double getLengthAtIndex(int i) const { return length_plan_.at(i); }

  /**
   * @brief Get the time of the plan at a particular index in seconds.
   * @return Plan time at index
   */
  inline double getTime(int i) const { return t_plan_.at(i); }

  /**
   * @brief Get a particular state from the plan
   * @return State at index i
   */
  inline FullState getStateFromIndex(int i) const { return body_plan_.at(i); }

  /**
   * @brief Get the motion primitive from the plan
   * @return Primitive at index i
   */
  inline int getPrimitiveFromIndex(int i) const {
    return primitive_id_plan_.at(i);
  }

  /**
   * @brief Get the distance of the plan end to the goal (retrieves member
   * variable, does not compute)
   * @return Distance from end of plan to goal
   */
  inline double getGoalDistance() const { return goal_distance_; }

  /**
   * @brief Set the timestamp at which this plan was computed (must be unique)
   * @param[in] timestamp Timestamp at which the plan returned
   */
  inline void setComputedTimestamp(ros::Time timestamp) {
    computed_timestamp_ = timestamp;
  }

  /**
   * @brief Get the timestamp at which this plan was computed
   * @return Timestamp at which the plan returned
   */
  inline ros::Time getComputedTimestamp() const { return computed_timestamp_; }

  /**
   * @brief Set the timestamp at which this plan was published
   * @param[in] timestamp Timestamp at which the plan was published
   */
  inline void setPublishedTimestamp(ros::Time timestamp) {
    published_timestamp_ = timestamp;
  }

  /**
   * @brief Get the timestamp at which this plan was published
   * @return Timestamp at which the plan was published
   */
  inline ros::Time getPublishedTimestamp() const {
    return published_timestamp_;
  }

  /**
   * @brief Reset the plan (clear all data, update data to unsolved)
   */
  void clear();

  /**
   * @brief Invalidate the plan so next will be accepted (set length to infinite
   * and status to unsolved)
   */
  void invalidate();

  /**
   * @brief Clear the plan after a particular index (used for replanning
   * segments)
   * @param[in] start_index Index which will start new plan (all higher indices
   * cleared)
   */
  void eraseAfterIndex(int start_index);

  /**
   * @brief Load the interpolated plan data and metadata
   * @param[in] plan_status Status of this particular plan
   * @param[in] start_state State at which this plan begins
   * @param[in] dist_to_goal Distance to goal from plan end
   * @param[in] state_sequence Discrete state sequence to be interpolated
   * @param[in] action_sequence Discrete action sequence to be interpolated
   * @param[in] dt Timestep for interpolation
   * @param[in] t0 Start time of this plan section
   * @param[in] planner_config Planning configuration parameters
   */
  void loadPlanData(int plan_status, FullState& start_state,
                    double dist_to_goal, std::vector<State>& state_sequence,
                    std::vector<Action>& action_sequence, double dt, double t0,
                    const PlannerConfig& planner_config);

  /**
   * @brief Load a semgent of interpolated plan data into a plan message
   * @param[in] t Time at which this data occurs
   * @param[in] plan_index Index in the plan for which this data will be
   * inserted
   * @param[in] body_state Body state data
   * @param[in] grf GRF data
   * @param[out] msg Robot plan message with data added
   */
  void addStateAndGRFToMsg(double t, int plan_index,
                           const FullState& body_state, const GRF& grf,
                           int primitive_id, quad_msgs::RobotPlan& msg);

  /**
   * @brief Load the entire plan into a plan message
   * @param[out] robot_plan_msg Interpolated robot plan message
   * @param[out] discrete_robot_plan_msg Discrete robot plan message
   */
  void convertToMsg(quad_msgs::RobotPlan& robot_plan_msg,
                    quad_msgs::RobotPlan& discrete_robot_plan_msg);

 private:
  /// Time stamp for when plan was computed (unique)
  ros::Time computed_timestamp_;

  /// Time stamp for when plan was published (not unique)
  ros::Time published_timestamp_;

  /// Std vector containing the interpolated time data
  std::vector<double> t_plan_;

  /// Std vector containing the interpolated robot body plan
  std::vector<FullState> body_plan_;

  /// Std vector containing the interpolated wrench plan
  std::vector<GRF> grf_plan_;

  /// Std vector containing the interpolated time data
  std::vector<int> primitive_id_plan_;

  /// Std vector containing the cumulative path length at each index of the plan
  std::vector<double> length_plan_;

  /// Sequence of discrete states in the plan
  std::vector<State> state_sequence_;

  /// Sequence of discrete actions in the plan
  std::vector<Action> action_sequence_;

  /// Plan status
  int plan_status_;

  /// Distance to goal
  double goal_distance_;
};

#endif  // GLOBAL_BODY_PLAN_H
