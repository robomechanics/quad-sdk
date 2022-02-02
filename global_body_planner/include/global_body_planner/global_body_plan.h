#ifndef GLOBAL_BODY_PLAN_H
#define GLOBAL_BODY_PLAN_H

#include "global_body_planner/planning_utils.h"
#include <quad_utils/ros_utils.h>

using namespace planning_utils;

//! A class to contain global plan data along with helper functions
/*!
   This class inherits GraphClass, and adds method to add random states to the graph and search for neighbors.
   These functions are useful for sample-based planners such as RRTs or PRMs.
*/
class GlobalBodyPlan
{
  public:
    /**
     * @brief Constructor for GlobalBodyPlan
     * @return Constructed object of type GlobalBodyPlan
     */
    GlobalBodyPlan() ;

    bool operator==(GlobalBodyPlan p1) {
      return (p1.getComputedTimestamp() == computed_timestamp_);
    };

    inline bool isEmpty() { return t_plan_.empty();};

    inline bool isWorseThan(int other_plan_status, double other_plan_length) {
      return (other_plan_status == VALID && plan_status_ != VALID);
    };

    inline int getStatus() { return plan_status_;};
    
    inline void setStatus(int status) { plan_status_ = status;};

    inline int getSize() { return t_plan_.size();};

    inline double getDuration() {return t_plan_.back();};
    
    inline double getLength() { return length_plan_.back();};

    inline double getLengthAtIndex(int i) { return length_plan_.at(i);};

    inline double getTime(int i) { return t_plan_.at(i);};

    inline FullState getState(int i) { return body_plan_.at(i);};

    inline double getGoalDistance() { return goal_distance_;};

    inline double setComputedTimestamp(ros::Time timestamp) { computed_timestamp_ = timestamp;};

    inline ros::Time getComputedTimestamp() { return computed_timestamp_;};

    inline double setPublishedTimestamp(ros::Time timestamp) { published_timestamp_ = timestamp;};

    inline ros::Time getPublishedTimestamp() { return published_timestamp_;};

    void reset();

    void invalidate();

    void eraseAfterIndex(int start_index);

    void load(int plan_status, FullState &start_state, double dist_to_goal,
      std::vector<State> &state_sequence, std::vector<Action> &action_sequence, double dt,
      double t0, const PlannerConfig &planner_config);

    void addStateAndGRFToMsg(double t, int plan_index, const FullState &body_state, 
        const GRF &grf, int primitive_id, quad_msgs::RobotPlan& msg);
    
    void toMsg(quad_msgs::RobotPlan &robot_plan_msg, quad_msgs::RobotPlan &discrete_robot_plan_msg);
  
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

#endif // GLOBAL_BODY_PLAN_H