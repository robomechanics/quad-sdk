#ifndef ACTION_H
#define ACTION_H

#include "global_body_planner/planning_utils.h"

namespace planning_utils {

//! A class that defines the base functionality of a planning action.
/*!
  This class implements the base logic for an action that can be taken by the planner, including
    - State generation
    - Validity checking
    - Printing/debugging
  Each derived action must support bidirectional evaluation for deployment in RRT-Connect
*/
class Aktion
{
  public:
    /**
     * @brief Constructor for Action
     * @return Constructed object of type Action
     */
    Aktion();

    /**
     * @brief Destructor for Action
     */
    ~Aktion();

    /**
     * @brief Print this action via printf
     */
    virtual void print();

    /**
     * @brief Check the validity of this action
     * @return bool for action validity
     */
    virtual bool isValid(const PlannerConfig &planner_config) = 0;

    /**
     * @brief Check the validity of a state under this action
     * @param[in] s State to check validity
     * @return bool for state validity
     */
    virtual bool isValidState(const State &s, const PlannerConfig &planner_config) = 0;

    /**
     * @brief Apply the action to a state to obtain a new state
     * @param[in] s State from which to apply the action
     * @return State resulting from the applied action
     */
    virtual State applyToState(const State &s, const PlannerConfig &planner_config) = 0;

    /**
     * @brief Apply the action to a state to obtain a new state
     * @param[in] s State from which to apply the action
     * @param[in] t Time into the action to return the state
     * @return State resulting from the applied action
     */
    virtual State applyToState(const State &s, double t, const PlannerConfig &planner_config) = 0;

    /**
     * @brief Check the validity of a state action pair, and give the 
     * @param[in] s State from which to apply the action
     * @param[out] result The result of attempting to apply this action to the given state
     * @return bool for validity of this pair
     */
    virtual bool isValidStateActionPair(const State &s, StateActionResult &result,
      const PlannerConfig &planner_config) = 0;

  protected:
  
    /// Time of action
    double t_;

    /// Initial acceleration
    Eigen::Vector3d acc_i_ = {0,0,0};

    /// Final acceleration
    Eigen::Vector3d acc_f_ = {0,0,0};

  private:
};

}

#endif // ACTION_H