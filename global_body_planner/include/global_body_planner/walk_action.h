#ifndef WALKACTION_H
#define WALKACTION_H

#include "global_body_planner/action.h"

namespace planning_utils {

//! A class that defines the walking action.
/*!
  This class implements the logic for a walk action that can be taken by the planner, including
    - State generation
    - Validity checking
    - Printing/debugging
*/
class WalkAction : public Aktion
{
  public:
    /**
     * @brief Constructor for WalkAction
     * @return Constructed object of type WalkAction
     */
    WalkAction();

    /**
     * @brief Destructor for Action
     */
    ~WalkAction();

    /**
     * @brief Check the validity of this action
     * @return bool for action validity
     */
    bool isValid();

    /**
     * @brief Check the validity of a state under this action
     * @param[in] s State to check validity
     * @return bool for state validity
     */
    bool isValidState(const State &s);

    /**
     * @brief Apply the action to a state to obtain a new state
     * @param[in] s State from which to apply the action
     * @return State resulting from the applied action
     */
    State applyToState(const State &s);

    /**
     * @brief Apply the action to a state to obtain a new state
     * @param[in] s State from which to apply the action
     * @param[in] t Time into the action to return the state
     * @return State resulting from the applied action
     */
    State applyToState(const State &s, double t);

    /**
     * @brief Check the validity of a state action pair, and give the 
     * @param[in] s State from which to apply the action
     * @param[out] result The result of attempting to apply this action to the given state
     * @return bool for validity of this pair
     */
    bool isValidStateActionPair(const State &s, StateActionResult &result);

  protected:

  private:
};

}

#endif // WALKACTION_H