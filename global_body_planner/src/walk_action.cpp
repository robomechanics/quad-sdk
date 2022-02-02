#include "global_body_planner/walk_action.h"

namespace planning_utils {

WalkAction::WalkAction(){}
WalkAction::~WalkAction(){}

bool WalkAction::isValid(){}

bool WalkAction::isValidState(const State &s){
  
}

State WalkAction::applyToState(const State &s){}

State WalkAction::applyToState(const State &s, double t){}

bool WalkAction::isValidStateActionPair(const State &s, StateActionResult &result){}

}