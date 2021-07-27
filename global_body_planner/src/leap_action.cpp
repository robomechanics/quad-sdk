#include "global_body_planner/leap_action.h"

namespace planning_utils {

LeapAction::LeapAction(){}
LeapAction::~LeapAction(){}

void LeapAction::print(){
  printf("acc_i_ = {%5.3f,%5.3f,%5.3f}, acc_f_ = {%5.3f,%5.3f,%5.3f}, t = %5.3f, t_f = %5.3f\n",
    acc_i_.x(),acc_i_.y(),acc_i_.z(),acc_f_.x(),acc_f_.y(),acc_f_.z(),t_,t_f_);
}

bool LeapAction::isValid(){}

bool LeapAction::isValidState(const State &s){}

State LeapAction::applyToState(const State &s){}

State LeapAction::applyToState(const State &s, double t){}

bool LeapAction::isValidStateActionPair(const State &s, StateActionResult &result){}

}