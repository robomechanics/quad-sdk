#include "global_body_planner/action.h"

namespace planning_utils {

Aktion::Aktion(){}
Aktion::~Aktion(){}

void Aktion::print(){
  printf("acc_i_ = {%5.3f,%5.3f,%5.3f}, acc_f_ = {%5.3f,%5.3f,%5.3f}, t = %5.3f\n",
    acc_i_.x(),acc_i_.y(),acc_i_.z(),acc_f_.x(),acc_f_.y(),acc_f_.z(),t_);
}

bool Aktion::isValid(){}

bool Aktion::isValidState(const State &s){}

State Aktion::applyToState(const State &s){}

State Aktion::applyToState(const State &s, double t){}

bool Aktion::isValidStateActionPair(const State &s, StateActionResult &result){}

}