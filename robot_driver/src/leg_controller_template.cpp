#include "robot_driver/leg_controller_template.h"

LegControllerTemplate::LegControllerTemplate() {
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
  override_state_machine_ = false;

  // Initialize contact state machine message
  contact_state_machine_.data.assign(4, STANCE);

  // Initialize new plan check vector
  get_new_plan_after_recovering_.assign(4, true);
}

void LegControllerTemplate::updateLocalPlanMsg(
    quad_msgs::RobotPlan::ConstPtr msg, const ros::Time &t_msg) {
  last_local_plan_msg_ = msg;
  last_local_plan_time_ = t_msg;

  for (size_t i = 0; i < 4; i++) {
    // If we recover from retraction and receive a new plan
    if (contact_state_machine_.data.at(i) == STANCE) {
      get_new_plan_after_recovering_.at(i) = true;
    }
  }
}

void LegControllerTemplate::setGains(double kp, double kd) {
  std::vector<double> kp_vec = {kp, kp, kp};
  std::vector<double> kd_vec = {kd, kd, kd};
  stance_kp_ = kp_vec;
  stance_kd_ = kd_vec;
  swing_kp_ = kp_vec;
  swing_kd_ = kd_vec;
}

void LegControllerTemplate::setGains(std::vector<double> kp,
                                     std::vector<double> kd) {
  stance_kp_ = kp;
  stance_kd_ = kd;
  swing_kp_ = kp;
  swing_kd_ = kd;
}

void LegControllerTemplate::setGains(
    std::vector<double> stance_kp, std::vector<double> stance_kd,
    std::vector<double> swing_kp, std::vector<double> swing_kd,
    std::vector<double> retraction_kp, std::vector<double> retraction_kd,
    std::vector<double> extend_kp, std::vector<double> extend_kd) {
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
  retraction_kp_ = retraction_kp;
  retraction_kd_ = retraction_kd;
  extend_kp_ = extend_kp;
  extend_kd_ = extend_kd;
}

void LegControllerTemplate::updateGrfSensorMsg(
    quad_msgs::GRFArray::ConstPtr msg) {
  last_grf_sensor_msg_ = msg;
}

std_msgs::UInt8MultiArray LegControllerTemplate::getcontactStateMachine() {
  return contact_state_machine_;
}
