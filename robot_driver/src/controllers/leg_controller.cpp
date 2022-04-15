#include "robot_driver/controllers/leg_controller.h"

LegController::LegController() {
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
  override_state_machine_ = false;
}

void LegController::updateLocalPlanMsg(quad_msgs::RobotPlan::ConstPtr msg,
                                       const ros::Time &t_msg) {
  last_local_plan_msg_ = msg;
  last_local_plan_time_ = t_msg;
}

void LegController::setGains(double kp, double kd) {
  std::vector<double> kp_vec = {kp, kp, kp};
  std::vector<double> kd_vec = {kd, kd, kd};
  stance_kp_ = kp_vec;
  stance_kd_ = kd_vec;
  swing_kp_ = kp_vec;
  swing_kd_ = kd_vec;
}

void LegController::setGains(std::vector<double> kp, std::vector<double> kd) {
  stance_kp_ = kp;
  stance_kd_ = kd;
  swing_kp_ = kp;
  swing_kd_ = kd;
}

void LegController::setGains(std::vector<double> stance_kp,
                             std::vector<double> stance_kd,
                             std::vector<double> swing_kp,
                             std::vector<double> swing_kd) {
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
}
