#include "leg_controller/leg_controller_template.h"

LegControllerTemplate::LegControllerTemplate() {
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
  override_state_machine_ = false;

  // Initialize contact sensing message
  last_contact_sensing_msg_.data.assign(4, false);

  // Initialize joint position record for the missing foot
  joint_pos_miss_contact_ = Eigen::VectorXd::Zero(12);
}

void LegControllerTemplate::updateLocalPlanMsg(quad_msgs::RobotPlan::ConstPtr msg,
  const ros::Time &t_msg) {

  last_local_plan_msg_ = msg;
  last_local_plan_time_ = t_msg;
}

void LegControllerTemplate::setGains(double kp, double kd) {
  std::vector<double> kp_vec = {kp, kp, kp};
  std::vector<double> kd_vec = {kd, kd, kd};
  stance_kp_ = kp_vec;
  stance_kd_ = kd_vec;
  swing_kp_ = kp_vec;
  swing_kd_ = kd_vec;
}

void LegControllerTemplate::setGains(std::vector<double> kp, std::vector<double> kd) {
  stance_kp_ = kp;
  stance_kd_ = kd;
  swing_kp_ = kp;
  swing_kd_ = kd;
}

void LegControllerTemplate::setGains(std::vector<double> stance_kp, std::vector<double> stance_kd,
  std::vector<double> swing_kp, std::vector<double> swing_kd) {

  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
}

void LegControllerTemplate::updateGrfSensorMsg(quad_msgs::GRFArray::ConstPtr msg)
{
  last_grf_sensor_msg_ = msg;
}

std_msgs::ByteMultiArray LegControllerTemplate::getContactSensingMsg()
{
  return last_contact_sensing_msg_;
}