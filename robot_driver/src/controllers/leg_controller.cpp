#include "robot_driver/controllers/leg_controller.hpp"

LegController::LegController(rclcpp::Node::SharedPtr node,
                             const std::string& robot_ns,
                             std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : node_(node), robot_ns_(robot_ns), quadKD_(quadKD) {
  override_state_machine_ = false;
}

void LegController::updateLocalPlanMsg(quad_msgs::msg::RobotPlan::SharedPtr msg,
                                       const rclcpp::Time& t_msg) {
  last_local_plan_msg_ = msg;
  last_local_plan_time_ = t_msg;
}

void LegController::init(double kp, double kd) {
  std::vector<double> kp_vec = {kp, kp, kp};
  std::vector<double> kd_vec = {kd, kd, kd};
  stance_kp_ = kp_vec;
  stance_kd_ = kd_vec;
  swing_kp_ = kp_vec;
  swing_kd_ = kd_vec;
}

void LegController::init(const std::vector<double>& kp,
                         const std::vector<double>& kd) {
  stance_kp_ = kp;
  stance_kd_ = kd;
  swing_kp_ = kp;
  swing_kd_ = kd;
}

void LegController::init(const std::vector<double>& stance_kp,
                         const std::vector<double>& stance_kd,
                         const std::vector<double>& swing_kp,
                         const std::vector<double>& swing_kd,
                         const std::vector<double>& swing_kp_cart,
                         const std::vector<double>& swing_kd_cart) {
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
  swing_kp_cart_ = swing_kp_cart;
  swing_kd_cart_ = swing_kd_cart;
}

void LegController::init(const std::vector<double>& stance_kp,
                         const std::vector<double>& stance_kd,
                         const std::vector<double>& swing_kp,
                         const std::vector<double>& swing_kd,
                         const std::vector<double>& swing_kp_cart,
                         const std::vector<double>& swing_kd_cart,
                         const std::string& model_path,
                         double /*policy_inference_rate*/) {
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
  swing_kp_cart_ = swing_kp_cart;
  swing_kd_cart_ = swing_kd_cart;
  model_path_ = model_path;
}

void LegController::init(const std::vector<double>& stance_kp,
                         const std::vector<double>& stance_kd,
                         const std::vector<double>& swing_kp,
                         const std::vector<double>& swing_kd,
                         const std::vector<double>& swing_kp_cart,
                         const std::vector<double>& swing_kd_cart,
                         const std::string& model_path,
                         double policy_inference_rate,
                         const std::vector<double>& /*stand_joint_angles*/) {
  init(stance_kp, stance_kd, swing_kp, swing_kd, swing_kp_cart, swing_kd_cart,
       model_path, policy_inference_rate);
}
