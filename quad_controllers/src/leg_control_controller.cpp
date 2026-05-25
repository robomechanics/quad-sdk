#include "quad_controllers/leg_control_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "quad_controllers/laws/grf_pid_controller.hpp"
#include "quad_controllers/laws/inertia_estimation_controller.hpp"
#include "quad_controllers/laws/inverse_dynamics_controller.hpp"
#include "quad_controllers/laws/joint_controller.hpp"
#include "quad_controllers/laws/underbrush_inverse_dynamics.hpp"
#include "quad_utils/ros_utils.hpp"
#ifdef HAS_ONNXRUNTIME
#include "quad_controllers/laws/learned_policy.hpp"
#endif

namespace quad_controllers {

namespace {
// Fill a MotorCommand (mirrors robot_driver_utils::loadMotorCommandMsg).
inline void loadMotorCommandMsg(double pos, double vel, double ff, double kp,
                                double kd, quad_msgs::msg::MotorCommand& msg) {
  msg.pos_setpoint = pos;
  msg.vel_setpoint = vel;
  msg.torque_ff = ff;
  msg.kp = kp;
  msg.kd = kd;
}
}  // namespace

controller_interface::CallbackReturn LegControlController::onInitExtra() {
  try {
    auto_declare<std::string>("controller", "inverse_dynamics");
    auto_declare<std::vector<double>>("stance_kp", {});
    auto_declare<std::vector<double>>("stance_kd", {});
    auto_declare<std::vector<double>>("swing_kp", {});
    auto_declare<std::vector<double>>("swing_kd", {});
    auto_declare<std::vector<double>>("swing_kp_cart", {});
    auto_declare<std::vector<double>>("swing_kd_cart", {});
    auto_declare<std::vector<double>>("stand_kp", {});
    auto_declare<std::vector<double>>("stand_kd", {});
    auto_declare<std::vector<double>>("stand_joint_angles", {});
    auto_declare<double>("policy_inference_rate", 50.0);
    auto_declare<double>("cmd_vel_filter_const", 0.10);
    auto_declare<double>("cmd_vel_scale", 1.0);
    auto_declare<std::string>("model_path", "");
    // Underbrush swing-law params (mirror robot_driver.yaml's underbrush_swing
    // block; consumed only by the `underbrush` law via setUnderbrushParams()).
    auto_declare<double>("underbrush_swing.retract_vel", 15.0);
    auto_declare<double>("underbrush_swing.tau_push", 2.0);
    auto_declare<double>("underbrush_swing.tau_contact_start", 3.0);
    auto_declare<double>("underbrush_swing.tau_contact_end", 3.0);
    auto_declare<double>("underbrush_swing.min_switch", 0.1);
    auto_declare<double>("underbrush_swing.t_down", 0.135);
    auto_declare<double>("underbrush_swing.t_up", 0.04);
    // State source for the control law: always state/ground_truth (the sim
    // estimator_plugin publishes it; on hardware the estimate is mirrored there
    // too). state/estimate exists only for debugging. Overridden by
    // controllers.yaml, but default it correctly so a controller launched
    // without that file can't silently read the debug topic.
    auto_declare<std::string>("robot_state_topic", "state/ground_truth");
    // Path to the robot config yaml (e.g. go2.yaml). Retained for backward
    // compatibility but no longer consumed here: the leg/frame/body params that
    // QuadKD2's params are resolved through this controller's own parameter
    // interface (the spawner supplies the robot yaml via --param-file), so no
    // separate robot_config_file is needed.
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "onInitExtra failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LegControlController::initLegController() {
  // The leg-control laws run against this controller's own LifecycleNode
  // (now()/get_logger()/get_clock()); no helper node is created.
  auto node = get_node();
  if (controller_id_ == "inverse_dynamics") {
    leg_controller_ =
        std::make_shared<InverseDynamicsController>(node, robot_ns_, quadKD_);
  } else if (controller_id_ == "grf_pid") {
    leg_controller_ =
        std::make_shared<GrfPidController>(node, robot_ns_, quadKD_);
  } else if (controller_id_ == "joint") {
    leg_controller_ =
        std::make_shared<JointController>(node, robot_ns_, quadKD_);
  } else if (controller_id_ == "underbrush") {
    leg_controller_ = std::make_shared<UnderbrushInverseDynamicsController>(
        node, robot_ns_, quadKD_);
    // Push the underbrush swing-law params into the law (mirrors
    // robot_driver::initLegController()'s underbrush branch).
    if (auto* c = dynamic_cast<UnderbrushInverseDynamicsController*>(
            leg_controller_.get())) {
      c->setUnderbrushParams(underbrush_retract_vel_, underbrush_tau_push_,
                             underbrush_tau_contact_start_,
                             underbrush_tau_contact_end_,
                             underbrush_min_switch_, underbrush_t_down_,
                             underbrush_t_up_);
    }
  } else if (controller_id_ == "inertia_estimation") {
    leg_controller_ =
        std::make_shared<InertiaEstimationController>(node, robot_ns_, quadKD_);
#ifdef HAS_ONNXRUNTIME
  } else if (controller_id_ == "learned") {
    leg_controller_ =
        std::make_shared<LearnedPolicy>(node, robot_ns_, quadKD_);
#endif
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Unknown controller id: %s",
                 controller_id_.c_str());
    return false;
  }

  if (controller_id_ != "learned") {
    leg_controller_->init(stance_kp_, stance_kd_, swing_kp_, swing_kd_,
                          swing_kp_cart_, swing_kd_cart_);
  } else {
    leg_controller_->init(stance_kp_, stance_kd_, swing_kp_, swing_kd_,
                          swing_kp_cart_, swing_kd_cart_, model_path_,
                          policy_inference_rate_, stand_joint_angles_);
  }
  return true;
}

controller_interface::CallbackReturn LegControlController::onConfigureExtra() {
  auto& p = *get_node();
  controller_id_ = p.get_parameter("controller").as_string();
  stance_kp_ = p.get_parameter("stance_kp").as_double_array();
  stance_kd_ = p.get_parameter("stance_kd").as_double_array();
  swing_kp_ = p.get_parameter("swing_kp").as_double_array();
  swing_kd_ = p.get_parameter("swing_kd").as_double_array();
  swing_kp_cart_ = p.get_parameter("swing_kp_cart").as_double_array();
  swing_kd_cart_ = p.get_parameter("swing_kd_cart").as_double_array();
  stand_kp_ = p.get_parameter("stand_kp").as_double_array();
  stand_kd_ = p.get_parameter("stand_kd").as_double_array();
  stand_joint_angles_ = p.get_parameter("stand_joint_angles").as_double_array();
  policy_inference_rate_ = p.get_parameter("policy_inference_rate").as_double();
  cmd_vel_filter_const_ = p.get_parameter("cmd_vel_filter_const").as_double();
  cmd_vel_scale_ = p.get_parameter("cmd_vel_scale").as_double();
  model_path_ = p.get_parameter("model_path").as_string();
  robot_state_topic_ = p.get_parameter("robot_state_topic").as_string();
  underbrush_retract_vel_ =
      p.get_parameter("underbrush_swing.retract_vel").as_double();
  underbrush_tau_push_ =
      p.get_parameter("underbrush_swing.tau_push").as_double();
  underbrush_tau_contact_start_ =
      p.get_parameter("underbrush_swing.tau_contact_start").as_double();
  underbrush_tau_contact_end_ =
      p.get_parameter("underbrush_swing.tau_contact_end").as_double();
  underbrush_min_switch_ =
      p.get_parameter("underbrush_swing.min_switch").as_double();
  underbrush_t_down_ = p.get_parameter("underbrush_swing.t_down").as_double();
  underbrush_t_up_ = p.get_parameter("underbrush_swing.t_up").as_double();

  // QuadKD2 now accepts a LifecycleNode, so build it on THIS controller's own
  // node -- no helper node. QuadKD2 reads robot_description as a param, but a
  // controller gets the URDF via get_robot_description() (not a param), so
  // publish it as one first. The leg/frame/body params come from go2.yaml, which
  // the spawner supplies to this node via --params-file (QuadKD2 declares-if-
  // needed to pick them up).
  if (!get_node()->has_parameter("robot_description")) {
    get_node()->declare_parameter("robot_description", get_robot_description());
  }
  try {
    quadKD_ = std::make_shared<quad_utils::QuadKD2>(get_node(), robot_ns_);
    if (!initLegController()) return controller_interface::CallbackReturn::ERROR;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "law construction failed: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  local_plan_sub_ = get_node()->create_subscription<quad_msgs::msg::RobotPlan>(
      "local_plan", 1, [this](quad_msgs::msg::RobotPlan::SharedPtr msg) {
        if (leg_controller_)
          leg_controller_->updateLocalPlanMsg(msg, get_node()->now());
      });
  robot_state_sub_ = get_node()->create_subscription<quad_msgs::msg::RobotState>(
      robot_state_topic_, rclcpp::SensorDataQoS(),
      [this](quad_msgs::msg::RobotState::SharedPtr msg) {
        last_robot_state_msg_ = msg;
      });
  single_joint_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::Vector3>(
          "control/single_joint_command", 1,
          [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            if (auto c =
                    std::dynamic_pointer_cast<JointController>(leg_controller_))
              c->updateSingleJointCommand(msg);
          });
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        const double a = cmd_vel_filter_const_, s = cmd_vel_scale_;
        cmd_vel_[0] = (1 - a) * cmd_vel_[0] + a * s * msg->linear.x;
        cmd_vel_[1] = (1 - a) * cmd_vel_[1] + a * s * msg->linear.y;
        cmd_vel_[2] = 0; cmd_vel_[3] = 0; cmd_vel_[4] = 0;
        cmd_vel_[5] = (1 - a) * cmd_vel_[5] + a * s * msg->angular.z;
#ifdef HAS_ONNXRUNTIME
        if (auto c = std::dynamic_pointer_cast<LearnedPolicy>(leg_controller_)) {
          rclcpp::Time t = get_node()->now();
          c->updateCmdVelMsg(cmd_vel_, t);
        }
#endif
      });
  imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
#ifdef HAS_ONNXRUNTIME
        if (auto c = std::dynamic_pointer_cast<LearnedPolicy>(leg_controller_))
          c->updateImuMsg(*msg);
#else
        (void)msg;
#endif
      });

  RCLCPP_INFO(get_node()->get_logger(),
              "LegControlController configured (controller=%s, mode=%s)",
              controller_id_.c_str(), interface_mode_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

bool LegControlController::computeCommand(quad_msgs::msg::LegCommandArray& out) {
  quad_msgs::msg::GRFArray grf;

  auto standFallback = [&]() {
    for (int i = 0; i < num_feet_; ++i) {
      out.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
        loadMotorCommandMsg(stand_joint_angles_.at(j), 0, 0, stand_kp_.at(j),
                            stand_kd_.at(j),
                            out.leg_commands.at(i).motor_commands.at(j));
    }
  };

  // Prime QuadKD2's Pinocchio data (M, N, J) with the CURRENT state before the
  // law runs. computeInverseDynamics / getJacobianBodyAngVel assume updated_ ==
  // true; without this they run on stale/default config -> singular SVD -> NaN
  // -> tau zeroed -> no stance support -> the robot drops. This is exactly
  // robot_driver::testDynamics(), which it calls every control cycle.
  if (last_robot_state_msg_ && quadKD_)
    quad_utils::updateDynamics(*quadKD_, *last_robot_state_msg_);

  if (!last_robot_state_msg_ || !leg_controller_ ||
      !leg_controller_->computeLegCommandArray(*last_robot_state_msg_, out,
                                               grf)) {
    standFallback();
    return false;
  }
  return true;
}

}  // namespace quad_controllers

PLUGINLIB_EXPORT_CLASS(quad_controllers::LegControlController,
                       controller_interface::ControllerInterface)
