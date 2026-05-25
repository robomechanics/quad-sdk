// Mode supervisor for the ros2_control quad-sdk stack (Option B).
//
// Faithful port of robot_driver's control-mode FSM (controlModeCallback +
// checkMessagesForSafety + the SIT_TO_READY/READY_TO_SIT auto-advance), but it
// realizes each mode by switching ros2_control controllers via
// controller_manager's switch_controller service:
//   SIT          -> sit_controller        (PoseController, sit pose/gains)
//   SIT_TO_READY -> sit_to_ready_controller (TransitionController sit->stand)
//   READY        -> locomotion_controller (LegControlController, selected law)
//   READY_TO_SIT -> ready_to_sit_controller (TransitionController stand->sit)
//   SAFETY       -> safety_controller      (pos-0 PD, safety gains)
// Transitions auto-advance after transition_duration, exactly like robot_driver.

#include <limits>
#include <memory>
#include <string>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "quad_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int8.hpp"

using SwitchController = controller_manager_msgs::srv::SwitchController;

class ModeSupervisor : public rclcpp::Node {
 public:
  ModeSupervisor() : rclcpp::Node("mode_supervisor") {
    const std::string cm = declare_parameter<std::string>(
        "controller_manager", "/controller_manager");
    sit_ctrl_ = declare_parameter<std::string>("sit_controller", "sit_controller");
    locomotion_ctrl_ = declare_parameter<std::string>(
        "locomotion_controller", "locomotion_controller");
    safety_ctrl_ =
        declare_parameter<std::string>("safety_controller", "safety_controller");
    sit_to_ready_ctrl_ = declare_parameter<std::string>(
        "sit_to_ready_controller", "sit_to_ready_controller");
    ready_to_sit_ctrl_ = declare_parameter<std::string>(
        "ready_to_sit_controller", "ready_to_sit_controller");
    transition_duration_ = declare_parameter<double>("transition_duration", 1.0);
    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout", 0.2);
    state_timeout_ = declare_parameter<double>("state_timeout", 0.1);
    is_hardware_ = declare_parameter<bool>("is_hardware", false);

    switch_client_ = create_client<SwitchController>(cm + "/switch_controller");

    mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "control/mode", 1,
        std::bind(&ModeSupervisor::controlModeCallback, this, std::placeholders::_1));
    heartbeat_sub_ = create_subscription<std_msgs::msg::Header>(
        "heartbeat", 1, [this](std_msgs::msg::Header::SharedPtr) {
          remote_heartbeat_received_time_ = now().seconds();
        });
    // Liveness watchdog for the sim-only state-timeout check below. Must track
    // state/ground_truth (the control state -- published by the estimator_plugin
    // in sim and the estimator node on hardware), NOT the debug-only
    // state/estimate, which is never published in sim.
    state_sub_ = create_subscription<quad_msgs::msg::RobotState>(
        "state/ground_truth", rclcpp::SensorDataQoS(),
        [this](quad_msgs::msg::RobotState::SharedPtr) {
          last_state_time_ = now().seconds();
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&ModeSupervisor::tick, this));

    // The launch spawns sit_controller ACTIVE, so the robot holds the sit pose
    // from spawn (robot_driver starts in SIT). Track it as the active controller
    // so the first mode change deactivates it. (switchTo also deactivates all
    // other mode controllers defensively, so this is robust to drift.)
    control_mode_ = SIT;
    active_ctrl_ = sit_ctrl_;

    RCLCPP_INFO(get_logger(), "Mode supervisor (Option B) up (cm=%s)", cm.c_str());
  }

 private:
  static constexpr int SIT = 0;
  static constexpr int READY = 1;
  static constexpr int SIT_TO_READY = 2;
  static constexpr int READY_TO_SIT = 3;
  static constexpr int SAFETY = 4;

  void switchTo(const std::string& controller) {
    if (controller.empty() || controller == active_ctrl_) return;
    if (!switch_client_->service_is_ready() &&
        !switch_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "switch_controller service unavailable");
      return;
    }
    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers = {controller};
    // Deactivate every OTHER mode controller — they all claim the same command
    // interfaces, so only one may be active. BEST_EFFORT tolerates ones that
    // aren't currently active, so this stays correct even if our active-state
    // tracking drifts (e.g. a controller activated by the spawner, not by us).
    for (const auto& c : {sit_ctrl_, locomotion_ctrl_, safety_ctrl_,
                          sit_to_ready_ctrl_, ready_to_sit_ctrl_}) {
      if (c != controller && !c.empty())
        req->deactivate_controllers.push_back(c);
    }
    req->strictness = SwitchController::Request::BEST_EFFORT;
    req->activate_asap = true;
    switch_client_->async_send_request(
        req, [this, controller](rclcpp::Client<SwitchController>::SharedFuture f) {
          if (f.get()->ok) {
            active_ctrl_ = controller;
          } else {
            RCLCPP_WARN(get_logger(), "switch_controller rejected: %s",
                        controller.c_str());
          }
        });
  }

  // Faithful port of RobotDriver::controlModeCallback.
  void controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    if (control_mode_ == SIT_TO_READY || control_mode_ == READY_TO_SIT) return;
    if (msg->data == READY && control_mode_ == SIT) {
      control_mode_ = SIT_TO_READY;
      transition_timestamp_ = now();
      switchTo(sit_to_ready_ctrl_);
    } else if (msg->data == SIT && control_mode_ == READY) {
      control_mode_ = READY_TO_SIT;
      transition_timestamp_ = now();
      switchTo(ready_to_sit_ctrl_);
    } else if (msg->data == SIT) {
      control_mode_ = SIT;
      switchTo(sit_ctrl_);
    } else if (msg->data == SAFETY) {
      control_mode_ = SAFETY;
      switchTo(safety_ctrl_);
    }
  }

  // Transition auto-advance + safety, run each tick.
  void tick() {
    if (control_mode_ == SIT_TO_READY &&
        (now() - transition_timestamp_).seconds() >= transition_duration_) {
      control_mode_ = READY;
      switchTo(locomotion_ctrl_);
    } else if (control_mode_ == READY_TO_SIT &&
               (now() - transition_timestamp_).seconds() >= transition_duration_) {
      control_mode_ = SIT;
      switchTo(sit_ctrl_);
    }
    checkMessagesForSafety();
  }

  // Faithful port of RobotDriver::checkMessagesForSafety.
  void checkMessagesForSafety() {
    if (control_mode_ == SAFETY) return;
    const double now_s = now().seconds();
    if (std::abs(now_s - remote_heartbeat_received_time_) >= heartbeat_timeout_ &&
        remote_heartbeat_received_time_ != std::numeric_limits<double>::max()) {
      control_mode_ = SAFETY;
      switchTo(safety_ctrl_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Remote heartbeat lost, entering safety mode");
    }
    if (!is_hardware_ &&
        std::abs(now_s - last_state_time_) >= state_timeout_ &&
        last_state_time_ != std::numeric_limits<double>::max()) {
      control_mode_ = SAFETY;
      switchTo(safety_ctrl_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "State messages lost, entering safety mode");
    }
  }

  std::string sit_ctrl_, locomotion_ctrl_, safety_ctrl_, sit_to_ready_ctrl_,
      ready_to_sit_ctrl_;
  double transition_duration_, heartbeat_timeout_, state_timeout_;
  bool is_hardware_{false};

  int control_mode_{SIT};
  rclcpp::Time transition_timestamp_;
  std::string active_ctrl_;
  double remote_heartbeat_received_time_{std::numeric_limits<double>::max()};
  double last_state_time_{std::numeric_limits<double>::max()};

  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeSupervisor>());
  rclcpp::shutdown();
  return 0;
}
