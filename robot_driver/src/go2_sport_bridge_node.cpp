// go2_sport_bridge_node: drive the Go2's stock (Unitree sport mode) controller
// from the same teleop topics the Quad-SDK learned policies listen to.
//
// Subscribes cmd_vel (geometry_msgs/Twist) and control/mode (std_msgs/UInt8)
// on the Quad-SDK domain and forwards them through unitree_sdk2's SportClient,
// which publishes rt/api/sport/request on the Unitree domain (0) over the MCU
// Ethernet. The velocity goes through exactly the shaping a learned policy
// sees, so both controllers are handed the same reference:
//   robot_driver: v = (1 - a) * v + a * cmd_vel_scale * cmd_vel   (per msg)
//   policy:       clip(v, vel_min, vel_max)                       (trained box)
//
// control/mode follows robot_driver: 1 (stand) -> RecoveryStand and arm Move,
// 0 (sit) -> StandDown, 4 (safety) -> Damp. Move is only sent while armed.
//
// Do NOT run alongside robot_driver: it releases sport mode on startup.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class Go2SportBridge {
 public:
  explicit Go2SportBridge(rclcpp::Node::SharedPtr node) : node_(node) {
    // Defaults mirror robot_driver.yaml and the clip box hardcoded in
    // learned_velocity_policy.cpp / underbrush_policy.cpp. The launch file
    // passes robot_driver.yaml's values so the two cannot silently diverge.
    cmd_vel_filter_const_ =
        node_->declare_parameter("cmd_vel_filter_const", 0.10);
    cmd_vel_scale_ = node_->declare_parameter("cmd_vel_scale", 1.0);
    vel_min_ = node_->declare_parameter("vel_min",
                                        std::vector<double>{-1.0, -0.4, -1.0});
    vel_max_ = node_->declare_parameter("vel_max",
                                        std::vector<double>{1.0, 0.4, 1.0});
    const double rate = node_->declare_parameter("move_rate", 50.0);
    cmd_timeout_ = node_->declare_parameter("cmd_timeout", 0.5);
    // Motion service to select if none is active (e.g. robot_driver ran since
    // boot and released it). Empty = refuse to start instead.
    const std::string select_mode =
        node_->declare_parameter("select_mode", std::string(""));
    const char* env_iface = std::getenv("ROBOT_MCU_IFACE");
    const std::string net_iface = node_->declare_parameter(
        "net_iface", std::string((env_iface && *env_iface) ? env_iface : "eth0"));
    const std::string cmd_vel_topic =
        node_->declare_parameter("topics.cmd_vel", std::string("cmd_vel"));
    const std::string control_mode_topic = node_->declare_parameter(
        "topics.control.mode", std::string("control/mode"));

    if (vel_min_.size() != 3 || vel_max_.size() != 3) {
      throw std::runtime_error("vel_min / vel_max must be [vx, vy, wz]");
    }

    RCLCPP_INFO(node_->get_logger(),
                "Unitree DDS on '%s'; filter a=%.3f scale=%.2f; clip "
                "vx[%.2f,%.2f] vy[%.2f,%.2f] wz[%.2f,%.2f]",
                net_iface.c_str(), cmd_vel_filter_const_, cmd_vel_scale_,
                vel_min_[0], vel_max_[0], vel_min_[1], vel_max_[1],
                vel_min_[2], vel_max_[2]);
    unitree::robot::ChannelFactory::Instance()->Init(0, net_iface);

    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(5.0f);
    msc.Init();
    std::string form, name;
    int32_t ret = msc.CheckMode(form, name);
    if (ret != 0) {
      throw std::runtime_error("MotionSwitcher CheckMode failed (" +
                               std::to_string(ret) +
                               "); is the Go2 reachable on " + net_iface + "?");
    }
    if (name.empty()) {
      if (select_mode.empty()) {
        throw std::runtime_error(
            "No motion service active (sport mode was released, likely by "
            "robot_driver). Reboot the Go2 or pass select_mode:=<name>.");
      }
      RCLCPP_WARN(node_->get_logger(), "No motion service active; selecting '%s'",
                  select_mode.c_str());
      ret = msc.SelectMode(select_mode);
      if (ret != 0) {
        throw std::runtime_error("SelectMode('" + select_mode + "') failed (" +
                                 std::to_string(ret) + ")");
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "Active motion service: %s",
                  name.c_str());
    }

    // Constructed only now: SportClient opens its DDS channels in its ctor,
    // which needs ChannelFactory::Init to have run.
    sport_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_->SetTimeout(1.0f);
    sport_->Init();

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10,
        std::bind(&Go2SportBridge::cmdVelCallback, this, std::placeholders::_1));
    control_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        control_mode_topic, 1,
        std::bind(&Go2SportBridge::controlModeCallback, this,
                  std::placeholders::_1));
    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&Go2SportBridge::moveTimerCallback, this));

    RCLCPP_INFO(node_->get_logger(),
                "Ready. Press stand to arm Move; sit = StandDown, safety = Damp.");
  }

  ~Go2SportBridge() { sport_->StopMove(); }

 private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Same per-message low-pass as RobotDriver::cmdVelCallback.
    const double a = cmd_vel_filter_const_;
    vel_[0] = (1 - a) * vel_[0] + a * cmd_vel_scale_ * msg->linear.x;
    vel_[1] = (1 - a) * vel_[1] + a * cmd_vel_scale_ * msg->linear.y;
    vel_[2] = (1 - a) * vel_[2] + a * cmd_vel_scale_ * msg->angular.z;
    last_cmd_vel_time_ = node_->now();
    have_cmd_vel_ = true;
  }

  void controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    // teleop_twist_joy republishes the mode on every Joy message while the
    // button is held; act once per press.
    const rclcpp::Time t_now = node_->now();
    const bool new_press = msg->data != last_mode_ ||
                           (t_now - last_mode_time_).seconds() > 0.3;
    last_mode_ = msg->data;
    last_mode_time_ = t_now;
    if (!new_press) return;

    int32_t ret = 0;
    switch (msg->data) {
      case kReady:
        ret = sport_->RecoveryStand();
        armed_ = true;
        RCLCPP_INFO(node_->get_logger(), "stand -> RecoveryStand (%d), Move armed",
                    ret);
        break;
      case kSit:
        armed_ = false;
        sport_->StopMove();
        ret = sport_->StandDown();
        RCLCPP_INFO(node_->get_logger(), "sit -> StandDown (%d)", ret);
        break;
      case kSafety:
        armed_ = false;
        ret = sport_->Damp();
        RCLCPP_WARN(node_->get_logger(), "safety -> Damp (%d)", ret);
        break;
      default:
        break;
    }
  }

  void moveTimerCallback() {
    if (!armed_) return;

    const bool stale =
        !have_cmd_vel_ ||
        (node_->now() - last_cmd_vel_time_).seconds() > cmd_timeout_;
    if (stale) {
      if (!stopped_) {
        sport_->StopMove();
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        stopped_ = true;
        RCLCPP_WARN(node_->get_logger(),
                    "cmd_vel stale (> %.2f s); StopMove sent", cmd_timeout_);
      }
      return;
    }
    stopped_ = false;

    float v[3];
    for (int i = 0; i < 3; i++) {
      v[i] = static_cast<float>(std::clamp(vel_[i], vel_min_[i], vel_max_[i]));
    }
    const int32_t ret = sport_->Move(v[0], v[1], v[2]);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Move(%.2f, %.2f, %.2f) returned %d", v[0], v[1],
                           v[2], ret);
    }
  }

  // control/mode values, as in robot_driver.h
  static constexpr uint8_t kSit = 0;
  static constexpr uint8_t kReady = 1;
  static constexpr uint8_t kSafety = 4;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<unitree::robot::go2::SportClient> sport_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double cmd_vel_filter_const_, cmd_vel_scale_, cmd_timeout_;
  std::vector<double> vel_min_, vel_max_;

  double vel_[3] = {0.0, 0.0, 0.0};  // filtered [vx, vy, wz]
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mode_time_{0, 0, RCL_ROS_TIME};
  uint8_t last_mode_ = 255;
  bool have_cmd_vel_ = false;
  bool armed_ = false;
  bool stopped_ = true;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("go2_sport_bridge");
  int status = 0;
  try {
    Go2SportBridge bridge(node);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  // Same static-teardown crash workaround as robot_driver_node (links the
  // same library).
  std::_Exit(status);
}
