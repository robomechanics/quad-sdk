#ifndef GAZEBO_SPIRIT_ESTIMATOR_PLUGIN
#define GAZEBO_SPIRIT_ESTIMATOR_PLUGIN

#ifdef LOG
#undef LOG
#endif

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <memory>
#include <array>
#include <quad_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <quad_utils/ros_utils.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/quad_kd2.hpp>
#include <std_msgs/msg/string.hpp>

namespace gz_plugins {
class GroundTruthEstimator : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPostUpdate {
 public:
  GroundTruthEstimator() = default;
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

 private:
  gz::sim::Model model_;
  gz::sim::Entity entity_;

  rclcpp::Node::SharedPtr node_;

  double update_rate_{500.0};

  std::string ground_truth_state_topic_;
  std::string ground_truth_body_frame_topic_;

  bool urdf_received_ = false;
  mutable bool time_initialized_ = false;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;

  mutable rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr
      ground_truth_state_pub_;
  mutable rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr
      ground_truth_state_body_frame_pub_;

  mutable std::shared_ptr<quad_utils::QuadKD2> quadKD_;
  std::string body_frame_name_{"body"};
  std::array<std::string, 4> lower_frame_names_{};
  std::array<std::string, 4> toe_frame_names_{};
  std::vector<std::string> joint_names_;

  mutable std::chrono::steady_clock::duration last_time_;
};
}  // namespace gz_plugins

#endif  // GAZEBO_SPIRIT_ESTIMATOR_PLUGIN
