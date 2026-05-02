#ifndef GAZEBO_QUAD_WHEEL_ESTIMATOR_PLUGIN
#define GAZEBO_QUAD_WHEEL_ESTIMATOR_PLUGIN

#ifdef LOG
#undef LOG
#endif

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

#include <array>
#include <memory>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace gz_plugins {

//! Ground truth estimator for wheeled quadrupeds (Go2-W and similar).
/*!
   QuadWheelGroundTruthEstimator mirrors GroundTruthEstimator's body and
   leg state extraction, then additionally publishes the four wheel
   joints (continuous, axis y) into RobotState.joints. The published
   joint vector therefore has 16 entries: 12 leg motors (in the same
   per-leg order as GroundTruthEstimator) followed by 4 wheel motors
   (one per leg).

   Wheel joint names are pulled from the robot yaml:
       leg_i.joints.wheel.name      - wheel motor (NEW)
*/
class QuadWheelGroundTruthEstimator : public gz::sim::System,
                                      public gz::sim::ISystemConfigure,
                                      public gz::sim::ISystemPostUpdate {
 public:
  QuadWheelGroundTruthEstimator() = default;
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

 private:
  static constexpr int kNumLegs = 4;
  static constexpr int kJointsPerLeg = 3;
  static constexpr int kNumLegJoints = kNumLegs * kJointsPerLeg;
  static constexpr int kNumWheels = kNumLegs;
  static constexpr int kNumJoints = kNumLegJoints + kNumWheels;

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

  std::string body_frame_name_{"body"};
  std::array<std::string, kNumLegs> lower_frame_names_{};
  std::array<std::string, kNumLegs> toe_frame_names_{};

  // joint_names_ layout: 12 leg joints in [abad, hip, knee] order per leg
  // (leg-major), followed by 4 wheel joints in leg order.
  std::vector<std::string> joint_names_;

  mutable std::chrono::steady_clock::duration last_time_;
};
}  // namespace gz_plugins

#endif  // GAZEBO_QUAD_WHEEL_ESTIMATOR_PLUGIN
