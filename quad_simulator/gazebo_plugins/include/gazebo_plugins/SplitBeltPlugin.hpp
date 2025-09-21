#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

// Joint velocity components only
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Joint.hh>

namespace splitbelt_plugins
{
  // A Gazebo Sim System plugin that drives two prismatic joints by velocity
  class SplitBeltPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
  public:
    SplitBeltPlugin();
    ~SplitBeltPlugin() override;

    // Read SDF, resolve joints, setup ROS
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;

    // Write JointVelocityCmd every sim step
    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override;

  private:
    // ROS helpers
    void initRosIfNeeded();
    void leftCmdCb(const std_msgs::msg::Float64::SharedPtr msg);
    void rightCmdCb(const std_msgs::msg::Float64::SharedPtr msg);

  private:
    // Model & joints
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity leftJoint_{gz::sim::kNullEntity};
    gz::sim::Entity rightJoint_{gz::sim::kNullEntity};

    // Names configurable via SDF
    std::string leftJointName_{"left_belt_joint"};
    std::string rightJointName_{"right_belt_joint"};

    // Initial velocities (m/s)
    double leftVelInit_{0.0};
    double rightVelInit_{0.0};

    // Target velocities updated by ROS callbacks
    std::atomic<double> leftVelCmd_{0.0};
    std::atomic<double> rightVelCmd_{0.0};

    // ROS2 bits
    bool rosInitialized_{false};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightSub_;
    std::thread rosSpinThread_;
  };
} // namespace splitbelt_plugins
