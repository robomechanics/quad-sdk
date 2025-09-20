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
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Link.hh>

namespace splitbelt_plugins
{
    // 一个 System 插件，同时实现 Configure 与 PreUpdate
    class SplitBeltPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    public:
        SplitBeltPlugin();
        ~SplitBeltPlugin() override;

        // 从 SDF 读参数，解析模型，找到关节
        void Configure(const gz::sim::Entity &entity,
                    const std::shared_ptr<const sdf::Element> &sdf,
                    gz::sim::EntityComponentManager &ecm,
                    gz::sim::EventManager &eventMgr) override;

        // 每步写关节速度
        void PreUpdate(const gz::sim::UpdateInfo &info,
                    gz::sim::EntityComponentManager &ecm) override;

    private:
        void initRosIfNeeded();
        void leftCmdCb(const std_msgs::msg::Float64::SharedPtr msg);
        void rightCmdCb(const std_msgs::msg::Float64::SharedPtr msg);

    private:
        // 模型与实体
        gz::sim::Model model_{gz::sim::kNullEntity};
        gz::sim::Entity leftLink_{gz::sim::kNullEntity};
        gz::sim::Entity rightLink_{gz::sim::kNullEntity};

        // 参数（可由 SDF 配置）
        std::string leftLinkName_{"left_belt_moving"};
        std::string rightLinkName_{"right_belt_moving"};
        double leftVelInit_{0.0};
        double rightVelInit_{0.0};

        // 目标速度（可被 ROS2 回调更新）
        std::atomic<double> leftVelCmd_{0.0};
        std::atomic<double> rightVelCmd_{0.0};

        // ROS2
        bool rosInitialized_{false};
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftSub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightSub_;
        std::thread rosSpinThread_;
        };

} // namespace splitbelt_plugins