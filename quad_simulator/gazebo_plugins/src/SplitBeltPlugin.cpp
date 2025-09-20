#include "gazebo_plugins/SplitBeltPlugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

using namespace splitbelt_plugins;
using gz::sim::Entity;
using gz::sim::EntityComponentManager;

SplitBeltPlugin::SplitBeltPlugin() = default;

SplitBeltPlugin::~SplitBeltPlugin()
{
    if (rosSpinThread_.joinable())
    {
        rclcpp::shutdown();
        rosSpinThread_.join();
    }
}

void SplitBeltPlugin::Configure(const Entity &entity,
                                const std::shared_ptr<const sdf::Element> &sdf,
                                EntityComponentManager &ecm,
                                gz::sim::EventManager &){
    this->model_ = gz::sim::Model(entity);

    // 1) 从 SDF 读取参数
    if (sdf && sdf->HasElement("left_link_name"))
        this->leftLinkName_ = sdf->Get<std::string>("left_link_name");
    if (sdf && sdf->HasElement("right_link_name"))
        this->rightLinkName_ = sdf->Get<std::string>("right_link_name");
    if (sdf && sdf->HasElement("left_vel_init"))
        this->leftVelInit_ = sdf->Get<double>("left_vel_init");
    if (sdf && sdf->HasElement("right_vel_init"))
        this->rightVelInit_ = sdf->Get<double>("right_vel_init");

    this->leftVelCmd_.store(this->leftVelInit_);
    this->rightVelCmd_.store(this->rightVelInit_);

    // 通过 link 名称查实体
    this->leftLink_  = this->model_.LinkByName(ecm, this->leftLinkName_);
    this->rightLink_ = this->model_.LinkByName(ecm, this->rightLinkName_);

    if (this->leftLink_ == gz::sim::kNullEntity)
        gzerr << "[SplitBeltPlugin] Cannot find left link: " << this->leftLinkName_ << "\n";
    if (this->rightLink_ == gz::sim::kNullEntity)
        gzerr << "[SplitBeltPlugin] Cannot find right link: " << this->rightLinkName_ << "\n";

    // 确保有线/角速度状态组件（可选，不加也行；仿真会自己补）
    if (this->leftLink_ != gz::sim::kNullEntity){
        if (!ecm.Component<gz::sim::components::LinearVelocity>(this->leftLink_))
            ecm.CreateComponent(this->leftLink_, gz::sim::components::LinearVelocity());
        if (!ecm.Component<gz::sim::components::AngularVelocity>(this->leftLink_))
            ecm.CreateComponent(this->leftLink_, gz::sim::components::AngularVelocity());
    }
    if (this->rightLink_ != gz::sim::kNullEntity){
        if (!ecm.Component<gz::sim::components::LinearVelocity>(this->rightLink_))
            ecm.CreateComponent(this->rightLink_, gz::sim::components::LinearVelocity());
        if (!ecm.Component<gz::sim::components::AngularVelocity>(this->rightLink_))
            ecm.CreateComponent(this->rightLink_, gz::sim::components::AngularVelocity());
    }

    // 4) 初始化 ROS 订阅与自转线程
    this->initRosIfNeeded();
}

void SplitBeltPlugin::PreUpdate(const gz::sim::UpdateInfo &info,
                                EntityComponentManager &ecm)
{
    if (info.paused)
        return;

    const double vL = this->leftVelCmd_.load();
    const double vR = this->rightVelCmd_.load();

    // 给 left link 写线速度命令，并把角速度置 0
    if (this->leftLink_ != gz::sim::kNullEntity){
        auto *lv = ecm.Component<gz::sim::components::LinearVelocityCmd>(this->leftLink_);
        if (!lv)
            ecm.CreateComponent(this->leftLink_, gz::sim::components::LinearVelocityCmd({vL, 0.0, 0.0}));
        else
            (*lv) = gz::sim::components::LinearVelocityCmd({vL, 0.0, 0.0});
        auto *av = ecm.Component<gz::sim::components::AngularVelocityCmd>(this->leftLink_);
        if (!av)
            ecm.CreateComponent(this->leftLink_, gz::sim::components::AngularVelocityCmd({0.0, 0.0, 0.0}));
        else
            (*av) = gz::sim::components::AngularVelocityCmd({0.0, 0.0, 0.0});
    }

    // 给 right link 写线速度命令，并把角速度置 0
    if (this->rightLink_ != gz::sim::kNullEntity){
        auto *rv = ecm.Component<gz::sim::components::LinearVelocityCmd>(this->rightLink_);
        if(!rv)
            ecm.CreateComponent(this->rightLink_, gz::sim::components::LinearVelocityCmd({vR, 0.0, 0.0}));
        else
            (*rv) = gz::sim::components::LinearVelocityCmd({vR, 0.0, 0.0});
        auto *av = ecm.Component<gz::sim::components::AngularVelocityCmd>(this->rightLink_);
        if(!av)
            ecm.CreateComponent(this->rightLink_, gz::sim::components::AngularVelocityCmd({0.0, 0.0, 0.0}));
        else
            (*av) = gz::sim::components::AngularVelocityCmd({0.0, 0.0, 0.0});
    }
}

/*** ROS2 ***/
void SplitBeltPlugin::initRosIfNeeded()
{
    // 1) 幂等保护：只初始化一次
    if (this->rosInitialized_)
        return;

    // 2) 初始化 rclcpp（若还没 init）
    int argc = 0;
    char **argv = nullptr;
    if (!rclcpp::ok())
        rclcpp::init(argc, argv);

    // 3) 创建节点
    this->node_ = std::make_shared<rclcpp::Node>("splitbelt_plugin");

    // 4) 建立订阅：左带速度
    this->leftSub_ = this->node_->create_subscription<std_msgs::msg::Float64>(
        "/left_belt/cmd_vel", 10,
        std::bind(&SplitBeltPlugin::leftCmdCb, this, std::placeholders::_1));

    // 5) 建立订阅：右带速度
    this->rightSub_ = this->node_->create_subscription<std_msgs::msg::Float64>(
        "/right_belt/cmd_vel", 10,
        std::bind(&SplitBeltPlugin::rightCmdCb, this, std::placeholders::_1));

    // 6) 独立线程运行 Executor，处理 ROS 回调
    this->rosSpinThread_ = std::thread([this]() {
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(this->node_);
        exec.spin();
    });

    // 7) 标记已初始化
    this->rosInitialized_ = true;
}

void SplitBeltPlugin::leftCmdCb(const std_msgs::msg::Float64::SharedPtr msg){
    this->leftVelCmd_.store(msg->data);
}

void SplitBeltPlugin::rightCmdCb(const std_msgs::msg::Float64::SharedPtr msg){
    this->rightVelCmd_.store(msg->data);
}

GZ_ADD_PLUGIN(
    splitbelt_plugins::SplitBeltPlugin,
    gz::sim::System,
    splitbelt_plugins::SplitBeltPlugin::ISystemConfigure,
    splitbelt_plugins::SplitBeltPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(splitbelt_plugins::SplitBeltPlugin, "splitbelt_plugin")