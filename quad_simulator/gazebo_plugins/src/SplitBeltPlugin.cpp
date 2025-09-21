#include "gazebo_plugins/SplitBeltPlugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

using namespace splitbelt_plugins;
using gz::sim::Entity;
using gz::sim::EntityComponentManager;

SplitBeltPlugin::SplitBeltPlugin() = default;

SplitBeltPlugin::~SplitBeltPlugin()
{
  // Ensure ROS thread stops before destruction
  if (rosSpinThread_.joinable())
  {
    rclcpp::shutdown();
    rosSpinThread_.join();
  }
}

void SplitBeltPlugin::Configure(const Entity &entity,
                                const std::shared_ptr<const sdf::Element> &sdf,
                                EntityComponentManager &ecm,
                                gz::sim::EventManager &)
{
  this->model_ = gz::sim::Model(entity);

  // --- Read SDF parameters (joint names and initial velocities)
  if (sdf && sdf->HasElement("left_joint_name"))
    this->leftJointName_ = sdf->Get<std::string>("left_joint_name");
  if (sdf && sdf->HasElement("right_joint_name"))
    this->rightJointName_ = sdf->Get<std::string>("right_joint_name");
  if (sdf && sdf->HasElement("left_vel_init"))
    this->leftVelInit_ = sdf->Get<double>("left_vel_init");
  if (sdf && sdf->HasElement("right_vel_init"))
    this->rightVelInit_ = sdf->Get<double>("right_vel_init");

  this->leftVelCmd_.store(this->leftVelInit_);
  this->rightVelCmd_.store(this->rightVelInit_);

  // --- Resolve prismatic joints by name
  this->leftJoint_  = this->model_.JointByName(ecm, this->leftJointName_);
  this->rightJoint_ = this->model_.JointByName(ecm, this->rightJointName_);

  if (this->leftJoint_ == gz::sim::kNullEntity)
    gzerr << "[SplitBeltPlugin] Cannot find left joint: " << this->leftJointName_ << "\n";
  if (this->rightJoint_ == gz::sim::kNullEntity)
    gzerr << "[SplitBeltPlugin] Cannot find right joint: " << this->rightJointName_ << "\n";

  // --- Ensure JointVelocity (state) components exist on both joints
  if (this->leftJoint_ != gz::sim::kNullEntity)
  {
    if (!ecm.Component<gz::sim::components::JointVelocity>(this->leftJoint_))
      ecm.CreateComponent(this->leftJoint_, gz::sim::components::JointVelocity({0.0}));
  }
  if (this->rightJoint_ != gz::sim::kNullEntity)
  {
    if (!ecm.Component<gz::sim::components::JointVelocity>(this->rightJoint_))
      ecm.CreateComponent(this->rightJoint_, gz::sim::components::JointVelocity({0.0}));
  }

  // --- Init ROS subscribers
  this->initRosIfNeeded();

  // Debug hint
  gzdbg << "[SplitBeltPlugin] Ready. LeftJoint=" << this->leftJointName_
        << " RightJoint=" << this->rightJointName_
        << " vInit(L/R)=(" << this->leftVelInit_ << ", " << this->rightVelInit_ << ")"
        << std::endl;
}

void SplitBeltPlugin::PreUpdate(const gz::sim::UpdateInfo &info,
                                EntityComponentManager &ecm)
{
  // Do nothing while paused
  if (info.paused)
    return;

  const double vL = this->leftVelCmd_.load();
  const double vR = this->rightVelCmd_.load();

  // --- Drive joints by velocity command (1-DOF prismatic => vector size 1)
  if (this->leftJoint_ != gz::sim::kNullEntity)
  {
    auto *cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(this->leftJoint_);
    if (!cmd)
      ecm.CreateComponent(this->leftJoint_, gz::sim::components::JointVelocityCmd({vL}));
    else
      (*cmd) = gz::sim::components::JointVelocityCmd({vL});
  }

  if (this->rightJoint_ != gz::sim::kNullEntity)
  {
    auto *cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(this->rightJoint_);
    if (!cmd)
      ecm.CreateComponent(this->rightJoint_, gz::sim::components::JointVelocityCmd({vR}));
    else
      (*cmd) = gz::sim::components::JointVelocityCmd({vR});
  }
}

/*** ROS2 ***/
void SplitBeltPlugin::initRosIfNeeded()
{
  // Idempotence guard
  if (this->rosInitialized_)
    return;

  // rclcpp init if needed
  int argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok())
    rclcpp::init(argc, argv);

  // Create node
  this->node_ = std::make_shared<rclcpp::Node>("splitbelt_plugin");

  // Subscriptions
  this->leftSub_ = this->node_->create_subscription<std_msgs::msg::Float64>(
      "/left_belt/cmd_vel", 10,
      std::bind(&SplitBeltPlugin::leftCmdCb, this, std::placeholders::_1));

  this->rightSub_ = this->node_->create_subscription<std_msgs::msg::Float64>(
      "/right_belt/cmd_vel", 10,
      std::bind(&SplitBeltPlugin::rightCmdCb, this, std::placeholders::_1));

  // Spin in a separate thread
  this->rosSpinThread_ = std::thread([this]()
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(this->node_);
    exec.spin();
  });

  this->rosInitialized_ = true;
}

void SplitBeltPlugin::leftCmdCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  // Atomic store for thread-safe handoff to simulation thread
  this->leftVelCmd_.store(msg->data);
}

void SplitBeltPlugin::rightCmdCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  // Atomic store for thread-safe handoff to simulation thread
  this->rightVelCmd_.store(msg->data);
}

// Register plugin and aliases (so SDF can use either name)
GZ_ADD_PLUGIN(
  splitbelt_plugins::SplitBeltPlugin,
  gz::sim::System,
  splitbelt_plugins::SplitBeltPlugin::ISystemConfigure,
  splitbelt_plugins::SplitBeltPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(splitbelt_plugins::SplitBeltPlugin, "splitbelt_plugin")
GZ_ADD_PLUGIN_ALIAS(splitbelt_plugins::SplitBeltPlugin, "splitbelt_plugins::SplitBeltPlugin")
