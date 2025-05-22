#include "gazebo_scripts/estimator_plugin.h"

namespace gazebo_plugins {
class GroundTruthEstimator : public ignition::gazebo::System,
                             public ignition::gazebo::ISystemConfigure,
                             public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &eventMgr) override {
    this->model = ignition::gazebo::Model(entity);
    this->node = std::make_shared<rclcpp::Node>("ground_truth_estimator");
    this->ros_pub = this->node->create_publisher<quad_msgs::msg::RobotState>(
        "/robot_1/state/ground_truth", 10);

    // Parse SDF update rate if present
    if (sdf->HasElement("updateRateHZ"))
      this->update_rate = sdf->Get<double>("updateRateHZ");

    RCLCPP_INFO(this->node->get_logger(),
                "Ground Truth Estimator initialized with rate %.1f Hz",
                this->update_rate);
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                 ignition::gazebo::EntityComponentManager &ecm) override {
    if (!this->node || !this->model.Valid(ecm)) return;

    double dt = (info.simTime - this->last_update_time).Double();
    if (this->update_rate > 0.0 && dt < (1.0 / this->update_rate)) return;

    this->last_update_time = info.simTime;

    auto bodyEntity = this->model.LinkByName(ecm, "body");

    if (!bodyEntity) {
      RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(),
                           2000, "Body link not found");
      return;
    }

    // Get pose and velocity
    auto pose = ignition::gazebo::components::WorldPose::Data(ecm, bodyEntity);
    auto lin_vel =
        ignition::gazebo::components::LinearVelocity::Data(ecm, bodyEntity);
    auto ang_vel =
        ignition::gazebo::components::AngularVelocity::Data(ecm, bodyEntity);

    quad_msgs::msg::RobotState msg;
    msg.header.stamp = this->node->get_clock()->now();

    msg.body.pose.position.x = pose.Pos().X();
    msg.body.pose.position.y = pose.Pos().Y();
    msg.body.pose.position.z = pose.Pos().Z();
    msg.body.pose.orientation.x = pose.Rot().X();
    msg.body.pose.orientation.y = pose.Rot().Y();
    msg.body.pose.orientation.z = pose.Rot().Z();
    msg.body.pose.orientation.w = pose.Rot().W();

    msg.body.twist.linear.x = lin_vel.X();
    msg.body.twist.linear.y = lin_vel.Y();
    msg.body.twist.linear.z = lin_vel.Z();

    msg.body.twist.angular.x = ang_vel.X();
    msg.body.twist.angular.y = ang_vel.Y();
    msg.body.twist.angular.z = ang_vel.Z();

    this->ros_pub->publish(msg);
  }

 private:
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr ros_pub;
  ignition::common::Time last_update_time{0};
  double update_rate{500.0};
};
}  // namespace gazebo_plugins

IGNITION_ADD_PLUGIN(gazebo_plugins::GroundTruthEstimator,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(gazebo_plugins::GroundTruthEstimator,
                          "ground_truth_estimator")
