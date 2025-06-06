#ifdef LOG
#undef LOG
#endif
#include "gazebo_plugins/estimator_plugin.h"

namespace gz_plugins {
void GroundTruthEstimator::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) {
  this->model_ = gz::sim::Model(entity);
  this->entity_ = entity;

  std::string robot_ns = this->model_.Name(ecm);
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  // options.arguments({"--ros-args", "--namespace", robot_ns});
  this->node_ = std::make_shared<rclcpp::Node>("gz_ground_truth_estimator",
                                               robot_ns, options);

  // Load update rate from SDF
  if (sdf->HasElement("updateRateHZ")) {
    this->update_rate_ = sdf->Get<double>("updateRateHZ");
    RCLCPP_INFO(this->node_->get_logger(),
                "Ground Truth Estimator initialized with rate %.1f Hz",
                this->update_rate_);
  } else {
    update_rate_ = 500.0;
    RCLCPP_WARN(this->node_->get_logger(),
                "Ground Truth State Estimator: missing "
                "<updateRateHZ>, set to default: %.1f",
                this->update_rate_);
  }

  // Set up publishers, attempt to load topics from SDF file
  std::string ground_truth_state_topic_ = "state/ground_truth";
  std::string ground_truth_body_frame_topic_ = "state/ground_truth_body_frame";
  if (sdf->HasElement("ground_truth_state_topic")) {
    this->ground_truth_state_topic_ =
        sdf->Get<std::string>("ground_truth_state_topic");
  }
  if (sdf->HasElement("ground_truth_body_frame_topic")) {
    this->ground_truth_body_frame_topic_ =
        sdf->Get<std::string>("ground_truth_body_frame_topic");
  }

  this->ground_truth_state_pub_ =
      this->node_->create_publisher<quad_msgs::msg::RobotState>(
          this->ground_truth_state_topic_, 10);
  this->ground_truth_state_body_frame_pub_ =
      this->node_->create_publisher<quad_msgs::msg::RobotState>(
          this->ground_truth_body_frame_topic_, 10);

  // Convert Kinematics, and initialize World Time
  std::string urdf_topic = "robot_description";
  rclcpp::QoS qos(10);
  qos.transient_local().reliable();
  this->urdf_sub_ = this->node_->create_subscription<std_msgs::msg::String>(
      urdf_topic, qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!this->urdf_received_) {
          this->urdf_received_ = true;
          RCLCPP_INFO(this->node_->get_logger(), "Inside Callback.");

          this->node_->declare_parameter<std::string>("robot_description",
                                                      msg->data);

          RCLCPP_INFO(this->node_->get_logger(),
                      "Received and set robot_description parameter.");

          try {
            this->quadKD_ = std::make_shared<quad_utils::QuadKD>(this->node_);
            RCLCPP_INFO(this->node_->get_logger(), "Makes QuadKD Class.");
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->node_->get_logger(), "QuadKD init failed: %s",
                         e.what());
          }
        }
      });

  std::vector<std::string> links_to_check = {"body", "toe0", "toe1", "toe2",
                                             "toe3"};

  for (const auto &link_name : links_to_check) {
    auto link_entity = this->model_.LinkByName(ecm, link_name);
    if (link_entity == gz::sim::kNullEntity) continue;

    if (!ecm.EntityHasComponentType(
            link_entity, gz::sim::components::WorldLinearVelocity::typeId)) {
      ecm.CreateComponent(link_entity,
                          gz::sim::components::WorldLinearVelocity());
    }

    if (!ecm.EntityHasComponentType(
            link_entity, gz::sim::components::AngularVelocity::typeId)) {
      ecm.CreateComponent(link_entity, gz::sim::components::AngularVelocity());
    }

    if (!ecm.EntityHasComponentType(link_entity,
                                    gz::sim::components::Pose::typeId)) {
      ecm.CreateComponent(link_entity, gz::sim::components::Pose());
    }
  }

  std::vector<std::string> joint_names = {"8",  "0", "1", "9",  "2", "3",
                                          "10", "4", "5", "11", "6", "7"};

  for (const auto &joint_name : joint_names) {
    auto joint_entity = this->model_.JointByName(ecm, joint_name);
    if (joint_entity == gz::sim::kNullEntity) continue;

    if (!ecm.EntityHasComponentType(
            joint_entity, gz::sim::components::JointPosition::typeId)) {
      ecm.CreateComponent(joint_entity, gz::sim::components::JointPosition());
    }

    if (!ecm.EntityHasComponentType(
            joint_entity, gz::sim::components::JointVelocity::typeId)) {
      ecm.CreateComponent(joint_entity, gz::sim::components::JointVelocity());
    }
  }

  for (const auto &joint_name : joint_names) {
    auto joint_entity = this->model_.JointByName(ecm, joint_name);
    if (joint_entity != gz::sim::kNullEntity) {
      gz::sim::Joint joint(joint_entity);
      joint.EnableVelocityCheck(ecm, true);
      joint.EnablePositionCheck(ecm, true);
      joint.EnableTransmittedWrenchCheck(ecm, true);
    }
  }
}

void GroundTruthEstimator::PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) {

  rclcpp::spin_some(this->node_);
  if (!this->node_ || !this->model_.Valid(ecm)) return;

  if (!urdf_received_) return;

  if (!this->time_initialized_) {
    this->last_time_ = info.simTime;
    this->time_initialized_ = true;
    RCLCPP_INFO(this->node_->get_logger(), "Initializing last_time_ to %.6f",
                std::chrono::duration<double>(last_time_).count());
    return;
  }

  double dt = std::chrono::duration<double>(info.simTime - last_time_).count();
  if (this->update_rate_ > 0.0 && dt < (1.0 / this->update_rate_)) return;
  this->last_time_ = info.simTime;

  // Extract all relevant information from the simulator
  auto body_entity = this->model_.LinkByName(ecm, "body");

  if (body_entity == gz::sim::kNullEntity) {
    RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(),
                         2000,
                         "Can't find body link in sdf. Make sure the name in "
                         "the plugin matches the sdf.");
    return;
  }

  auto lower0_entity = this->model_.LinkByName(ecm, "lower0");
  auto lower1_entity = this->model_.LinkByName(ecm, "lower1");
  auto lower2_entity = this->model_.LinkByName(ecm, "lower2");
  auto lower3_entity = this->model_.LinkByName(ecm, "lower3");

  auto toe0_entity = this->model_.LinkByName(ecm, "toe0");
  auto toe1_entity = this->model_.LinkByName(ecm, "toe1");
  auto toe2_entity = this->model_.LinkByName(ecm, "toe2");
  auto toe3_entity = this->model_.LinkByName(ecm, "toe3");

  gz::sim::Link body_link(body_entity);

  gz::sim::Link lower0(lower0_entity);
  gz::sim::Link lower1(lower1_entity);
  gz::sim::Link lower2(lower2_entity);
  gz::sim::Link lower3(lower3_entity);

  gz::sim::Link toe0(toe0_entity);
  gz::sim::Link toe1(toe1_entity);
  gz::sim::Link toe2(toe2_entity);
  gz::sim::Link toe3(toe3_entity);

  auto pose_opt = body_link.WorldPose(ecm);
  auto lin_vel_opt = body_link.WorldLinearVelocity(ecm);
  auto ang_vel_opt = body_link.WorldAngularVelocity(ecm);

  const auto &pose    = *pose_opt;
  const auto &lin_vel = *lin_vel_opt;
  const auto &ang_vel = *ang_vel_opt;

  // Update and publish state estimate message
  quad_msgs::msg::RobotState state;
  state.body.pose.position.x = pose.Pos().X();
  state.body.pose.position.y = pose.Pos().Y();
  state.body.pose.position.z = pose.Pos().Z();
  state.body.pose.orientation.w = pose.Rot().W();
  state.body.pose.orientation.x = pose.Rot().X();
  state.body.pose.orientation.y = pose.Rot().Y();
  state.body.pose.orientation.z = pose.Rot().Z();

  state.body.twist.linear.x = lin_vel.X();
  state.body.twist.linear.y = lin_vel.Y();
  state.body.twist.linear.z = lin_vel.Z();

  state.body.twist.angular.x = ang_vel.X();
  state.body.twist.angular.y = ang_vel.Y();
  state.body.twist.angular.z = ang_vel.Z();

  // Sanity Check Robot Spawn Pose
  // RCLCPP_INFO(this->node_->get_logger(),
  //             "Body Position: x=%.3f, y=%.3f, z=%.3f",
  //             state.body.pose.position.x, state.body.pose.position.y,
  //             state.body.pose.position.z);

  // Update the Joints
  int num_joints = 12;
  state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                       "10", "4", "5", "11", "6", "7"};

  for (int i = 0; i < num_joints; i++) {
    auto joint_entity = this->model_.JointByName(ecm, state.joints.name[i]);
    double pos = 0.0, vel = 0.0, torque = 0.0;
    if (joint_entity) {

      gz::sim::Joint joint(joint_entity);
      auto pos_opt = joint.Position(ecm);
      auto vel_opt = joint.Velocity(ecm);
      auto wrench_opt = joint.TransmittedWrench(ecm);

      if (pos_opt && !pos_opt->empty()) {
        pos = (*pos_opt)[0];
      }
      if ( vel_opt && !vel_opt->empty()) {
        vel = (*vel_opt)[0];
      }
      if (wrench_opt) {
        const auto &wrench_msg = (*wrench_opt)[0];
        const auto &torque_msg = wrench_msg.torque();

        // Interpret based on leg phase (same logic as Classic)
        switch (i % 3) {
          case 0:  // Abad
            torque = torque_msg.x();
            break;
          case 1:  // Hip
            torque = -torque_msg.y();
            break;
          case 2:  // Knee
            torque = torque_msg.y();
            break;
        }
      }
    }
    state.joints.position.push_back(pos);
    state.joints.velocity.push_back(vel);
    state.joints.effort.push_back(torque);
  }

  int num_feet = 4;
  state.feet.feet.resize(num_feet);
  quad_utils::fkRobotState(*this->quadKD_, state);

  // Update the Feet Positions and Velocities
  std::vector<gz::sim::Link> toes = {toe0, toe1, toe2, toe3};

  for (int i = 0; i < 4; i++) {
    auto toe_pose_opt = toes[i].WorldPose(ecm);
    auto toe_vel_opt = toes[i].WorldLinearVelocity(ecm);
    if (toe_pose_opt) {
      const auto &toe_pose = *toe_pose_opt;
      state.feet.feet[i].position.x = toe_pose.Pos().X();
      state.feet.feet[i].position.y = toe_pose.Pos().Y();
      state.feet.feet[i].position.z = toe_pose.Pos().Z();

    }
    if (toe_vel_opt) {
      const auto &toe_vel = *toe_vel_opt;
      state.feet.feet[i].velocity.x = toe_vel.X();
      state.feet.feet[i].velocity.y = toe_vel.Y();
      state.feet.feet[i].velocity.z = toe_vel.Z();
    }
  }

  state.header.stamp = this->node_->now();
  this->ground_truth_state_pub_->publish(state);

  // Body frame version
  quad_msgs::msg::RobotState state_body_frame = state;
  state_body_frame.body.pose.orientation.w = 1.0;
  state_body_frame.body.pose.orientation.x = 0.0;
  state_body_frame.body.pose.orientation.y = 0.0;
  state_body_frame.body.pose.orientation.z = 0.0;
  state_body_frame.body.pose.position.x = 0.0;
  state_body_frame.body.pose.position.y = 0.0;
  state_body_frame.body.pose.position.z = 0.0;

  this->ground_truth_state_body_frame_pub_->publish(state_body_frame);
}

}  // namespace gz_plugins

GZ_ADD_PLUGIN(gz_plugins::GroundTruthEstimator, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_plugins::GroundTruthEstimator, "ground_truth_estimator")
