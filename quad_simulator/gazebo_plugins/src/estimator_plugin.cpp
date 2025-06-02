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
          // RCLCPP_INFO(this->node_->get_logger(), "URDF (raw msg):\n%s",
          //             msg->data.c_str());
          // Declare and set the robot_description parameter manually
          // if (!this->node_->has_parameter("robot_description")) {
          this->node_->declare_parameter<std::string>("robot_description",
                                                      msg->data);
          // } else {
          //   this->node_->set_parameter(
          //       rclcpp::Parameter("robot_description", msg->data));
          // }

          RCLCPP_INFO(this->node_->get_logger(),
                      "Received and set robot_description parameter.");

          // You can now create QuadKD since robot_description is set
          try {
            // RCLCPP_INFO(this->node_->get_logger(), "Inside Try Block");
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

  // RCLCPP_INFO(this->node_->get_logger(),
  //             "Makes it to the end of the Configure");
  // this->quadKD_ = std::make_shared<quad_utils::QuadKD>(node_, robot_ns);
}

void GroundTruthEstimator::PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) {
  // RCLCPP_INFO(this->node_->get_logger(), "Updating the Estimate");
  rclcpp::spin_some(this->node_);
  if (!this->node_ || !this->model_.Valid(ecm)) return;
  // RCLCPP_INFO(this->node_->get_logger(), "Valid Node, Model");
  if (!urdf_received_) return;

  if (!this->time_initialized_) {
    this->last_time_ = info.simTime;
    this->time_initialized_ = true;
    RCLCPP_INFO(this->node_->get_logger(), "Initializing last_time_ to %.6f",
                std::chrono::duration<double>(last_time_).count());
    return;
  }
  // if (this->last_time_.count() == 0) {
  //   this->last_time_ = info.simTime;
  //     RCLCPP_INFO(this->node_->get_logger(), "Doing this too often");
  //   return;
  // }
  // RCLCPP_INFO(this->node_->get_logger(), "Initializing Time");
  double dt = std::chrono::duration<double>(info.simTime - last_time_).count();
  // RCLCPP_INFO(this->node_->get_logger(), "dt: %.6f", dt);
  // RCLCPP_INFO(this->node_->get_logger(), "SimTime: %.6f, LastTime: %.6f",
  //             std::chrono::duration<double>(info.simTime).count(),
  //             std::chrono::duration<double>(last_time_).count());
  if (this->update_rate_ > 0.0 && dt < (1.0 / this->update_rate_)) return;
  // RCLCPP_INFO(this->node_->get_logger(), "Passing Update Rate Check");
  this->last_time_ = info.simTime;
  // Extract all relevant information from the simulator
  auto body_link = this->model_.LinkByName(ecm, "body");
  // RCLCPP_INFO(this->node_->get_logger(), "Finds Body");
  if (!body_link) {
    RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(),
                         2000,
                         "Can't find body link in sdf. Make sure the name in "
                         "the plugin matches the sdf.");
    return;
  }

  auto lower0 = this->model_.LinkByName(ecm, "lower0");
  auto lower1 = this->model_.LinkByName(ecm, "lower1");
  auto lower2 = this->model_.LinkByName(ecm, "lower2");
  auto lower3 = this->model_.LinkByName(ecm, "lower3");

  auto toe0 = this->model_.LinkByName(ecm, "toe0");
  auto toe1 = this->model_.LinkByName(ecm, "toe1");
  auto toe2 = this->model_.LinkByName(ecm, "toe2");
  auto toe3 = this->model_.LinkByName(ecm, "toe3");

  // Ign Gazebo returns these as std::optional<gz::math::Pose3d>
  // Dereference them before assignment
  // RCLCPP_INFO(this->node_->get_logger(), "3");
  auto pose_comp = ecm.Component<gz::sim::components::Pose>(body_link);
  auto lin_vel_comp =
      ecm.Component<gz::sim::components::WorldLinearVelocity>(body_link);
  auto ang_vel_comp =
      ecm.Component<gz::sim::components::AngularVelocity>(body_link);
  // RCLCPP_INFO(this->node_->get_logger(), "4");
  // if (!pose_comp) {
  //   RCLCPP_WARN(this->node_->get_logger(),
  //               "[Tick] Pose component still missing");
  // }
  // if (!lin_vel_comp) {
  //   RCLCPP_WARN(this->node_->get_logger(),
  //               "[Tick] LinVel  component still missing");
  // }
  // if (!ang_vel_comp) {
  //   RCLCPP_WARN(this->node_->get_logger(),
  //               "[Tick] AngVel component still missing");
  // }

  if (!pose_comp || !lin_vel_comp || !ang_vel_comp) return;
  const auto &pose = pose_comp->Data();
  const auto &lin_vel = lin_vel_comp->Data();
  const auto &ang_vel = ang_vel_comp->Data();
  // RCLCPP_INFO(this->node_->get_logger(), "5");

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

  // RCLCPP_INFO(this->node_->get_logger(),
  //             "Body Position: x=%.3f, y=%.3f, z=%.3f",
  //             state.body.pose.position.x, state.body.pose.position.y,
  //             state.body.pose.position.z);

  // Update the Joints
  int num_joints = 12;
  state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                       "10", "4", "5", "11", "6", "7"};
  for (int i = 0; i < num_joints; i++) {
    auto joint = this->model_.JointByName(ecm, state.joints.name[i]);
    double jpos = 0.0, jvel = 0.0;
    if (joint) {
      auto posComp = ecm.Component<gz::sim::components::JointPosition>(joint);
      auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joint);
      if (posComp && !posComp->Data().empty()) jpos = posComp->Data()[0];
      if (velComp && !velComp->Data().empty()) jvel = velComp->Data()[0];
    }
    state.joints.position.push_back(jpos);
    state.joints.velocity.push_back(jvel);
    state.joints.effort.push_back(0.0);  // Torque placeholder
  }

  int num_feet = 4;
  state.feet.feet.resize(num_feet);
  quad_utils::fkRobotState(*this->quadKD_, state);

  // Update the Feet Positions and Velocities
  std::vector<gz::sim::Entity> toes = {toe0, toe1, toe2, toe3};

  for (int i = 0; i < 4; i++) {
    auto toe_pose_comp = ecm.Component<gz::sim::components::Pose>(toes[i]);
    auto toe_vel_comp =
        ecm.Component<gz::sim::components::WorldLinearVelocity>(toes[i]);
    if (toe_pose_comp) {
      const auto &toe_pose = toe_pose_comp->Data();
      state.feet.feet[i].position.x = toe_pose.Pos().X();
      state.feet.feet[i].position.y = toe_pose.Pos().Y();
      state.feet.feet[i].position.z = toe_pose.Pos().Z();
    }
    if (toe_vel_comp) {
      const auto &toe_vel = toe_vel_comp->Data();
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
