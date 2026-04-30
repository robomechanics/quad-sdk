#ifdef LOG
#undef LOG
#endif
#include "gazebo_plugins/estimator_plugin.hpp"

namespace gz_plugins {
namespace {

template <typename ParamType>
void declare_if_missing(const rclcpp::Node::SharedPtr& node,
                        const std::string& param_name,
                        const ParamType& default_value) {
  if (!node->has_parameter(param_name)) {
    node->declare_parameter<ParamType>(param_name, default_value);
  }
}

bool load_leg_joint_names(const rclcpp::Node::SharedPtr& node, int leg_index,
                          std::array<std::string, 3>& joint_names) {
  const std::string leg_ns = "leg_" + std::to_string(leg_index);
  declare_if_missing<std::vector<std::string>>(
      node, leg_ns + ".joint_names", std::vector<std::string>({"", "", ""}));
  declare_if_missing<std::string>(node, leg_ns + ".joints.abad.name", "");
  declare_if_missing<std::string>(node, leg_ns + ".joints.hip.name", "");
  declare_if_missing<std::string>(node, leg_ns + ".joints.knee.name", "");
  std::string abad_name;
  std::string hip_name;
  std::string knee_name;
  if (node->get_parameter(leg_ns + ".joints.abad.name", abad_name) &&
      node->get_parameter(leg_ns + ".joints.hip.name", hip_name) &&
      node->get_parameter(leg_ns + ".joints.knee.name", knee_name) &&
      !abad_name.empty() && !hip_name.empty() && !knee_name.empty()) {
    joint_names = {abad_name, hip_name, knee_name};
    return true;
  }

  std::vector<std::string> legacy_joint_names;
  if (node->get_parameter(leg_ns + ".joint_names", legacy_joint_names) &&
      legacy_joint_names.size() == 3) {
    joint_names = {legacy_joint_names[0], legacy_joint_names[1],
                   legacy_joint_names[2]};
    return true;
  }
  return false;
}

bool load_robot_names(const rclcpp::Node::SharedPtr& node,
                      std::string& body_frame_name,
                      std::array<std::string, 4>& lower_frame_names,
                      std::array<std::string, 4>& toe_frame_names,
                      std::vector<std::string>& joint_names) {
  declare_if_missing<std::string>(node, "body.frame", "body");
  node->get_parameter("body.frame", body_frame_name);

  joint_names.clear();
  for (int leg_index = 0; leg_index < 4; ++leg_index) {
    const std::string leg_ns = "leg_" + std::to_string(leg_index);
    declare_if_missing<std::string>(node, leg_ns + ".frames.lower", "");
    declare_if_missing<std::string>(node, leg_ns + ".frames.toe", "");
    node->get_parameter(leg_ns + ".frames.lower", lower_frame_names[leg_index]);
    node->get_parameter(leg_ns + ".frames.toe", toe_frame_names[leg_index]);

    std::array<std::string, 3> leg_joint_names;
    if (!load_leg_joint_names(node, leg_index, leg_joint_names)) {
      return false;
    }
    if (lower_frame_names[leg_index].empty() ||
        toe_frame_names[leg_index].empty()) {
      return false;
    }

    // Preserve the estimator's published order: [abad, hip, knee] per leg.
    joint_names.insert(joint_names.end(), leg_joint_names.begin(),
                       leg_joint_names.end());
  }

  return !body_frame_name.empty() && joint_names.size() == 12;
}

}  // namespace

void GroundTruthEstimator::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) {
  this->model_ = gz::sim::Model(entity);
  this->entity_ = entity;

  std::string robot_ns = this->model_.Name(ecm);
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::string params_file;
  if (sdf->HasElement("parameters")) {
    params_file = sdf->Get<std::string>("parameters");
    // RCLCPP_INFO(this->node_->get_logger(), "")
  }

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);
  // Force use_sim_time=true via parameter_overrides BEFORE the Node is
  // constructed.
  options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
  });

  if (!params_file.empty()) {
    options.arguments({"--ros-args", "--params-file", params_file});
  }
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

  if (!load_robot_names(this->node_, this->body_frame_name_,
                        this->lower_frame_names_, this->toe_frame_names_,
                        this->joint_names_)) {
    RCLCPP_FATAL(this->node_->get_logger(),
                 "Missing robot naming config. Expected 'body.frame', "
                 "'leg_i.frames.(lower|toe)', and either "
                 "'leg_i.joints.(abad|hip|knee).name' or legacy "
                 "'leg_i.joint_names'.");
    rclcpp::shutdown();
    return;
  }

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
            this->quadKD_ = std::make_shared<quad_utils::QuadKD2>(this->node_);
            RCLCPP_INFO(this->node_->get_logger(), "Makes QuadKD Class.");
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->node_->get_logger(), "QuadKD init failed: %s",
                         e.what());
          }
        }
      });

  std::vector<std::string> links_to_check = {this->body_frame_name_};
  links_to_check.insert(links_to_check.end(), this->toe_frame_names_.begin(),
                        this->toe_frame_names_.end());

  for (const auto& link_name : links_to_check) {
    auto link_entity = this->model_.LinkByName(ecm, link_name);
    if (link_entity == gz::sim::kNullEntity) continue;

    if (!ecm.EntityHasComponentType(
            link_entity, gz::sim::components::WorldLinearVelocity::typeId)) {
      ecm.CreateComponent(link_entity,
                          gz::sim::components::WorldLinearVelocity());
    }

    if (!ecm.EntityHasComponentType(
            link_entity, gz::sim::components::WorldAngularVelocity::typeId)) {
      ecm.CreateComponent(link_entity,
                          gz::sim::components::WorldAngularVelocity());
    }

    if (!ecm.EntityHasComponentType(link_entity,
                                    gz::sim::components::Pose::typeId)) {
      ecm.CreateComponent(link_entity, gz::sim::components::Pose());
    }
  }

  for (const auto& joint_name : this->joint_names_) {
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

  for (const auto& joint_name : this->joint_names_) {
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
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm) {
  rclcpp::spin_some(this->node_);
  if (!this->node_ || !this->model_.Valid(ecm)) return;

  if (!urdf_received_) return;

  const double now_s = this->node_->now().seconds();
  if (now_s > 1e8) {
    RCLCPP_WARN_THROTTLE(
        this->node_->get_logger(), *this->node_->get_clock(), 1000,
        "node_->now() returned %.3f s (looks like wall clock, not sim "
        "time). Skipping ground_truth publish until /clock arrives.",
        now_s);
    return;
  }

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
  auto body_entity = this->model_.LinkByName(ecm, this->body_frame_name_);

  if (body_entity == gz::sim::kNullEntity) {
    RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(),
                         2000,
                         "Can't find body link in sdf. Make sure the name in "
                         "the plugin matches the sdf.");
    return;
  }

  gz::sim::Link body_link(body_entity);
  std::array<gz::sim::Link, 4> toe_links;
  for (int i = 0; i < 4; ++i) {
    auto lower_entity =
        this->model_.LinkByName(ecm, this->lower_frame_names_[i]);
    auto toe_entity = this->model_.LinkByName(ecm, this->toe_frame_names_[i]);
    if (lower_entity == gz::sim::kNullEntity ||
        toe_entity == gz::sim::kNullEntity) {
      RCLCPP_WARN_THROTTLE(
          this->node_->get_logger(), *this->node_->get_clock(), 2000,
          "Can't find leg links in sdf. Make sure the lower "
          "and toe frame names in the robot yaml match the model.");
      return;
    }
    toe_links[i] = gz::sim::Link(toe_entity);
  }

  auto pose_opt = body_link.WorldPose(ecm);
  auto lin_vel_opt = body_link.WorldLinearVelocity(ecm);
  auto ang_vel_opt = body_link.WorldAngularVelocity(ecm);

  const auto& pose = *pose_opt;
  const auto& lin_vel = *lin_vel_opt;
  const auto& ang_vel = *ang_vel_opt;

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

  const auto q_wb = pose.Rot();
  const auto q_bw = q_wb.Inverse();

  const gz::math::Vector3d v_w = lin_vel;     // world linear vel of body origin
  const gz::math::Vector3d w_w = ang_vel;     // world angular vel
  const gz::math::Vector3d v_b = q_bw * v_w;  // express in body frame
  const gz::math::Vector3d w_b = q_bw * w_w;  // express in body frame

  state.body.twist.angular.x = w_b.X();
  state.body.twist.angular.y = w_b.Y();
  state.body.twist.angular.z = w_b.Z();

  // Update the Joints
  int num_joints = 12;
  state.joints.name = this->joint_names_;

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
      if (vel_opt && !vel_opt->empty()) {
        vel = (*vel_opt)[0];
      }
      if (wrench_opt) {
        const auto& wrench_msg = (*wrench_opt)[0];
        const auto& torque_msg = wrench_msg.torque();

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
  quad_utils::updateDynamics(*this->quadKD_, state);
  quad_utils::fkRobotState(*this->quadKD_, state);

  // Update the Feet Positions and Velocities
  for (int i = 0; i < 4; i++) {
    auto toe_pose_opt = toe_links[i].WorldPose(ecm);
    auto toe_vel_opt = toe_links[i].WorldLinearVelocity(ecm);
    if (toe_pose_opt) {
      const auto& toe_pose = *toe_pose_opt;
      state.feet.feet[i].position.x = toe_pose.Pos().X();
      state.feet.feet[i].position.y = toe_pose.Pos().Y();
      state.feet.feet[i].position.z = toe_pose.Pos().Z();
    }
    if (toe_vel_opt) {
      const auto& toe_vel = *toe_vel_opt;
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

  state_body_frame.body.twist.linear.x = v_b.X();
  state_body_frame.body.twist.linear.y = v_b.Y();
  state_body_frame.body.twist.linear.z = v_b.Z();
  state_body_frame.body.twist.angular.x = w_b.X();
  state_body_frame.body.twist.angular.y = w_b.Y();
  state_body_frame.body.twist.angular.z = w_b.Z();

  // Feet positions/velocities in body frame
  const gz::math::Vector3d p_body_w =
      pose.Pos();  // world position of body origin

  for (int i = 0; i < 4; ++i) {
    auto toe_pose_opt = toe_links[i].WorldPose(ecm);
    auto toe_vel_opt = toe_links[i].WorldLinearVelocity(ecm);
    if (!toe_pose_opt || !toe_vel_opt) continue;

    const auto& toe_pose_w = *toe_pose_opt;
    const auto& toe_vel_w = *toe_vel_opt;

    // r: toe position relative to body origin, in world
    const gz::math::Vector3d r_w = toe_pose_w.Pos() - p_body_w;
    // position expressed in body frame
    const gz::math::Vector3d p_toe_b = q_bw * r_w;

    // relative velocity of toe wrt body origin in world:
    // v_rel_w = (toe_vel - body_vel) - ω × r
    const gz::math::Vector3d v_rel_w = (toe_vel_w - v_w) - w_w.Cross(r_w);
    // express in body frame
    const gz::math::Vector3d v_toe_b = q_bw * v_rel_w;

    state_body_frame.feet.feet[i].position.x = p_toe_b.X();
    state_body_frame.feet.feet[i].position.y = p_toe_b.Y();
    state_body_frame.feet.feet[i].position.z = p_toe_b.Z();

    state_body_frame.feet.feet[i].velocity.x = v_toe_b.X();
    state_body_frame.feet.feet[i].velocity.y = v_toe_b.Y();
    state_body_frame.feet.feet[i].velocity.z = v_toe_b.Z();
  }

  this->ground_truth_state_body_frame_pub_->publish(state_body_frame);
}

}  // namespace gz_plugins

GZ_ADD_PLUGIN(gz_plugins::GroundTruthEstimator, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_plugins::GroundTruthEstimator, "ground_truth_estimator")
