#ifdef LOG
#undef LOG
#endif
#include "gazebo_plugins/spawn_lock_plugin.hpp"

#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

namespace gz_plugins {

void SpawnLock::Configure(const gz::sim::Entity& entity,
                          const std::shared_ptr<const sdf::Element>& sdf,
                          gz::sim::EntityComponentManager& /*ecm*/,
                          gz::sim::EventManager& /*eventMgr*/) {
  this->model_ = gz::sim::Model(entity);
  if (sdf->HasElement("body_link")) {
    this->body_link_name_ = sdf->Get<std::string>("body_link");
  }
  if (sdf->HasElement("hold_duration")) {
    this->hold_duration_ = sdf->Get<double>("hold_duration");
  }
}

void SpawnLock::PreUpdate(const gz::sim::UpdateInfo& info,
                          gz::sim::EntityComponentManager& ecm) {
  if (this->released_) return;

  auto body_entity = this->model_.LinkByName(ecm, this->body_link_name_);
  if (body_entity == gz::sim::kNullEntity) return;

  if (!this->initialized_) {
    auto pose_comp = ecm.Component<gz::sim::components::Pose>(body_entity);
    if (pose_comp == nullptr) return;
    this->initial_pose_ = pose_comp->Data();
    this->start_time_ = info.simTime;
    this->wall_start_time_ = std::chrono::steady_clock::now();
    this->initialized_ = true;
  }

  const double elapsed_sim =
      std::chrono::duration<double>(info.simTime - this->start_time_).count();
  const double elapsed_wall =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    this->wall_start_time_)
          .count();

  // Release when either clock crosses hold_duration. Removing the cmd
  // components is required — leaving them latched pins the body forever.
  if (elapsed_sim >= this->hold_duration_ ||
      elapsed_wall >= this->hold_duration_) {
    ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(body_entity);
    ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(body_entity);
    ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(body_entity);
    this->released_ = true;
    return;
  }

  // Pin pose, zero velocities each tick.
  auto pose_cmd = ecm.Component<gz::sim::components::WorldPoseCmd>(body_entity);
  if (pose_cmd == nullptr) {
    ecm.CreateComponent(body_entity,
                        gz::sim::components::WorldPoseCmd(this->initial_pose_));
  } else {
    *pose_cmd = gz::sim::components::WorldPoseCmd(this->initial_pose_);
  }

  auto lin_cmd =
      ecm.Component<gz::sim::components::LinearVelocityCmd>(body_entity);
  if (lin_cmd == nullptr) {
    ecm.CreateComponent(body_entity, gz::sim::components::LinearVelocityCmd(
                                         gz::math::Vector3d::Zero));
  } else {
    *lin_cmd = gz::sim::components::LinearVelocityCmd(gz::math::Vector3d::Zero);
  }

  auto ang_cmd =
      ecm.Component<gz::sim::components::AngularVelocityCmd>(body_entity);
  if (ang_cmd == nullptr) {
    ecm.CreateComponent(body_entity, gz::sim::components::AngularVelocityCmd(
                                         gz::math::Vector3d::Zero));
  } else {
    *ang_cmd =
        gz::sim::components::AngularVelocityCmd(gz::math::Vector3d::Zero);
  }
}

}  // namespace gz_plugins

GZ_ADD_PLUGIN(gz_plugins::SpawnLock, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_plugins::SpawnLock, "spawn_lock")
