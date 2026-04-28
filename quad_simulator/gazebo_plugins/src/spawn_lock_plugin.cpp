#ifdef LOG
#undef LOG
#endif
#include "gazebo_plugins/spawn_lock_plugin.hpp"

#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/common/Console.hh>

namespace gz_plugins {

void SpawnLock::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& /*eventMgr*/) {
  this->model_ = gz::sim::Model(entity);

  if (sdf->HasElement("body_link")) {
    this->body_link_name_ = sdf->Get<std::string>("body_link");
  }
  if (sdf->HasElement("hold_duration")) {
    this->hold_duration_ = sdf->Get<double>("hold_duration");
  }

  gzmsg << "[SpawnLock] model='" << this->model_.Name(ecm)
        << "' body_link='" << this->body_link_name_
        << "' hold_duration=" << this->hold_duration_ << "s" << std::endl;
}

void SpawnLock::PreUpdate(const gz::sim::UpdateInfo& info,
                          gz::sim::EntityComponentManager& ecm) {
  if (this->released_) return;

  auto body_entity = this->model_.LinkByName(ecm, this->body_link_name_);
  if (body_entity == gz::sim::kNullEntity) return;
  gz::sim::Link body_link(body_entity);

  if (!this->initialized_) {
    auto pose_comp = ecm.Component<gz::sim::components::Pose>(body_entity);
    if (pose_comp == nullptr) return;
    this->initial_pose_ = pose_comp->Data();
    this->start_time_ = info.simTime;
    this->initialized_ = true;
    gzmsg << "[SpawnLock] locking body at "
          << this->initial_pose_ << " for "
          << this->hold_duration_ << "s" << std::endl;
  }

  const double elapsed =
      std::chrono::duration<double>(info.simTime - this->start_time_).count();
  if (elapsed >= this->hold_duration_) {
    // Explicitly remove the cmd components so physics stops re-applying
    // them. Returning without removing leaves the last-written values
    // latched and the body stays pinned indefinitely.
    ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(body_entity);
    ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(body_entity);
    ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(body_entity);
    this->released_ = true;
    gzmsg << "[SpawnLock] releasing body after " << elapsed << "s" << std::endl;
    return;
  }

  // Pin pose and zero velocities each tick. WorldPoseCmd snaps the link to
  // the requested pose on the next physics step; the velocity cmds zero
  // out any momentum the body would otherwise accumulate from gravity or
  // joint reaction forces during the lock window.
  auto pose_cmd =
      ecm.Component<gz::sim::components::WorldPoseCmd>(body_entity);
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
    *ang_cmd = gz::sim::components::AngularVelocityCmd(gz::math::Vector3d::Zero);
  }
}

}  // namespace gz_plugins

GZ_ADD_PLUGIN(gz_plugins::SpawnLock, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_plugins::SpawnLock, "spawn_lock")
