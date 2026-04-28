#ifndef GAZEBO_PLUGINS_SPAWN_LOCK_PLUGIN
#define GAZEBO_PLUGINS_SPAWN_LOCK_PLUGIN

#ifdef LOG
#undef LOG
#endif

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include <chrono>
#include <string>

namespace gz_plugins {

// Pins a model's body link at its spawn pose for a fixed duration, then
// releases it to normal physics. Joints articulate freely throughout, so
// ros2_control's joint controllers can fold the legs into sit pose while
// the body is held — by the time the lock releases, the robot is in a
// kinematically stable configuration and doesn't tip.
//
// SDF parameters:
//   <body_link>      name of the link to pin (default: "body")
//   <hold_duration>  seconds of sim-time to hold (default: 8.0)
class SpawnLock : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate {
 public:
  SpawnLock() = default;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

 private:
  gz::sim::Model model_;
  std::string body_link_name_{"body"};
  double hold_duration_{8.0};

  bool initialized_{false};
  bool released_{false};
  gz::math::Pose3d initial_pose_;
  std::chrono::steady_clock::duration start_time_{};
};

}  // namespace gz_plugins

#endif  // GAZEBO_PLUGINS_SPAWN_LOCK_PLUGIN
