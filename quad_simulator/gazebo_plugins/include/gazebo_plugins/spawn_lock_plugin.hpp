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

// Pins a model's body link at its spawn pose for hold_duration, then
// releases. Joints articulate freely so controllers can fold legs into
// sit pose before release.
//
// SDF: <body_link> (default "body"), <hold_duration> seconds (default 8.0).
// Releases when sim-time OR wall-clock elapsed crosses hold_duration —
// wall-clock fallback covers gz-sim sim-time stalls.
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
  std::chrono::steady_clock::time_point wall_start_time_{};
};

}  // namespace gz_plugins

#endif  // GAZEBO_PLUGINS_SPAWN_LOCK_PLUGIN
