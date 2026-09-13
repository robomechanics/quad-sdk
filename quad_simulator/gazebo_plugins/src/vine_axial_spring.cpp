#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <sdf/Element.hh>
namespace underbrush {
class VineAxialSpring : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 gz::sim::Entity joint{gz::sim::kNullEntity};
 double stiffness{}, damping{}, target{}, maxForce{};
 public:
 void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
   joint=gz::sim::Model(entity).JointByName(ecm,sdf->Get<std::string>("joint_name"));
   stiffness=sdf->Get<double>("stiffness"); damping=sdf->Get<double>("damping");
   target=sdf->Get<double>("target"); maxForce=sdf->Get<double>("max_force");
   if(joint==gz::sim::kNullEntity || !std::isfinite(stiffness) || !std::isfinite(damping) ||
      !std::isfinite(target) || !std::isfinite(maxForce) || stiffness<0 || damping<0 || maxForce<=0)
      throw std::runtime_error("Invalid vine axial spring configuration");
   if(!ecm.Component<gz::sim::components::JointPosition>(joint))
     ecm.CreateComponent(joint,gz::sim::components::JointPosition());
   if(!ecm.Component<gz::sim::components::JointVelocity>(joint))
     ecm.CreateComponent(joint,gz::sim::components::JointVelocity());
 }
 void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
   if(info.paused) return;
   const auto *q=ecm.Component<gz::sim::components::JointPosition>(joint);
   const auto *v=ecm.Component<gz::sim::components::JointVelocity>(joint);
   if(!q || !v || q->Data().empty() || v->Data().empty()) return;
   double effort=std::clamp(stiffness*(target-q->Data()[0])-damping*v->Data()[0],-maxForce,maxForce);
   auto *cmd=ecm.Component<gz::sim::components::JointForceCmd>(joint);
   if(!cmd) ecm.CreateComponent(joint,gz::sim::components::JointForceCmd({effort}));
   else cmd->Data()={effort};
 }
};
}
GZ_ADD_PLUGIN(underbrush::VineAxialSpring, gz::sim::System,
              underbrush::VineAxialSpring::ISystemConfigure,
              underbrush::VineAxialSpring::ISystemPreUpdate)
