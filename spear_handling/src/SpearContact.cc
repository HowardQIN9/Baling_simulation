// Native support contact plus finite, yielding spear/material interaction.
// No attachment joints, pose commands, or insertion-success switches.
#include "EmbeddedSpear.hh"
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/physics/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/sim/components/Model.hh>

namespace spear_handling {
class SpearContact final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate {
  enum Kind {Other, Bale, Spear, Ground};
  std::map<gz::sim::Entity,Kind> kinds;
  gz::common::ConnectionPtr connection;
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  gz::transport::Node::Publisher posePub;
  std::map<std::string, gz::sim::Entity> measuredEntities;
  double bg=.4,bs=.3,bb=.4,depth=1.2192,length=1.,rootX=.1,rootZ=.15;
  double baleLength=2.4384,baleHeight=.9144,spacing=.5666667;
  double stiffness=100.,viscous=30.,forceCap=200.;
  bool resistance=false;
  bool entitiesDiscovered=false;
  bool material=false;
  MaterialParameters materialParameters;
  std::map<gz::sim::Entity,std::vector<EmbeddedSpear>> passages;
  std::map<gz::sim::Entity,gz::transport::Node::Publisher> materialPubs;
  std::map<gz::sim::Entity,std::vector<double>> materialStates;
  double previousTime=-1;
  int count=4;
  gz::sim::Entity carrier=gz::sim::kNullEntity;
  std::vector<gz::sim::Entity> bales;
  double drag=0,elapsed=0;
  unsigned long contacts[3]{};
  public: void Configure(const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &,gz::sim::EventManager &events) override {
    bg=sdf->Get<double>("bale_ground"); bs=sdf->Get<double>("bale_spear");
    bb=sdf->Get<double>("bale_bale"); depth=sdf->Get<double>("bale_depth");
    baleLength=sdf->Get<double>("bale_length"); baleHeight=sdf->Get<double>("bale_height");
    length=sdf->Get<double>("spear_length"); rootX=sdf->Get<double>("root_x");
    rootZ=sdf->Get<double>("root_z"); count=sdf->Get<int>("count");
    spacing=sdf->Get<double>("spacing"); resistance=sdf->Get<bool>("resistance");
    stiffness=sdf->Get<double>("resistance_stiffness");
    viscous=sdf->Get<double>("resistance_viscous"); forceCap=sdf->Get<double>("resistance_cap");
    material=sdf->Get<bool>("material",false).first;
    if(material) {
      materialParameters.samples=sdf->Get<int>("material_samples");
      materialParameters.transverseStiffness=sdf->Get<double>("material_transverse_stiffness");
      materialParameters.transverseDamping=sdf->Get<double>("material_transverse_damping");
      materialParameters.bearingLimit=sdf->Get<double>("material_bearing_limit_per_m");
      materialParameters.axialStiffness=sdf->Get<double>("material_axial_stiffness");
      materialParameters.axialDamping=sdf->Get<double>("material_axial_damping");
      materialParameters.confinement=sdf->Get<double>("material_confinement_per_m");
      materialParameters.cuttingForce=sdf->Get<double>("material_cutting_force");
      materialParameters.friction=bs;
      materialParameters.Validate();
    }
    pub=node.Advertise<gz::msgs::Double_V>("/spear/contact/state");
    posePub=node.Advertise<gz::msgs::Pose_V>("/spear/world_poses");
    connection=events.Connect<gz::sim::events::CollectContactSurfaceProperties>(
      [this](const gz::sim::Entity &a,const gz::sim::Entity &b,
        const gz::math::Vector3d &,const std::optional<gz::math::Vector3d>,
        const std::optional<gz::math::Vector3d>,const std::optional<double>,
        const size_t, auto &params) {
        auto ka=kinds.count(a)?kinds.at(a):Other;
        auto kb=kinds.count(b)?kinds.at(b):Other;
        double mu=-1; int index=-1;
        if(ka==Bale && kb==Bale) {mu=bb;index=2;}
        else if((ka==Bale && kb==Spear)||(kb==Bale && ka==Spear)) {mu=bs;index=1;}
        else if((ka==Bale && kb==Ground)||(kb==Bale && ka==Ground)) {mu=bg;index=0;}
        if(index>=0) {
          params.frictionCoeff=mu; params.secondaryFrictionCoeff=mu;
          ++contacts[index];
        }
      });
    gzmsg << "SpearContact: pair coefficients ground=" << bg << " spear=" << bs
          << " bale=" << bb << "; resistance=" << resistance << "\n";
  }
  public: void PreUpdate(const gz::sim::UpdateInfo &info,
                         gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    ecm.Each<gz::sim::components::Collision,gz::sim::components::Name>(
      [this,&ecm](const gz::sim::Entity &e,const auto *,const auto *name){
        if(kinds.count(e)) return true;
        auto full=gz::sim::scopedName(e,ecm,"::");
        Kind k=Other;
        if(full.find("::bale_")!=std::string::npos) k=Bale;
        else if(full.find("::ground::")!=std::string::npos) k=Ground;
        else if(name->Data().find("spear_")==0) k=Spear;
        kinds[e]=k;
        if(k!=Other) ecm.CreateComponent(e,gz::sim::components::EnableContactSurfaceCustomization(true));
        return true;
      });
    if(!entitiesDiscovered) {
      ecm.Each<gz::sim::components::Link,gz::sim::components::Name>(
        [this,&ecm](const gz::sim::Entity &e,const auto *,const auto *name){
          if(name->Data()=="attachment_carrier") carrier=e;
          if(name->Data()=="attachment_carrier") measuredEntities["attachment_carrier"]=e;
          if(name->Data()=="bale_link") bales.push_back(e);
          if(name->Data()=="bale_link") {
            const auto *parent=ecm.Component<gz::sim::components::ParentEntity>(e);
            if(parent) {
              const auto *modelName=ecm.Component<gz::sim::components::Name>(parent->Data());
              if(modelName) {
                measuredEntities[modelName->Data()]=e;
                materialPubs[e]=node.Advertise<gz::msgs::Double_V>(
                    "/spear/material/"+modelName->Data());
              }
            }
          }
          if(name->Data()=="attachment_carrier" || name->Data()=="bale_link")
            gz::sim::Link(e).EnableVelocityChecks(ecm);
          return true;
        });
      ecm.Each<gz::sim::components::Model,gz::sim::components::Name>(
        [this](const gz::sim::Entity &e,const auto *,const auto *name){
          if(name->Data()=="simple_tractor") measuredEntities[name->Data()]=e;
          return true;
        });
      entitiesDiscovered=true;
    }
    drag=0;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if(now<previousTime) passages.clear();
    previousTime=now;
    if(material && carrier!=gz::sim::kNullEntity) {
      ApplyMaterial(std::chrono::duration<double>(info.dt).count(),ecm);
      return;
    }
    if(!resistance || carrier==gz::sim::kNullEntity) return;
    gz::sim::Link tool(carrier);
    auto cp=tool.WorldPose(ecm); if(!cp) return;
    auto axis=cp->Rot().RotateVector(gz::math::Vector3d::UnitX);
    for(auto e:bales) {
      gz::sim::Link bale(e); auto bp=bale.WorldPose(ecm); if(!bp) continue;
      for(int i=0;i<count;++i) {
        gz::math::Vector3d local(rootX,(i-(count-1)*.5)*spacing,rootZ);
        auto origin=cp->Pos()+cp->Rot().RotateVector(local);
        auto o=bp->Rot().RotateVectorReverse(origin-bp->Pos());
        auto a=bp->Rot().RotateVectorReverse(axis);
        // Clip the actual finite spear segment against the visual bale volume.
        double lo=0,hi=length;
        const double half[3]={depth*.5,baleLength*.5,baleHeight*.5};
        bool hit=true;
        for(int j=0;j<3;++j) {
          if(std::abs(a[j])<1e-8) {if(std::abs(o[j])>half[j]) hit=false;}
          else {
            double u=(-half[j]-o[j])/a[j],v=(half[j]-o[j])/a[j];
            if(u>v) std::swap(u,v);
            lo=std::max(lo,u); hi=std::min(hi,v);
          }
        }
        if(!hit || hi<=lo) continue;
        auto point=origin+axis*((lo+hi)*.5);
        auto tv=tool.WorldLinearVelocity(ecm,cp->Rot().RotateVectorReverse(point-cp->Pos()));
        auto bv=bale.WorldLinearVelocity(ecm,bp->Rot().RotateVectorReverse(point-bp->Pos()));
        if(!tv || !bv) continue;
        double speed=(*tv-*bv).Dot(axis);
        if(std::abs(speed)<1e-5) continue;
        double force=std::min(forceCap,stiffness*(hi-lo)+viscous*std::abs(speed));
        auto f=axis*std::copysign(force,speed);
        // Dissipative drag: F dot relative velocity <= 0; both bodies receive
        // the reaction at the same world point, preserving net force/moment.
        bale.AddWorldWrench(ecm,f,(point-bp->Pos()).Cross(f));
        tool.AddWorldWrench(ecm,-f,(point-cp->Pos()).Cross(-f));
        drag+=force;
      }
    }
  }
  private: void ApplyMaterial(double dt,gz::sim::EntityComponentManager &ecm) {
    if(dt<=0) return;
    gz::sim::Link tool(carrier);
    auto cp=tool.WorldPose(ecm);
    if(!cp || !tool.WorldLinearVelocity(ecm) || !tool.WorldAngularVelocity(ecm)) return;
    const auto axisWorld=cp->Rot().RotateVector(Vec::UnitX);
    Vec toolForce=Vec::Zero,toolTorque=Vec::Zero;
    for(auto entity:bales) {
      gz::sim::Link bale(entity);
      auto bp=bale.WorldPose(ecm);
      if(!bp || !bale.WorldLinearVelocity(ecm) || !bale.WorldAngularVelocity(ecm)) continue;
      auto &shafts=passages[entity];
      if(shafts.empty()) shafts.resize(count);
      Vec force=Vec::Zero,torque=Vec::Zero;
      double overlap=0,axialDrag=0,dissipation=0;
      int active=0,yielded=0;
      for(int i=0;i<count;++i) {
        const Vec originWorld=cp->Pos()+cp->Rot().RotateVector(
            Vec(rootX,(i-(count-1)*.5)*spacing,rootZ));
        const Vec origin=bp->Rot().RotateVectorReverse(originWorld-bp->Pos());
        const Vec axis=bp->Rot().RotateVectorReverse(axisWorld);
        auto velocity=[&](const Vec &point) {
          const Vec world=bp->Pos()+bp->Rot().RotateVector(point);
          auto tv=tool.WorldLinearVelocity(ecm,cp->Rot().RotateVectorReverse(world-cp->Pos()));
          auto bv=bale.WorldLinearVelocity(ecm,point);
          return bp->Rot().RotateVectorReverse(*tv-*bv);
        };
        auto result=shafts[i].Step(materialParameters,origin,axis,length,
                                  Vec(depth*.5,baleLength*.5,baleHeight*.5),dt,velocity);
        overlap+=result.overlap; axialDrag+=result.axialDrag;
        dissipation+=result.dampingPowerProxy;
        active+=result.active; yielded+=result.yielded;
        for(const auto &applied:result.forces) {
          const Vec world=bp->Pos()+bp->Rot().RotateVector(applied.point);
          const Vec f=bp->Rot().RotateVector(applied.force);
          force+=f; torque+=(world-bp->Pos()).Cross(f);
          // Same world application point on both bodies: balanced force and
          // moment even though the implement center of mass is offset.
          toolForce-=f; toolTorque+=(world-cp->Pos()).Cross(-f);
        }
      }
      bale.AddWorldWrench(ecm,force,torque);
      materialStates[entity]={double(entity),overlap,force.X(),force.Y(),force.Z(),
          torque.X(),torque.Y(),torque.Z(),double(active),double(yielded),axialDrag,dissipation};
      drag+=axialDrag;
    }
    tool.AddWorldWrench(ecm,toolForce,toolTorque);
  }
  public: void PostUpdate(const gz::sim::UpdateInfo &info,
                          const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    elapsed+=std::chrono::duration<double>(info.dt).count();
    if(elapsed<.05) return;
    elapsed=0;
    gz::msgs::Double_V m;
    for(double v:{std::chrono::duration<double>(info.simTime).count(),
        double(contacts[0]),double(contacts[1]),double(contacts[2]),drag,bg,bs,bb}) m.add_data(v);
    pub.Publish(m);
    for(const auto &[entity,values]:materialStates) {
      gz::msgs::Double_V state;
      state.add_data(std::chrono::duration<double>(info.simTime).count());
      for(double v:values) state.add_data(v);
      materialPubs.at(entity).Publish(state);
    }
    // Pose_V from the scene mixes model-world and link-parent poses. Export
    // explicit world poses from one simulation update for experimental control.
    gz::msgs::Pose_V poses;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    poses.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    poses.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    for(const auto &[name,entity]:measuredEntities) {
      const auto wp=gz::sim::worldPose(entity,ecm);
      auto *p=poses.add_pose(); p->set_name(name); p->set_id(entity);
      p->mutable_position()->set_x(wp.Pos().X());
      p->mutable_position()->set_y(wp.Pos().Y());
      p->mutable_position()->set_z(wp.Pos().Z());
      p->mutable_orientation()->set_w(wp.Rot().W());
      p->mutable_orientation()->set_x(wp.Rot().X());
      p->mutable_orientation()->set_y(wp.Rot().Y());
      p->mutable_orientation()->set_z(wp.Rot().Z());
    }
    posePub.Publish(poses);
  }
};
}
GZ_ADD_PLUGIN(spear_handling::SpearContact,gz::sim::System,
  spear_handling::SpearContact::ISystemConfigure,
  spear_handling::SpearContact::ISystemPreUpdate,
  spear_handling::SpearContact::ISystemPostUpdate)
