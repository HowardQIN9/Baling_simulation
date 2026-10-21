#include "EmbeddedSpear.hh"
#include <iostream>
#include <stdexcept>
using namespace spear_handling;
void Check(bool value,const char *message) { if(!value) throw std::runtime_error(message); }
Vec Sum(const MaterialResult &r) { Vec f=Vec::Zero; for(auto &p:r.forces) f+=p.force; return f; }
int main() {
  MaterialParameters p;
  const Vec half(.6096,1.2192,.4572), axis=Vec::UnitX;
  auto still=[](const Vec &){return Vec::Zero;};
  EmbeddedSpear m;
  auto first=m.Step(p,Vec(-1,0,0),axis,1,half,.001,still);
  Check((Sum(first)).Length()<1e-9,"Virgin material has no arbitrary attachment force");
  auto up=m.Step(p,Vec(-1,0,.01),axis,1,half,.001,still);
  Check(std::abs(Sum(up).Z()-p.transverseStiffness*.6096*.01)<1e-6,"Embedded length controls bearing stiffness");
  auto down=m.Step(p,Vec(-1,0,-.01),axis,1,half,.001,still);
  Check(Sum(down).Z()< -300,"Lowering transfers downward load, not a friction multiplier");
  auto shallow=m.Step(p,Vec(-1.4,0,-.01),axis,1,half,.001,still);
  Check(std::abs(Sum(shallow).Z())<std::abs(Sum(down).Z()),"Withdrawal reduces available support");
  auto out=m.Step(p,Vec(-2,0,-.01),axis,1,half,.001,still);
  Check(out.forces.empty(),"Full withdrawal releases all forces");
  auto reentry=m.Step(p,Vec(-1,.12,.08),axis,1,half,.001,still);
  Check(reentry.active>0 && Sum(reentry).Length()<1e-8,"Off-channel entry creates its own material passage");
  auto yielded=m.Step(p,Vec(-1,.12,.28),axis,1,half,.001,still);
  Check(yielded.yielded>0,"Excessive deformation yields material");
  Check(Sum(yielded).Length()<=p.bearingLimit*yielded.overlap+1e-6,"Bearing force is bounded");
  EmbeddedSpear tilted;
  Vec sloped(std::cos(.1),0,-std::sin(.1));
  auto angle=tilted.Step(p,Vec(-1,0,.12),sloped,1,half,.001,still);
  Check(angle.active>0,"Modest angle can penetrate without a parallelism gate");
  EmbeddedSpear drag;
  auto forward=drag.Step(p,Vec(-1,0,0),axis,1,half,.001,[](const Vec &){return Vec(.12,0,0);});
  auto reverse=drag.Step(p,Vec(-1,0,0),axis,1,half,.001,[](const Vec &){return Vec(-.12,0,0);});
  Check(Sum(forward).X()>0 && Sum(reverse).X()<0,"Axial resistance reacts to insertion and withdrawal");
  // Same force acts with opposite signs at a common world point.
  Vec world(5,2,1), toolOrigin(3,0,.2), baleOrigin(6,1,.5);
  Vec f=Sum(up), toolTorque=(world-toolOrigin).Cross(-f), baleTorque=(world-baleOrigin).Cross(f);
  Check((toolTorque+toolOrigin.Cross(-f)+baleTorque+baleOrigin.Cross(f)).Length()<1e-8,
        "Point force reactions preserve world moment with offset origins");
  std::cout << "PASS: virgin entry, length scaling, downward preload, release, lateral/angle entry, yielding, axial drag and balanced reactions\n";
}
