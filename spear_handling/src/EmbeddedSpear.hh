#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>
#include <gz/math/Vector3.hh>

namespace spear_handling
{
using Vec = gz::math::Vector3d;

struct MaterialParameters
{
  int samples = 32;
  double transverseStiffness = 100000.; // N/m displacement per m embedded shaft
  double transverseDamping = 3000.;    // N/(m/s) per m embedded shaft
  double bearingLimit = 5000.;        // N per m embedded shaft, both directions
  double axialStiffness = 10000.;
  double axialDamping = 200.;
  double confinement = 100.;          // N per m: unresolved radial compression
  double friction = .3;
  double cuttingForce = 20.;          // N per shaft, only advancing into new material
  void Validate() const
  {
    if(samples<4 || samples>256) throw std::invalid_argument("Invalid material sample count");
    for(double v:{transverseStiffness,bearingLimit,axialStiffness})
      if(!std::isfinite(v) || v<=0) throw std::invalid_argument("Invalid material stiffness/capacity");
    for(double v:{transverseDamping,axialDamping,confinement,friction,cuttingForce})
      if(!std::isfinite(v) || v<0) throw std::invalid_argument("Invalid material damping/friction");
  }
};

inline bool Clip(const Vec &origin, const Vec &axis, const Vec &half,
                 double &lo, double &hi)
{
  for (int j=0; j<3; ++j)
  {
    if (std::abs(axis[j]) < 1e-10)
    {
      if (std::abs(origin[j]) > half[j]) return false;
    }
    else
    {
      double a=(-half[j]-origin[j])/axis[j], b=(half[j]-origin[j])/axis[j];
      if (a>b) std::swap(a,b);
      lo=std::max(lo,a); hi=std::min(hi,b);
    }
  }
  return hi>lo;
}

struct MaterialNode
{
  double coordinate=0, slip=0;
  Vec anchor=Vec::Zero;
  bool entered=false;
};

struct AppliedForce { Vec point, force; };
struct MaterialResult
{
  std::vector<AppliedForce> forces;
  double overlap=0, axialDrag=0, dampingPowerProxy=0;
  int active=0, yielded=0;
};

// A finite shaft moving through a local elastic/plastic material foundation.
// Anchors are created where the shaft actually enters, not at pre-cut holes.
// All coordinates and velocities passed here are in the bale frame.
class EmbeddedSpear
{
  Vec referenceOrigin=Vec::Zero, referenceAxis=Vec::UnitX;
  double cell=0, maxTip=0;
  std::vector<MaterialNode> nodes;
 public:
  void Reset() { nodes.clear(); }

  MaterialResult Step(const MaterialParameters &p, const Vec &origin,
      const Vec &axis, double length, const Vec &half, double dt,
      const std::function<Vec(const Vec &)> &relativeVelocity)
  {
    MaterialResult out;
    double lo=0, hi=length;
    if (!Clip(origin,axis,half,lo,hi) || hi-lo<1e-8)
    {
      Reset(); // Fully withdrawn: no residual force or attachment.
      return out;
    }
    out.overlap=hi-lo;
    if (nodes.empty())
    {
      referenceOrigin=origin; referenceAxis=axis;
      double entry=0, exit=1e6;
      Clip(origin,axis,half,entry,exit);
      cell=(exit-entry)/p.samples;
      maxTip=entry;
      for (int i=0; i<p.samples; ++i)
        nodes.push_back({entry+(i+.5)*cell,0,Vec::Zero,false});
    }
    const double projection=axis.Dot(referenceAxis);
    if (projection<.1) { Reset(); return out; }
    const double root=(origin-referenceOrigin).Dot(referenceAxis);
    const double begin=root+lo*projection, end=root+hi*projection;
    for (auto &n:nodes)
    {
      const double a=std::max(begin,n.coordinate-cell*.5);
      const double b=std::min(end,n.coordinate+cell*.5);
      if (b<=a) { n.slip=0; continue; }
      const double weight=(b-a)/projection;
      // The material cell is sampled at the midpoint of its covered interval.
      // Its anchor starts at the current passage, so virgin material carries
      // no pre-existing transverse spring strain.
      const Vec initial=origin+axis*((.5*(a+b)-root)/projection);
      if (!n.entered) { n.anchor=initial; n.entered=true; }
      const double s=std::clamp((n.anchor-origin).Dot(axis),lo,hi);
      const Vec point=origin+axis*s;
      Vec error=point-n.anchor;
      error-=axis*error.Dot(axis);
      // Perfect plasticity limits stored strain and allows overload to move
      // the passage rather than supporting arbitrary force or moment.
      const double yield=p.bearingLimit/p.transverseStiffness;
      if (error.Length()>yield)
      {
        n.anchor+=error*(1.-yield/error.Length());
        error*=yield/error.Length();
        ++out.yielded;
      }
      const Vec velocity=relativeVelocity(point);
      const double axialSpeed=velocity.Dot(axis);
      const Vec transverseVelocity=velocity-axis*axialSpeed;
      Vec bearing=(error*p.transverseStiffness+
                   transverseVelocity*p.transverseDamping)*weight;
      const double cap=p.bearingLimit*weight;
      if (bearing.Length()>cap) bearing*=cap/bearing.Length();
      // Axial bristle friction supports static load below a Coulomb limit;
      // sliding yields continuously instead of an attached/detached switch.
      const double frictionLimit=p.friction*(p.confinement*weight+bearing.Length());
      n.slip+=axialSpeed*dt;
      const double trial=(p.axialStiffness*n.slip+p.axialDamping*axialSpeed)*weight;
      const double drag=std::clamp(trial,-frictionLimit,frictionLimit);
      if (std::abs(trial)>frictionLimit)
        n.slip=(drag/weight-p.axialDamping*axialSpeed)/p.axialStiffness;
      out.forces.push_back({point,bearing+axis*drag});
      out.axialDrag+=std::abs(drag);
      out.dampingPowerProxy+=p.transverseDamping*weight*transverseVelocity.SquaredLength();
      ++out.active;
    }
    const double tip=(origin+axis*length-referenceOrigin).Dot(referenceAxis);
    const Vec tipPoint=origin+axis*hi;
    if (hi>=length-1e-8 && tip>maxTip+1e-7 && relativeVelocity(tipPoint).Dot(axis)>0)
    {
      const double cut=p.cuttingForce*std::min(1.,out.overlap/.02);
      out.forces.push_back({tipPoint,axis*cut});
      out.axialDrag+=cut;
    }
    maxTip=std::max(maxTip,tip);
    return out;
  }
};
}
