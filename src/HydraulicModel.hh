#pragma once

#include <algorithm>
#include <cmath>

namespace bale_sim
{
// Reduced-order actuator; arm is stroke/joint-coordinate ratio. For revolute
// joints it is an effective moment arm (m/rad), for direct prismatic joints 1.
// Effort is consequently torque (Nm) or linear force (N). No fluid states.
struct Cylinder
{
  double bore, rod, count, arm, maxSpeed, branchFlow;
  double Area(bool extending) const
  {
    constexpr double pi = 3.14159265358979323846;
    return pi * (bore * bore - (extending ? 0.0 : rod * rod)) / 4.0;
  }
  double Demand(double valve) const
  {
    const double area = count * Area(valve >= 0.0);
    return std::min(branchFlow * std::abs(valve),
                    area * arm * maxSpeed);
  }
  double Speed(double valve, double flowScale) const
  {
    if (valve == 0.0) return 0.0;
    return std::copysign(Demand(valve) * flowScale /
      (count * Area(valve >= 0.0) * arm), valve);
  }
  double EffortLimit(double effort, double pressure) const
  {
    return pressure * count * Area(effort >= 0.0) * arm;
  }
};

inline double FlowScale(double pump, double demand)
{
  return demand > pump ? pump / demand : 1.0;
}

inline double Valve(double value)
{
  return std::isfinite(value) && std::abs(value) > 0.05 ?
    std::clamp(value, -1.0, 1.0) : 0.0;
}
}
