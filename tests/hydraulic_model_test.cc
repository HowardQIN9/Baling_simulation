#include "HydraulicModel.hh"
#include <cmath>
#include <cstdlib>
#include <iostream>

void Check(bool result, const char *message)
{
  if (!result) { std::cerr << message << '\n'; std::exit(1); }
}
int main()
{
  bale_sim::Cylinder lift{0.07, 0.04, 2, 0.32, 0.20, 60.0/60000};
  bale_sim::Cylinder tilt{0.05, 0.03, 2, 0.18, 0.35, 45.0/60000};
  Check(lift.Area(true) > lift.Area(false), "Rod reduces retracting area");
  Check(lift.Speed(0, 1) == 0, "Neutral must request zero speed");
  Check(lift.Speed(-1, 1) < 0, "Negative valve retracts");
  Check(std::abs(lift.Speed(1, 1) - 0.20) < 1e-12, "Lift speed bound");
  const double demand = lift.Demand(1) + tilt.Demand(1);
  const double pump = 10.0/60000;
  const double scale = bale_sim::FlowScale(pump, demand);
  Check(std::abs(demand*scale - pump) < 1e-12, "Shared pump flow conservation");
  const double expected = 200e5 * 2 * 3.141592653589793 * .07*.07/4 * .32;
  Check(std::abs(lift.EffortLimit(1, 200e5)-expected) < 1e-8,
    "Pressure-to-torque conversion");
  Check(lift.EffortLimit(-1, 200e5) < lift.EffortLimit(1, 200e5),
    "Asymmetric double-acting force limits");
  bale_sim::Cylinder jaw{.04, .025, 1, 1, .12, 15.0/60000};
  Check(std::abs(jaw.EffortLimit(1, 100e5)-12566.370614359) < 1e-6,
    "Prismatic jaw force cap at 100 bar");
  const double allDemand = demand + 2*jaw.Demand(1);
  const double allScale = bale_sim::FlowScale(pump, allDemand);
  Check(std::abs(allDemand*allScale-pump) < 1e-12,
    "Grip jaws share pump flow with boom and tilt");
  Check(bale_sim::Valve(NAN)==0 && bale_sim::Valve(INFINITY)==0,
    "Reject nonfinite commands");
  Check(bale_sim::Valve(2)==1 && bale_sim::Valve(.03)==0,
    "Command clamp and deadband");
  std::cout << "Hydraulic flow and force checks passed\n";
}
