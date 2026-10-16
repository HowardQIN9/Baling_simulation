#include "HydraulicModel.hh"

#include <array>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace bale_sim
{
class LoaderHydraulics : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  using Clock = std::chrono::steady_clock;
  struct Axis
  {
    gz::sim::Joint joint;
    Cylinder cylinder;
    double lower, upper, kp, ki, kd;
    double target = 0, integral = 0, spool = 0, lastEffort = 0;
    bool initialized = false, commanded = false, linear = false;
    double pressureLimit = 200e5, previousValve = 0;
    gz::transport::Node::Publisher publisher;
  };
  std::array<Axis, 4> axes;
  unsigned axisCount = 2;
  std::mutex mutex;
  std::array<double, 4> commands{};
  std::array<Clock::time_point, 4> received{};
  double pump = 86.0 / 60000.0, relief = 200e5, timeout = 0.5;
  double publishElapsed = 0;
  bool ready = false;
  // Destroy subscriptions before the callback-owned command state and mutex.
  gz::transport::Node node;

  void Receive(unsigned index, const gz::msgs::Double &msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    commands[index] = Valve(msg.data());
    received[index] = Clock::now();
  }

  public: void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    try
    {
      pump = sdf->Get<double>("pump_flow_lpm", 86).first / 60000.0;
      relief = sdf->Get<double>("relief_pressure_bar", 200).first * 1e5;
      timeout = sdf->Get<double>("command_timeout", 0.5).first;
      if (!(pump > 0 && relief > 0 && timeout > 0) ||
          !std::isfinite(pump + relief + timeout))
        throw std::runtime_error("Invalid pump, pressure, or timeout");
      gz::sim::Model model(entity);
      const std::array<std::string, 4> names{"lift", "tilt", "grip_left", "grip_right"};
      if (sdf->FindElement("grip_left") || sdf->FindElement("grip_right")) axisCount = 4;
      for (unsigned i = 0; i < axisCount; ++i)
      {
        const auto cfg = sdf->FindElement(names[i]);
        if (!cfg) throw std::runtime_error("Missing actuator configuration");
        auto &a = axes[i];
        a.joint = gz::sim::Joint(model.JointByName(ecm,
          cfg->Get<std::string>("joint")));
        if (!a.joint.Valid(ecm)) throw std::runtime_error("Missing loader joint");
        a.cylinder = {cfg->Get<double>("bore"), cfg->Get<double>("rod"),
          cfg->Get<double>("cylinders"), cfg->Get<double>("moment_arm"),
          cfg->Get<double>("max_speed"), cfg->Get<double>("flow_lpm") / 60000.0};
        a.linear = cfg->Get<bool>("linear", false).first;
        a.pressureLimit = std::min(relief,
          cfg->Get<double>("pressure_limit_bar", relief/1e5).first * 1e5);
        a.lower = cfg->Get<double>("lower");
        a.upper = cfg->Get<double>("upper");
        a.kp = cfg->Get<double>("p_gain");
        a.ki = cfg->Get<double>("i_gain");
        a.kd = cfg->Get<double>("d_gain");
        const auto &c = a.cylinder;
        if (!(c.bore > c.rod && c.rod > 0 && c.count > 0 && c.arm > 0 &&
              c.maxSpeed > 0 && c.branchFlow > 0 && a.lower < a.upper &&
              a.kp > 0 && a.ki >= 0 && a.kd > 0 && a.pressureLimit > 0) ||
            !std::isfinite(c.bore+c.rod+c.count+c.arm+c.maxSpeed+c.branchFlow+
              a.lower+a.upper+a.kp+a.ki+a.kd+a.pressureLimit))
          throw std::runtime_error("Invalid actuator parameters");
        a.joint.EnablePositionCheck(ecm);
        a.joint.EnableVelocityCheck(ecm);
        const std::string base = "/tractor/loader/" + names[i];
        // Callbacks change only mutex-protected commands, never physics state.
        if (i == 0)
          node.Subscribe(base + "/valve", &LoaderHydraulics::Lift, this);
        else if (i == 1)
          node.Subscribe(base + "/valve", &LoaderHydraulics::Tilt, this);
        else if (i == 2)
          node.Subscribe("/tractor/loader/grip/valve", &LoaderHydraulics::Grip, this);
        a.publisher = node.Advertise<gz::msgs::Double_V>(base + "/state");
      }
      ready = true;
      gzmsg << "Loader hydraulics: " << pump * 60000 << " L/min, "
            << relief / 1e5 << " bar (provisional equivalent-actuator model)\n";
    }
    catch (const std::exception &e)
    {
      gzerr << "Loader hydraulics disabled: " << e.what() << '\n';
    }
  }

  public: void Lift(const gz::msgs::Double &msg) { Receive(0, msg); }
  public: void Tilt(const gz::msgs::Double &msg) { Receive(1, msg); }

  public: void Grip(const gz::msgs::Double &msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    commands[2] = commands[3] = Valve(msg.data());
    received[2] = received[3] = Clock::now();
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready || info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    std::array<double, 4> valve{};
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (unsigned i = 0; i < axisCount; ++i)
        valve[i] = std::chrono::duration<double>(Clock::now()-received[i]).count()
          <= timeout ? commands[i] : 0.0;
    }
    double demand = 0;
    for (unsigned i = 0; i < axisCount; ++i)
    {
      auto &a = axes[i];
      // First-order valve response. Missing commands return gradually to neutral.
      a.spool += (valve[i] - a.spool) * (1 - std::exp(-dt / 0.12));
      if (std::abs(a.spool) < 1e-6) a.spool = 0;
      demand += a.cylinder.Demand(a.spool);
    }
    const double scale = FlowScale(pump, demand);
    publishElapsed += dt;
    const bool publish = publishElapsed >= 0.05;
    for (unsigned i = 0; i < axisCount; ++i)
    {
      auto &a = axes[i];
      const auto position = a.joint.Position(ecm);
      const auto velocity = a.joint.Velocity(ecm);
      if (!position || !velocity || position->empty() || velocity->empty()) continue;
      const double q = position->at(0), v = velocity->at(0);
      if (!a.initialized)
      {
        a.target = std::clamp(q, a.lower, a.upper);
        a.initialized = true;
      }
      if (valve[i] == 0 && a.commanded)
      {
        // Closing the valve captures the current position instead of completing
        // an old motion reference. Retain torque for a bumpless loaded hold.
        a.target = std::clamp(q, a.lower, a.upper);
        a.integral = a.lastEffort;
      }
      // Reversing a clamping jaw must release stored holding pressure.
      if (a.linear && valve[i] < 0 && a.previousValve >= 0)
      {
        a.integral = 0;
        a.target = std::clamp(q, a.lower, a.upper);
      }
      a.previousValve = valve[i];
      a.commanded = valve[i] != 0;
      const double speed = a.cylinder.Speed(a.spool, scale);
      // Limit tracking lag under overload: no accumulating unreachable setpoints.
      a.target = std::clamp(a.target + speed * dt,
        std::max(a.lower, q - 0.12), std::min(a.upper, q + 0.12));
      const double error = a.target - q;
      const double proposedIntegral = a.integral + a.ki * error * dt;
      const double raw = a.kp * error + proposedIntegral + a.kd * (speed - v);
      const double limit = a.cylinder.EffortLimit(raw, a.pressureLimit);
      const double effort = std::clamp(raw, -limit, limit);
      // Freeze integral when it would drive further into pressure saturation.
      if (std::abs(raw) <= limit || error * raw < 0)
        a.integral = std::clamp(proposedIntegral, -limit, limit);
      a.joint.SetForce(ecm, {effort});
      a.lastEffort = effort;
      if (publish)
      {
        gz::msgs::Double_V msg;
        // q, qdot, target, effort (Nm or N), pressure (bar), flow (L/min), sim time.
        for (double value : {q, v, a.target, effort,
            std::abs(effort) / (a.cylinder.count * a.cylinder.Area(effort >= 0) *
              a.cylinder.arm) / 1e5,
            a.cylinder.Demand(a.spool) * scale * 60000,
            std::chrono::duration<double>(info.simTime).count()})
          msg.add_data(value);
        a.publisher.Publish(msg);
      }
    }
    if (publish) publishElapsed = 0;
  }

  public: void Reset(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    commands = {};
    received = {};
    publishElapsed = 0;
    for (auto &a : axes)
    {
      a.initialized = false;
      a.commanded = false;
      a.lastEffort = a.previousValve = 0;
      a.integral = a.spool = 0;
    }
  }
};
}

GZ_ADD_PLUGIN(bale_sim::LoaderHydraulics, gz::sim::System,
  bale_sim::LoaderHydraulics::ISystemConfigure,
  bale_sim::LoaderHydraulics::ISystemPreUpdate,
  bale_sim::LoaderHydraulics::ISystemReset)
