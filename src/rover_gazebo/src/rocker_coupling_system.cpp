// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Holds the rocker suspension's closed kinematic loops inside the physics step.
//
// rocker_coupling_node.py applies the same virtual constraints as torques, but over a ROS
// topic round trip (/joint_states in, an effort controller out). Under CPU load that loop
// lags the physics: at a real-time factor of 0.58 the rockers swung 17 to 36 deg and the body
// pitched up to 16.6 deg on flat ground (2026-11-08 probe). This system runs the identical
// law in PreUpdate, every physics step, on the joint state of that same step, so delay and
// machine load cannot open the loop. Selected with suspension:=plugin.
//
// It runs late in each step (priority 1000, default systems run at 0): this workspace's patched
// gz_ros2_control removes the JointForceCmd of every joint it does not drive in effort mode
// on each write, which would otherwise delete these torques before Physics applies them.
//
// For each constraint (joint a, joint b, ratio), with rest offset c captured once settled:
//   e    = (theta_a - ratio * theta_b) - c        residual, zero when satisfied
//   g    = K e + D edot                           constraint force
//   tau_a -= g,  tau_b += ratio * g               equal and opposite, no work when satisfied
// Torques are clamped per joint at max_torque.
//
// SDF:
//   <plugin filename="rover_gazebo_rocker_coupling" name="rover_gazebo::RockerCouplingSystem">
//     <stiffness>8000</stiffness> <damping>200</damping> <max_torque>400</max_torque>
//     <rest_capture_delay>2.5</rest_capture_delay>
//     <constraint a="BLS_joint" b="FLS_joint" ratio="-0.577165"/>
//     ...
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Console.hh>

#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace rover_gazebo
{

class RockerCouplingSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigurePriority,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 1000;}

  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    k_ = sdf->Get<double>("stiffness", k_).first;
    d_ = sdf->Get<double>("damping", d_).first;
    max_torque_ = sdf->Get<double>("max_torque", max_torque_).first;
    rest_delay_ = sdf->Get<double>("rest_capture_delay", rest_delay_).first;
    debug_ = sdf->Get<bool>("debug", debug_).first;
    for (auto e = sdf->FindElement("constraint"); e; e = e->GetNextElement("constraint")) {
      Constraint c;
      c.a = e->Get<std::string>("a");
      c.b = e->Get<std::string>("b");
      c.ratio = e->Get<double>("ratio");
      constraints_.push_back(c);
    }
    for (const auto & c : constraints_) {
      for (const auto & name : {c.a, c.b}) {
        if (joints_.count(name)) {continue;}
        const auto je = model_.JointByName(ecm, name);
        if (je == gz::sim::kNullEntity) {
          gzerr << "RockerCouplingSystem: joint " << name << " not found, disabled\n";
          constraints_.clear();
          return;
        }
        gz::sim::Joint j(je);
        j.EnablePositionCheck(ecm, true);
        j.EnableVelocityCheck(ecm, true);
        joints_.emplace(name, j);
      }
      gzmsg << "RockerCouplingSystem: " << c.a << " = " << c.ratio << " x " << c.b << "\n";
    }
    gzmsg << "RockerCouplingSystem: K " << k_ << ", D " << d_ << ", max " << max_torque_
          << " N m, in the physics step\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || constraints_.empty()) {return;}
    std::map<std::string, double> pos, vel;
    for (auto & [name, j] : joints_) {
      const auto p = j.Position(ecm);
      const auto v = j.Velocity(ecm);
      if (!p || p->empty() || !v || v->empty()) {return;}   // first step: components not filled yet
      pos[name] = p->front();
      vel[name] = v->front();
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (!rest_captured_) {
      // The wheels hold the rover still, so a CAD-pose reference would push forever against
      // the small residual left once it settles. Capture the settled pose instead, as the node does.
      if (first_t_ < 0.0) {first_t_ = t;}
      if (t - first_t_ < rest_delay_) {return;}
      for (auto & c : constraints_) {c.rest = pos[c.a] - c.ratio * pos[c.b];}
      rest_captured_ = true;
      gzmsg << "RockerCouplingSystem: rest pose captured at t = " << t << " s\n";
    }
    std::map<std::string, double> tau;
    for (const auto & [name, j] : joints_) {tau[name] = 0.0;}
    for (const auto & c : constraints_) {
      const double e = (pos[c.a] - c.ratio * pos[c.b]) - c.rest;
      const double edot = vel[c.a] - c.ratio * vel[c.b];
      const double g = k_ * e + d_ * edot;
      tau[c.a] -= g;
      tau[c.b] += c.ratio * g;
    }
    for (auto & [name, j] : joints_) {
      j.SetForce(ecm, {std::clamp(tau[name], -max_torque_, max_torque_)});
    }
    if (debug_ && t - last_print_ >= 1.0) {
      last_print_ = t;
      std::string line;
      for (const auto & c : constraints_) {
        line += c.a + "-" + c.b + " e " +
          std::to_string((pos[c.a] - c.ratio * pos[c.b]) - c.rest) + "  ";
      }
      gzmsg << "RockerCouplingSystem t " << t << ": " << line << "\n";
    }
  }

private:
  struct Constraint
  {
    std::string a, b;
    double ratio{0.0};
    double rest{0.0};
  };
  gz::sim::Model model_{gz::sim::kNullEntity};
  std::vector<Constraint> constraints_;
  std::map<std::string, gz::sim::Joint> joints_;
  double k_{8000.0}, d_{200.0}, max_torque_{400.0}, rest_delay_{2.5};
  double first_t_{-1.0}, last_print_{-1.0};
  bool debug_{false};
  bool rest_captured_{false};
};

}  // namespace rover_gazebo

GZ_ADD_PLUGIN(
  rover_gazebo::RockerCouplingSystem, gz::sim::System,
  rover_gazebo::RockerCouplingSystem::ISystemConfigurePriority,
  rover_gazebo::RockerCouplingSystem::ISystemConfigure,
  rover_gazebo::RockerCouplingSystem::ISystemPreUpdate)
