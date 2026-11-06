// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Speed governor and safety monitor logic (SPEC.md, speed and safety). Plain functions, tested
// without ROS. The nodes in src/nodes wire them to topics.

#ifndef ROVER_TRAVERSABILITY__CORE__SPEED_SAFETY_HPP_
#define ROVER_TRAVERSABILITY__CORE__SPEED_SAFETY_HPP_

#include <vector>

#include "rover_traversability/core/pose_evaluation.hpp"

namespace rover_traversability
{
namespace core
{

struct Pose2 {double x, y, yaw;};

// Poses along a constant body twist (vx, vy, wz) from start, every dt for horizon seconds,
// start excluded.
std::vector<Pose2> predictPoses(
  const Pose2 & start, double vx, double vy, double wz, double horizon, double dt);

// Worst utilisation over the poses: unknown poses count as unknown_u, lethal ones as +inf.
double worstUtilisation(
  const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e,
  const std::vector<Pose2> & poses, double unknown_u, const EvalCache * cache = nullptr);

struct GovernorParams
{
  double v_nominal{0.25};    // m/s
  double v_min{0.05};        // m/s
  double alpha{0.8};         // speed taken off per unit of utilisation ahead
  double u_free{0.0};        // utilisation below which the rover keeps v_nominal, in [0, 1)
};

// v_max = v_nominal * (1 - alpha * u'), floored at v_min, with u' = (u_ahead - u_free) /
// (1 - u_free) clamped at 0, so easy ground runs at full speed and u' still reaches 1 where
// u_ahead does. u_free 0 is the SPEC formula. A lethal pose ahead (u > 1) floors it too:
// stopping is the safety monitor's job, the governor only slows.
double speedLimit(double u_ahead, const GovernorParams & g);

// Roll and pitch limits (rad): flat tip-over angles minus stability_margin_min.
void tipLimits(const PhysicalParams & p, double & roll_limit, double & pitch_limit);

// True when |roll| or |pitch| comes within tip_margin of its limit.
bool tipRisk(double roll, double pitch, const PhysicalParams & p, double tip_margin);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__SPEED_SAFETY_HPP_
