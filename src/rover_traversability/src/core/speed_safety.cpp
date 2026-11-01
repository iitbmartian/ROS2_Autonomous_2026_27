// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/speed_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rover_traversability
{
namespace core
{

std::vector<Pose2> predictPoses(
  const Pose2 & start, double vx, double vy, double wz, double horizon, double dt)
{
  std::vector<Pose2> out;
  Pose2 s = start;
  for (double t = dt; t <= horizon + 1e-9; t += dt) {
    // Midpoint heading over the step keeps arcs accurate at coarse dt.
    const double mid = s.yaw + 0.5 * wz * dt;
    s.x += (vx * std::cos(mid) - vy * std::sin(mid)) * dt;
    s.y += (vx * std::sin(mid) + vy * std::cos(mid)) * dt;
    s.yaw += wz * dt;
    out.push_back(s);
  }
  return out;
}

double worstUtilisation(
  const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e,
  const std::vector<Pose2> & poses, double unknown_u, const EvalCache * cache)
{
  double worst = 0.0;
  for (const auto & q : poses) {
    const auto r = evaluatePose(g, q.x, q.y, q.yaw, p, e, cache);
    if (r.status == PoseStatus::kUnknown) {
      worst = std::max(worst, unknown_u);
    } else if (r.status == PoseStatus::kLethal) {
      return std::numeric_limits<double>::infinity();
    } else {
      worst = std::max(worst, r.u);
    }
  }
  return worst;
}

double speedLimit(double u_ahead, const GovernorParams & g)
{
  const double u = std::isfinite(u_ahead) ? std::max(0.0, u_ahead) : 1.0e9;
  return std::max(g.v_min, g.v_nominal * (1.0 - g.alpha * u));
}

void tipLimits(const PhysicalParams & p, double & roll_limit, double & pitch_limit)
{
  roll_limit = std::atan2(p.track / 2.0 - std::fabs(p.cg_offset_y), p.cg_height) -
    p.stability_margin_min;
  pitch_limit = std::atan2(p.wheelbase / 2.0 - std::fabs(p.cg_offset_x), p.cg_height) -
    p.stability_margin_min;
}

bool tipRisk(double roll, double pitch, const PhysicalParams & p, double tip_margin)
{
  double rl, pl;
  tipLimits(p, rl, pl);
  return std::fabs(roll) >= rl - tip_margin || std::fabs(pitch) >= pl - tip_margin;
}

}  // namespace core
}  // namespace rover_traversability
