// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/trajectory_cost.hpp"

#include <cmath>

namespace rover_traversability
{
namespace core
{

Eigen::VectorXf trajectoryCosts(
  const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e,
  const TrajectoryCostParams & c, const Rollouts & x, const Rollouts & y, const Rollouts & yaw,
  const Rollouts & vx, const Rollouts & vy, double dt, const EvalCache * cache)
{
  EvalCache local;
  if (cache == nullptr) {
    local = makeEvalCache(g, p, e);
    cache = &local;
  }
  const int batch = static_cast<int>(x.rows()), steps = static_cast<int>(x.cols());
  const int stride = std::max(1, c.point_step);
  Eigen::VectorXf out = Eigen::VectorXf::Zero(batch);
  for (int b = 0; b < batch; ++b) {
    double cost = 0.0;
    bool have_dir = false;
    double last_dir = 0.0;
    for (int t = 0; t < steps; t += stride) {
      const double v_x = vx(b, t), v_y = vy(b, t);
      const double speed = std::hypot(v_x, v_y);
      const double body_dir = std::atan2(v_y, v_x);
      if (speed >= c.min_speed) {
        const double dir = yaw(b, t) + body_dir;
        if (have_dir) {
          const double d = std::remainder(dir - last_dir, 2.0 * M_PI);
          cost += c.w_redirect * std::fabs(d);
        }
        last_dir = dir;
        have_dir = true;
      }

      const PoseResult r = evaluatePose(g, x(b, t), y(b, t), yaw(b, t), p, e, cache);
      if (r.status == PoseStatus::kUnknown) {
        cost += c.unknown_cost;
        continue;
      }
      if (r.status == PoseStatus::kLethal) {
        cost += c.collision_cost;
        continue;
      }
      cost += c.w_terrain * r.u * r.u;
      if (speed >= c.min_speed) {
        // Slope of the support plane along the direction of travel, body frame.
        const double s = std::tan(r.pitch) * std::cos(body_dir) +
          std::tan(r.roll) * std::sin(body_dir);
        if (s > 0.0) {cost += c.w_uphill * s * dt * stride;}
      }
    }
    out(b) = static_cast<float>(cost);
  }
  return out;
}

}  // namespace core
}  // namespace rover_traversability
