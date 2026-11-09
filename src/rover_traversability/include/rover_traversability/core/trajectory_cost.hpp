// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Terrain cost of controller rollouts (SPEC.md, controller cost), shared by the MPPI critic and
// its tests. Plain arrays, no MPPI types.
//
// Per evaluated point of a rollout, at its actual yaw:
//   w_terrain * u^2                          u from pose evaluation (yaw-dependent checks)
//   + collision_cost                         when the pose is lethal
//   + unknown_cost                           when a contact is unknown
//   + w_redirect * |change of travel direction| between consecutive points, since the wheels
//                                            need time to re-aim (max_wheel_turn_rate)
//   + w_uphill * max(0, slope along travel) * dt, a time penalty: uphill is slower

#ifndef ROVER_TRAVERSABILITY__CORE__TRAJECTORY_COST_HPP_
#define ROVER_TRAVERSABILITY__CORE__TRAJECTORY_COST_HPP_

#include <Eigen/Core>

#include "rover_traversability/core/pose_evaluation.hpp"

namespace rover_traversability
{
namespace core
{

struct TrajectoryCostParams
{
  double w_terrain{20.0};
  double collision_cost{1.0e6};
  double unknown_cost{1.0};
  // Utilisation below u_free costs nothing, above it w_terrain * ((u - u_free) / (1 - u_free))^2.
  // 0 keeps the plain w_terrain * u^2. A slope that is safe at the rollout's heading then costs
  // no more than flat ground, so the critic only acts where the heading matters.
  double u_free{0.0};
  // A pose whose tip utilisation exceeds this is a collision. Below 1 it matches the safety
  // monitor, which aborts within tip_margin of the tip limits, at about u_tip 0.82.
  double u_tip_lethal{1.0};
  double w_redirect{2.0};
  double w_uphill{5.0};
  int point_step{3};               // evaluate every n-th point of a rollout
  double min_speed{0.02};          // m/s, below it the travel direction is undefined
  // Threads over the batch. At 1000 rollouts x 17 points the critic took 24 to 34 ms of a 50 ms
  // control period on one core, and every TerrainCritic run missed its rate hundreds of times.
  int threads{1};
};

using Rollouts = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// x, y, yaw: rollout poses in the grid's frame. vx, vy: body velocities. All batch x steps.
// Returns one cost per rollout.
Eigen::VectorXf trajectoryCosts(
  const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e,
  const TrajectoryCostParams & c, const Rollouts & x, const Rollouts & y, const Rollouts & yaw,
  const Rollouts & vx, const Rollouts & vy, double dt, const EvalCache * cache = nullptr);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__TRAJECTORY_COST_HPP_
