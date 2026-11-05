// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Pose evaluation, the utilisation cost model of SPEC.md, shared by the costmap layer, the
// feature node and the MPPI critic. For a pose (x, y, yaw) on a 2.5D terrain grid it places the
// four wheel contacts, fits the support plane through them and computes, for each limit, the
// fraction of the allowed amount the pose uses (0 benign, 1 at the limit):
//
//   u_tip    stability margin: (margin_flat - margin) / (margin_flat - stability_margin_min)
//   u_clear  belly clearance:  highest terrain under the belly above the plane / ground_clearance
//   u_art    articulation:     |rocker_left - rocker_right| / 2 / rocker_max
//   u_step   wheel step:       max step layer at the contacts / step_max
//   u_rough  roughness:        roughness under the support polygon / roughness_max
//
// u = max of all. Lethal when u > 1 or a contact sits on a negative obstacle. Unknown when a
// contact height is unknown.
//
// Map uncertainty (CVaR-style, risk_kappa): every height read is taken as z +- kappa * sigma in
// the direction that makes the check stricter. Contact-derived angles are widened by kappa times
// their propagated standard deviation, belly heights raised by kappa * sigma.
//
// Grid convention as everywhere in the stack: rows along -x, columns along -y, NaN unknown.

#ifndef ROVER_TRAVERSABILITY__CORE__POSE_EVALUATION_HPP_
#define ROVER_TRAVERSABILITY__CORE__POSE_EVALUATION_HPP_

#include <vector>

#include <Eigen/Core>

#include "rover_traversability/core/physical_params.hpp"

namespace rover_traversability
{
namespace core
{

// A terrain grid in a map frame: geometry plus the layers pose evaluation reads.
struct TerrainGrid
{
  double resolution{0.05};
  double center_x{0.0}, center_y{0.0};      // map frame position of the grid centre
  Eigen::MatrixXf elevation;                 // m
  Eigen::MatrixXf variance;                  // m^2, may be empty (no uncertainty)
  Eigen::MatrixXf step;                      // m, may be empty
  Eigen::MatrixXf roughness;                 // m, may be empty
  Eigen::MatrixXf negative_obstacle;         // 1 at holes, may be empty

  int rows() const {return static_cast<int>(elevation.rows());}
  int cols() const {return static_cast<int>(elevation.cols());}
  double row(double x) const {return (rows() * resolution / 2.0 - (x - center_x)) / resolution - 0.5;}
  double col(double y) const {return (cols() * resolution / 2.0 - (y - center_y)) / resolution - 0.5;}
  double x(double r) const {return center_x + rows() * resolution / 2.0 - (r + 0.5) * resolution;}
  double y(double c) const {return center_y + cols() * resolution / 2.0 - (c + 0.5) * resolution;}
  // Nearest cell, false outside the grid.
  bool cell(double x, double y, int & i, int & j) const;
};

struct EvalParams
{
  double risk_kappa{1.0};     // standard deviations added in the strict direction
  // Belly: the chassis between the wheels, approximated by the circumscribed disc of the
  // rectangle (wheelbase - 2 wheel_radius) x (track - wheel_width). Larger than the belly, so
  // strict. Computed from PhysicalParams unless set.
  double belly_radius{-1.0};
  // Roughness window. 0 reads the centre cell: the roughness layer is already the RMS residual
  // of a plane fitted over a wheelbase / 2 disc, the support area, and widening it again counted
  // that area twice (0.8 m unsafe margin around tall rocks). -1 is the wheel rectangle's
  // circumscribed disc, for a roughness layer measured at a smaller scale.
  double rough_radius{0.0};
  // The belly disc and the roughness disc do not depend on yaw. The MPPI critic leaves them to
  // the costmap, which already applies them at every cell, and evaluates only the yaw-dependent
  // checks along each rollout.
  bool check_clearance{true};
  bool check_roughness{true};
};

enum class PoseStatus { kOk, kLethal, kUnknown };

struct PoseResult
{
  PoseStatus status{PoseStatus::kUnknown};
  double u{0.0};
  double u_tip{0.0}, u_clear{0.0}, u_art{0.0}, u_step{0.0}, u_rough{0.0};
  double roll{0.0}, pitch{0.0};      // rad, support plane in the body frame (pitch + nose up)
  double margin{0.0};                // rad, stability margin
  double clearance_height{0.0};      // m, highest belly terrain above the plane
  bool negative_contact{false};
};

// Stability margin on flat ground: the smallest tip-over angle about the support polygon edges.
double flatStabilityMargin(const PhysicalParams & p);

// Per-map values that do not change from pose to pose. Build once with makeEvalCache when
// evaluating many poses on one grid. The grid must outlive the cache.
struct EvalCache
{
  double flat_margin{0.0};
  double belly_radius{0.0};
  struct Offset {int di, dj; double dx, dy;};
  std::vector<Offset> belly;        // cells of the belly disc, offsets in metres
  Eigen::MatrixXf sigma;            // sqrt(variance), empty without variance
};

EvalCache makeEvalCache(const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e);

PoseResult evaluatePose(
  const TerrainGrid & g, double x, double y, double yaw, const PhysicalParams & p,
  const EvalParams & e = EvalParams(), const EvalCache * cache = nullptr);

// Stability margin (rad) of a CG at body offset (cx, cy, h above the support plane) for a
// rectangular support polygon (half extents hx, hy) on a plane with body-frame slopes
// dz/dx = a and dz/dy = b. Exact 3D: for each edge, the angle between the edge-to-CG vector
// (perpendicular to the edge) and the vertical, signed positive while the CG is inside.
double stabilityMargin(double a, double b, double hx, double hy, double cx, double cy, double h);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__POSE_EVALUATION_HPP_
