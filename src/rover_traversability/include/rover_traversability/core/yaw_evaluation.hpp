// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Per-cell heading evaluation (SPEC.md, cell cost): evaluate the pose at yaw_bins headings per
// cell and keep which headings are safe and the best one.
//
//   safe_yaw_mask     bit k set when heading k * 2 pi / yaw_bins is safe (u <= 1). Stored in a
//                     float layer, exact for yaw_bins <= 24.
//   best_yaw          rad, the heading with the lowest u, NaN if the cell is unknown
//   best_utilisation  that u. NaN if unknown, +inf if lethal at every heading.

#ifndef ROVER_TRAVERSABILITY__CORE__YAW_EVALUATION_HPP_
#define ROVER_TRAVERSABILITY__CORE__YAW_EVALUATION_HPP_

#include <Eigen/Core>

#include "rover_traversability/core/pose_evaluation.hpp"

namespace rover_traversability
{
namespace core
{

// limiting_check at the best heading: which utilisation set u. 0 tip, 1 clearance,
// 2 articulation, 3 step, 4 roughness, 5 negative obstacle under a wheel. NaN where unknown.
struct YawLayers
{
  Eigen::MatrixXf safe_yaw_mask, best_yaw, best_utilisation, limiting_check;
};

struct YawParams
{
  int yaw_bins{16};
  int threads{4};
};

YawLayers evaluateYaws(
  const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e, const YawParams & y);

// Largest finite value within radius_cells of each cell (grey dilation by a disc), NaN where
// the disc holds nothing known. Used to precompute roughness under the support polygon once
// per map instead of once per pose.
Eigen::MatrixXf discMax(const Eigen::MatrixXf & m, double radius_cells);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__YAW_EVALUATION_HPP_
