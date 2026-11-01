// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Terrain features from a 2.5D elevation grid, per SPEC.md (layer contract and cost model).
// Pure computation on Eigen matrices, no ROS types, so every function is unit tested.
//
// Grid convention: cell (i, j) is row i, column j of a matrix whose rows run along -x and
// columns along -y (grid_map's order after unwrapping its circular buffer). A neighbour at
// (i + di, j + dj) is displaced (-di * res, -dj * res) in metres. NaN marks an unknown cell.

#ifndef ROVER_TERRAIN_MAPPING__FEATURES_HPP_
#define ROVER_TERRAIN_MAPPING__FEATURES_HPP_

#include <Eigen/Core>

namespace rover_terrain_mapping
{

using Grid = Eigen::MatrixXf;

struct FeatureParams
{
  double resolution{0.05};       // m per cell
  double plane_radius{0.37};     // m, plane fit window: chassis scale, wheelbase / 2
  double step_radius{0.15};      // m, step window: wheel scale, a wheel radius
  // A plane fit needs this share of its disc observed, and its cells spread in both
  // directions (each principal standard deviation at least spread_fraction of the radius).
  // A single scan line fails the spread test instead of returning a wild normal.
  double min_fill{0.3};
  double spread_fraction{0.25};
  // Variance floor for weighting, m^2. Keeps one over-confident cell from dominating.
  double min_variance{1e-4};
};

struct FeatureLayers
{
  Grid slope;        // rad
  Grid slope_std;    // rad, from the plane fit covariance and the cell variances
  Grid roughness;    // m, RMS residual to the plane
  Grid step;         // m, max minus min in the step window
  Grid normal_x, normal_y, normal_z;
  Grid curvature;    // smallest over total eigenvalue of the window's 3D covariance
};

// Every feature for every cell. Cells whose window fails the fill or spread test get NaN in the
// plane-derived layers. step needs two known cells in its window.
FeatureLayers computeFeatures(const Grid & elevation, const Grid & variance, const FeatureParams & p);

struct PlaneFit
{
  bool valid{false};
  double a{0.0}, b{0.0}, c{0.0};    // z = a x + b y + c in metres about the centre cell
  double slope{0.0}, slope_std{0.0};
  double rms{0.0};
  double curvature{0.0};
  int count{0};
};

// Weighted least-squares plane about cell (i, j) over the cells within radius_cells.
PlaneFit fitPlane(
  const Grid & elevation, const Grid & variance, int i, int j, double radius_cells,
  const FeatureParams & p);

struct NegativeObstacleParams
{
  double resolution{0.05};
  double max_range{8.0};          // m, beyond this nothing is inferred
  // Unknown runs shorter than this along a ray are never flagged: smaller than any hole the
  // rover cares about. Wheel diameter scale.
  double min_gap{0.30};
  // The sensor's angular spacing between rows (rad). Two neighbouring rows land on flat ground
  // about d^2 * row_spacing / h apart at distance d from a sensor h above it, so an unknown run
  // up to gap_factor times that is a sampling gap, not a hole.
  double row_spacing{0.035};
  double gap_factor{2.0};
  int azimuth_bins{720};
  // A cell counts as visible-if-flat when its flat-continuation elevation angle clears the
  // terrain horizon by this much (tangent units).
  double horizon_margin{0.01};
  // The sensor's reach. A cell below its lowest beam (min_beam_angle, rad, negative is down)
  // or closer than min_range is never visible, so an unknown cell there is a blind zone, not a
  // hole. Defaults: the 16-ring roof lidar (lowest beam -15 deg) and FastDEM's range_min.
  double min_beam_angle{-0.2618};
  double min_range{1.0};
};

// Occlusion reasoning for negative obstacles. Marches rays out from the sensor at (si, sj) in
// fractional cell coordinates, height sensor_z. Along a ray it tracks the terrain horizon (the
// largest tangent of elevation angle seen so far). An unknown run that starts after an observed
// cell and that the sensor would have seen had the ground continued at that cell's height, and
// that is longer than both min_gap and gap_factor times the expected row spacing, is flagged.
// Unknown cells shadowed by higher terrain are not flagged: unknown is not the same as a hole.
// Returns a grid of 0 and 1, NaN where nothing was inferred (outside range, out of the grid).
Grid negativeObstacles(
  const Grid & elevation, double si, double sj, double sensor_z,
  const NegativeObstacleParams & p);

}  // namespace rover_terrain_mapping

#endif  // ROVER_TERRAIN_MAPPING__FEATURES_HPP_
