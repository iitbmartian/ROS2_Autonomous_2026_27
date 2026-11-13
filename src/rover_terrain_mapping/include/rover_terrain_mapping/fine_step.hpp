// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Step from the fine local map carried into the coarse global map. At 0.2 m a rock smaller than
// a cell fills one cell: the Kalman cell estimate mixes it with ground and the trimmed step drops
// a single high cell as noise, so the global planner routes a wheel onto it. The local map at
// 0.05 m sees the rock, so each coarse cell remembers the worst fine step last seen inside it.
// Pure computation on Eigen matrices, no ROS types.

#ifndef ROVER_TERRAIN_MAPPING__FINE_STEP_HPP_
#define ROVER_TERRAIN_MAPPING__FINE_STEP_HPP_

#include <Eigen/Core>

namespace rover_terrain_mapping
{

// Square grid in its own frame, grid_map order: rows along -x, columns along -y, cell centres
// at whole indices, (center_x, center_y) the middle of the grid.
struct CellGrid
{
  int rows{0}, cols{0};
  double resolution{0.0};
  double center_x{0.0}, center_y{0.0};

  double x(int i) const {return center_x + (rows / 2.0 - (i + 0.5)) * resolution;}
  double y(int j) const {return center_y + (cols / 2.0 - (j + 0.5)) * resolution;}
  int row(double px) const;   // -1 outside
  int col(double py) const;   // -1 outside
  bool sameAs(const CellGrid & o) const;
};

// Planar pose of the fine grid's frame in the coarse grid's frame.
struct Planar
{
  double x{0.0}, y{0.0}, yaw{0.0};
};

// Updates memory (coarse-sized, NaN where nothing is known) with one fine step map. A coarse cell
// at least min_cover covered by finite fine cells takes their max, replacing what it held, so a
// rock that has gone (or was misread) clears on the next close look. A cell covered less keeps
// the larger of the old value and the new max, so the fine map's edge never erases a rock seen
// earlier from closer.
void fuseFineStep(
  Eigen::MatrixXf & memory, const CellGrid & coarse, const Eigen::MatrixXf & fine_step,
  const CellGrid & fine, const Planar & fine_in_coarse, double min_cover = 0.5);

// step = max(step, memory) where both are finite. Cells unknown in step stay unknown.
void applyFineStep(Eigen::MatrixXf & step, const Eigen::MatrixXf & memory);

}  // namespace rover_terrain_mapping

#endif  // ROVER_TERRAIN_MAPPING__FINE_STEP_HPP_
