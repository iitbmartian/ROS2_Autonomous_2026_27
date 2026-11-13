// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_terrain_mapping/fine_step.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rover_terrain_mapping
{

int CellGrid::row(double px) const
{
  const double r = rows / 2.0 - (px - center_x) / resolution;
  const int i = static_cast<int>(std::floor(r));
  return (r >= 0.0 && i < rows) ? i : -1;
}

int CellGrid::col(double py) const
{
  const double c = cols / 2.0 - (py - center_y) / resolution;
  const int j = static_cast<int>(std::floor(c));
  return (c >= 0.0 && j < cols) ? j : -1;
}

bool CellGrid::sameAs(const CellGrid & o) const
{
  return rows == o.rows && cols == o.cols && std::fabs(resolution - o.resolution) < 1e-9 &&
         std::fabs(center_x - o.center_x) < 1e-6 && std::fabs(center_y - o.center_y) < 1e-6;
}

void fuseFineStep(
  Eigen::MatrixXf & memory, const CellGrid & coarse, const Eigen::MatrixXf & fine_step,
  const CellGrid & fine, const Planar & T, double min_cover)
{
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  if (memory.rows() != coarse.rows || memory.cols() != coarse.cols) {
    memory.setConstant(coarse.rows, coarse.cols, kNaN);
  }
  Eigen::MatrixXf mx = Eigen::MatrixXf::Constant(coarse.rows, coarse.cols, kNaN);
  Eigen::MatrixXi n = Eigen::MatrixXi::Zero(coarse.rows, coarse.cols);
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  for (int i = 0; i < fine_step.rows(); ++i) {
    const double fx = fine.x(i);
    for (int j = 0; j < fine_step.cols(); ++j) {
      const float v = fine_step(i, j);
      if (!std::isfinite(v)) {continue;}
      const double fy = fine.y(j);
      const int ci = coarse.row(T.x + c * fx - s * fy);
      const int cj = coarse.col(T.y + s * fx + c * fy);
      if (ci < 0 || cj < 0) {continue;}
      float & m = mx(ci, cj);
      m = std::isfinite(m) ? std::max(m, v) : v;
      ++n(ci, cj);
    }
  }
  const double ratio = coarse.resolution / fine.resolution;
  const int full = std::max(1, static_cast<int>(std::lround(min_cover * ratio * ratio)));
  for (int i = 0; i < coarse.rows; ++i) {
    for (int j = 0; j < coarse.cols; ++j) {
      if (n(i, j) == 0) {continue;}
      float & m = memory(i, j);
      m = (n(i, j) >= full || !std::isfinite(m)) ? mx(i, j) : std::max(m, mx(i, j));
    }
  }
}

void applyFineStep(Eigen::MatrixXf & step, const Eigen::MatrixXf & memory)
{
  if (step.rows() != memory.rows() || step.cols() != memory.cols()) {return;}
  for (int i = 0; i < step.rows(); ++i) {
    for (int j = 0; j < step.cols(); ++j) {
      const float m = memory(i, j);
      float & v = step(i, j);
      if (std::isfinite(v) && std::isfinite(m)) {v = std::max(v, m);}
    }
  }
}

}  // namespace rover_terrain_mapping
