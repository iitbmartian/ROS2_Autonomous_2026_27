// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/yaw_evaluation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

namespace rover_traversability
{
namespace core
{

Eigen::MatrixXf discMax(const Eigen::MatrixXf & m, double radius_cells)
{
  const int rows = static_cast<int>(m.rows()), cols = static_cast<int>(m.cols());
  const int r = static_cast<int>(std::floor(radius_cells + 1e-6));
  const double r2 = (radius_cells + 1e-6) * (radius_cells + 1e-6);
  Eigen::MatrixXf out = Eigen::MatrixXf::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      float best = -std::numeric_limits<float>::infinity();
      for (int di = -r; di <= r; ++di) {
        for (int dj = -r; dj <= r; ++dj) {
          if (di * di + dj * dj > r2) {continue;}
          const int ii = i + di, jj = j + dj;
          if (ii < 0 || jj < 0 || ii >= rows || jj >= cols) {continue;}
          const float v = m(ii, jj);
          if (std::isfinite(v)) {best = std::max(best, v);}
        }
      }
      if (std::isfinite(best)) {out(i, j) = best;}
    }
  }
  return out;
}

YawLayers evaluateYaws(
  const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e, const YawParams & y)
{
  const int rows = g.rows(), cols = g.cols();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  YawLayers out;
  out.safe_yaw_mask = Eigen::MatrixXf::Constant(rows, cols, nan);
  out.best_yaw = Eigen::MatrixXf::Constant(rows, cols, nan);
  out.best_utilisation = Eigen::MatrixXf::Constant(rows, cols, nan);

  // With no fore-aft CG offset, heading k and k + bins/2 put the same wheels on the same cells
  // (front and rear swap), so every check gives the same answer and half the work suffices.
  const bool symmetric = p.cg_offset_x == 0.0 && y.yaw_bins % 2 == 0;
  const int evals = symmetric ? y.yaw_bins / 2 : y.yaw_bins;

  const EvalCache cache = makeEvalCache(g, p, e);
  auto work = [&](int r0, int r1) {
      for (int i = r0; i < r1; ++i) {
        for (int j = 0; j < cols; ++j) {
          if (!std::isfinite(g.elevation(i, j))) {continue;}
          const double x = g.x(i), yy = g.y(j);
          unsigned mask = 0;
          double best_u = std::numeric_limits<double>::infinity();
          double best_yaw = 0.0;
          bool any_known = false;
          for (int k = 0; k < evals; ++k) {
            const double yaw = 2.0 * M_PI * k / y.yaw_bins;
            const PoseResult r = evaluatePose(g, x, yy, yaw, p, e, &cache);
            if (r.status == PoseStatus::kUnknown) {continue;}
            any_known = true;
            if (r.status == PoseStatus::kOk) {
              mask |= 1u << k;
              if (symmetric) {mask |= 1u << (k + y.yaw_bins / 2);}
            }
            if (r.u < best_u) {best_u = r.u; best_yaw = yaw;}
          }
          if (!any_known) {continue;}
          out.safe_yaw_mask(i, j) = static_cast<float>(mask);
          out.best_utilisation(i, j) = static_cast<float>(best_u);
          out.best_yaw(i, j) = static_cast<float>(best_yaw);
        }
      }
    };

  const int n = std::max(1, std::min(y.threads, rows));
  std::vector<std::thread> pool;
  for (int t = 0; t < n; ++t) {
    pool.emplace_back(work, rows * t / n, rows * (t + 1) / n);
  }
  for (auto & th : pool) {th.join();}
  return out;
}

}  // namespace core
}  // namespace rover_traversability
