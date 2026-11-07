// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_terrain_mapping/features.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace rover_terrain_mapping
{

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

std::vector<std::pair<int, int>> discOffsets(double radius_cells)
{
  std::vector<std::pair<int, int>> out;
  // radius / resolution is often a whole number that floating point lands just under
  // (0.15 / 0.05 = 2.9999999999999996), which would drop the disc's edge cells.
  const double rc = radius_cells + 1e-6;
  const int r = static_cast<int>(std::floor(rc));
  const double r2 = rc * rc;
  for (int di = -r; di <= r; ++di) {
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj <= r2) {
        out.emplace_back(di, dj);
      }
    }
  }
  return out;
}

PlaneFit fitPlaneOffsets(
  const Grid & z, const Grid & var, int i, int j, const std::vector<std::pair<int, int>> & disc,
  double radius_m, const FeatureParams & p)
{
  PlaneFit f;
  const int rows = static_cast<int>(z.rows());
  const int cols = static_cast<int>(z.cols());
  const double res = p.resolution;

  // Accumulate in double. Coordinates in metres about the centre cell.
  Eigen::Matrix3d AtWA = Eigen::Matrix3d::Zero();
  Eigen::Vector3d AtWz = Eigen::Vector3d::Zero();
  double sw = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(disc.size());
  for (const auto & [di, dj] : disc) {
    const int ii = i + di, jj = j + dj;
    if (ii < 0 || jj < 0 || ii >= rows || jj >= cols) {continue;}
    const float zv = z(ii, jj);
    if (!std::isfinite(zv)) {continue;}
    const double x = -di * res, y = -dj * res;
    const float vv = var(ii, jj);
    const double w = 1.0 / std::max(std::isfinite(vv) ? static_cast<double>(vv) : 1e-2,
                                    p.min_variance);
    const Eigen::Vector3d a(x, y, 1.0);
    AtWA.noalias() += w * a * a.transpose();
    AtWz.noalias() += w * zv * a;
    sw += 1.0; sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
    pts.emplace_back(x, y, zv);
  }
  f.count = static_cast<int>(pts.size());
  if (f.count < 4 || f.count < p.min_fill * static_cast<double>(disc.size())) {
    return f;
  }

  // Spread: both principal standard deviations of the cell positions must reach a share of the
  // radius, or the cells are close to a line and the plane's tilt across it is undetermined.
  const double mx = sx / sw, my = sy / sw;
  Eigen::Matrix2d cxy;
  cxy << sxx / sw - mx * mx, sxy / sw - mx * my, sxy / sw - mx * my, syy / sw - my * my;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(cxy);
  const double min_std = std::sqrt(std::max(es2.eigenvalues()(0), 0.0));
  if (min_std < p.spread_fraction * radius_m) {
    return f;
  }

  const Eigen::Matrix3d cov = AtWA.inverse();     // weights are inverse variances
  const Eigen::Vector3d sol = cov * AtWz;
  f.a = sol(0); f.b = sol(1); f.c = sol(2);
  const double g = std::hypot(f.a, f.b);
  f.slope = std::atan(g);
  // First-order propagation of the (a, b) covariance through slope = atan(|grad|).
  const Eigen::Matrix2d cab = cov.topLeftCorner<2, 2>();
  if (g > 1e-9) {
    const Eigen::Vector2d J(f.a / (g * (1.0 + g * g)), f.b / (g * (1.0 + g * g)));
    f.slope_std = std::sqrt(std::max(J.dot(cab * J), 0.0));
  } else {
    f.slope_std = std::sqrt(std::max(cab.trace() / 2.0, 0.0));
  }

  double r2 = 0.0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & q : pts) {
    const double r = q.z() - (f.a * q.x() + f.b * q.y() + f.c);
    r2 += r * r;
    mean += q;
  }
  f.rms = std::sqrt(r2 / f.count);
  mean /= f.count;
  Eigen::Matrix3d c3 = Eigen::Matrix3d::Zero();
  for (const auto & q : pts) {
    const Eigen::Vector3d d = q - mean;
    c3.noalias() += d * d.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es3(c3 / f.count);
  const double tr = es3.eigenvalues().sum();
  f.curvature = tr > 0.0 ? std::max(es3.eigenvalues()(0), 0.0) / tr : 0.0;
  f.valid = true;
  return f;
}

}  // namespace

Grid median3(const Grid & z)
{
  const int rows = static_cast<int>(z.rows());
  const int cols = static_cast<int>(z.cols());
  Grid out(rows, cols);
  std::array<float, 9> v{};
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (!std::isfinite(z(i, j))) {out(i, j) = z(i, j); continue;}
      int n = 0;
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          const int ii = i + di, jj = j + dj;
          if (ii < 0 || jj < 0 || ii >= rows || jj >= cols) {continue;}
          if (std::isfinite(z(ii, jj))) {v[n++] = z(ii, jj);}
        }
      }
      // A median of few cells is not a filter, so thin neighbourhoods keep the raw value.
      if (n < 5) {out(i, j) = z(i, j); continue;}
      std::nth_element(v.begin(), v.begin() + n / 2, v.begin() + n);
      float m = v[n / 2];
      if (n % 2 == 0) {m = 0.5f * (m + *std::max_element(v.begin(), v.begin() + n / 2));}
      out(i, j) = m;
    }
  }
  return out;
}

PlaneFit fitPlane(
  const Grid & elevation, const Grid & variance, int i, int j, double radius_cells,
  const FeatureParams & p)
{
  return fitPlaneOffsets(
    elevation, variance, i, j, discOffsets(radius_cells), radius_cells * p.resolution, p);
}

FeatureLayers computeFeatures(const Grid & z, const Grid & var, const FeatureParams & p)
{
  const int rows = static_cast<int>(z.rows());
  const int cols = static_cast<int>(z.cols());
  FeatureLayers out;
  for (Grid * g : {&out.slope, &out.slope_std, &out.roughness, &out.step, &out.normal_x,
      &out.normal_y, &out.normal_z, &out.curvature})
  {
    g->setConstant(rows, cols, kNaN);
  }
  const double plane_cells = p.plane_radius / p.resolution;
  const auto plane_disc = discOffsets(plane_cells);
  const auto step_disc = discOffsets(p.step_radius / p.resolution);
  const Grid zm = p.step_median ? median3(z) : Grid();

  // Second highest minus second lowest over the wheel disc, heights taken relative to the
  // plane gradient (ga, gb). Plain max minus min with 2 or 3 cells, NaN with fewer.
  auto stepOf = [&](const Grid & h, int i, int j, double ga, double gb) {
      const double inf = std::numeric_limits<double>::infinity();
      double lo1 = inf, lo2 = inf, hi1 = -inf, hi2 = -inf;
      int n = 0;
      for (const auto & [di, dj] : step_disc) {
        const int ii = i + di, jj = j + dj;
        if (ii < 0 || jj < 0 || ii >= rows || jj >= cols) {continue;}
        const float v = h(ii, jj);
        if (!std::isfinite(v)) {continue;}
        const double r = v - (ga * (-di * p.resolution) + gb * (-dj * p.resolution));
        if (r < lo1) {lo2 = lo1; lo1 = r;} else if (r < lo2) {lo2 = r;}
        if (r > hi1) {hi2 = hi1; hi1 = r;} else if (r > hi2) {hi2 = r;}
        ++n;
      }
      if (n >= 4) {return hi2 - lo2;}
      if (n >= 2) {return hi1 - lo1;}
      return std::numeric_limits<double>::quiet_NaN();
    };

  // Every cell is independent and writes only its own output, so rows split across threads.
  auto work = [&](int r0, int r1) {
  for (int i = r0; i < r1; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (!std::isfinite(z(i, j))) {continue;}

      const PlaneFit f = fitPlaneOffsets(z, var, i, j, plane_disc, p.plane_radius, p);

      // Step relative to the chassis-scale plane, so a smooth ramp is not a step. An abrupt
      // jump keeps its height, less one cell of plane slope. Raw heights where no plane fits.
      const double ga = f.valid ? f.a : 0.0, gb = f.valid ? f.b : 0.0;
      // Second highest minus second lowest: one stray cell is sensor noise, any real feature
      // under a wheel covers two or more cells. Plain max minus min with fewer than 4 cells.
      const double raw = stepOf(z, i, j, ga, gb);
      if (std::isfinite(raw)) {
        const double st = p.step_median ?
          std::max(stepOf(zm, i, j, ga, gb), raw - p.step_noise_floor) : raw;
        out.step(i, j) = static_cast<float>(std::max(st, 0.0));
      }

      if (!f.valid) {continue;}
      out.slope(i, j) = static_cast<float>(f.slope);
      out.slope_std(i, j) = static_cast<float>(f.slope_std);
      out.roughness(i, j) = static_cast<float>(f.rms);
      out.curvature(i, j) = static_cast<float>(f.curvature);
      // z = a x + b y + c has upward normal (-a, -b, 1) / norm.
      const double nn = std::sqrt(f.a * f.a + f.b * f.b + 1.0);
      out.normal_x(i, j) = static_cast<float>(-f.a / nn);
      out.normal_y(i, j) = static_cast<float>(-f.b / nn);
      out.normal_z(i, j) = static_cast<float>(1.0 / nn);
    }
  }
  };
  const int n = std::max(1, std::min(p.threads, rows));
  std::vector<std::thread> pool;
  for (int t = 0; t < n; ++t) {pool.emplace_back(work, rows * t / n, rows * (t + 1) / n);}
  for (auto & th : pool) {th.join();}
  return out;
}

Grid negativeObstacles(
  const Grid & z, double si, double sj, double sensor_z, const NegativeObstacleParams & p)
{
  const int rows = static_cast<int>(z.rows());
  const int cols = static_cast<int>(z.cols());
  Grid out = Grid::Constant(rows, cols, kNaN);
  const double step = 0.5;                       // cells per ray sample
  const double max_cells = p.max_range / p.resolution;

  struct Gap
  {
    std::vector<std::pair<int, int>> cells;
    int visible{0};
    double start_d{0.0};
    double ref_z{0.0};
  };

  auto close_gap = [&](Gap & g, double end_d) {
      if (g.cells.empty()) {return;}
      const double length = end_d - g.start_d + p.resolution;
      const double h = sensor_z - g.ref_z;
      bool hole = false;
      if (h > 0.05) {
        const double spacing = g.start_d * g.start_d * p.row_spacing / h;
        const double need = std::max(p.min_gap, p.gap_factor * spacing);
        hole = length >= need && 2 * g.visible >= static_cast<int>(g.cells.size());
      }
      for (const auto & [ci, cj] : g.cells) {
        float & o = out(ci, cj);
        if (hole) {
          o = 1.0f;
        } else if (!std::isfinite(o)) {
          o = 0.0f;
        }
      }
      g.cells.clear();
      g.visible = 0;
    };

  for (int b = 0; b < p.azimuth_bins; ++b) {
    const double th = 2.0 * M_PI * b / p.azimuth_bins;
    const double ui = std::cos(th), uj = std::sin(th);
    double horizon = -std::numeric_limits<double>::infinity();
    double ref_z = std::numeric_limits<double>::quiet_NaN();
    Gap gap;
    int last_i = -1, last_j = -1;
    double last_d = 0.0;
    for (double s = step; s <= max_cells; s += step) {
      const int ci = static_cast<int>(std::lround(si + ui * s));
      const int cj = static_cast<int>(std::lround(sj + uj * s));
      if (ci < 0 || cj < 0 || ci >= rows || cj >= cols) {break;}
      if (ci == last_i && cj == last_j) {continue;}
      last_i = ci; last_j = cj;
      const double d = std::max(s * p.resolution, p.resolution);
      last_d = d;
      const float zv = z(ci, cj);
      if (std::isfinite(zv)) {
        close_gap(gap, d - p.resolution);
        horizon = std::max(horizon, (zv - sensor_z) / d);
        ref_z = zv;
        if (!std::isfinite(out(ci, cj))) {out(ci, cj) = 0.0f;}
        continue;
      }
      if (!std::isfinite(ref_z)) {continue;}    // nothing observed yet along this ray
      if (gap.cells.empty()) {
        gap.start_d = d;
        gap.ref_z = ref_z;
      }
      gap.cells.emplace_back(ci, cj);
      // Would the sensor have seen this cell had the ground continued at ref_z height? Only if
      // the cell is in reach (range and lowest beam) and clears the terrain horizon.
      const double tan_flat = (gap.ref_z - sensor_z) / d;
      const bool in_reach = d >= p.min_range && tan_flat >= std::tan(p.min_beam_angle);
      if (in_reach && tan_flat > horizon + p.horizon_margin) {++gap.visible;}
    }
    // A run that reaches the range or grid edge has no far side: it may be a drop-off.
    close_gap(gap, last_d);
  }
  return out;
}

int fillFootprint(
  Grid & elevation, Grid & variance, const std::vector<double> & row_x,
  const std::vector<double> & col_y, const FootprintPlane & fp, float fill_variance)
{
  if (fp.nz <= 1e-6) {return 0;}
  const double c = std::cos(fp.yaw), s = std::sin(fp.yaw);
  int filled = 0;
  for (int i = 0; i < elevation.rows(); ++i) {
    for (int j = 0; j < elevation.cols(); ++j) {
      if (std::isfinite(elevation(i, j))) {continue;}
      const double dx = row_x[i] - fp.x, dy = col_y[j] - fp.y;
      const double along = c * dx + s * dy, across = -s * dx + c * dy;
      if (std::abs(along) > fp.half_length || std::abs(across) > fp.half_width) {continue;}
      // Plane through the base origin with normal n: n . (p - origin) = 0.
      elevation(i, j) = static_cast<float>(fp.z - (fp.nx * dx + fp.ny * dy) / fp.nz);
      variance(i, j) = fill_variance;
      ++filled;
    }
  }
  return filled;
}

int fillSmallHoles(Grid & elevation, Grid & variance, int max_extent_cells, float extra_variance)
{
  const int R = static_cast<int>(elevation.rows()), C = static_cast<int>(elevation.cols());
  if (max_extent_cells < 1 || R < 3 || C < 3) {return 0;}
  Eigen::MatrixXi label = Eigen::MatrixXi::Zero(R, C);
  const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
  int filled = 0, next = 0;
  std::vector<std::pair<int, int>> patch, stack;
  for (int i0 = 0; i0 < R; ++i0) {
    for (int j0 = 0; j0 < C; ++j0) {
      if (std::isfinite(elevation(i0, j0)) || label(i0, j0)) {continue;}
      // Flood one 4-connected unknown patch.
      ++next;
      patch.clear();
      stack.assign(1, {i0, j0});
      label(i0, j0) = next;
      bool edge = false;
      int imin = i0, imax = i0, jmin = j0, jmax = j0;
      while (!stack.empty()) {
        const auto [i, j] = stack.back();
        stack.pop_back();
        patch.push_back({i, j});
        edge = edge || i == 0 || j == 0 || i == R - 1 || j == C - 1;
        imin = std::min(imin, i); imax = std::max(imax, i);
        jmin = std::min(jmin, j); jmax = std::max(jmax, j);
        for (int k = 0; k < 4; ++k) {
          const int a = i + di[k], b = j + dj[k];
          if (a < 0 || b < 0 || a >= R || b >= C) {continue;}
          if (std::isfinite(elevation(a, b)) || label(a, b)) {continue;}
          label(a, b) = next;
          stack.push_back({a, b});
        }
      }
      if (edge || imax - imin + 1 > max_extent_cells || jmax - jmin + 1 > max_extent_cells) {
        continue;
      }
      // Border statistics seed the solve: start from the border mean, take the worst variance.
      double sum = 0.0;
      int n = 0;
      float var_border = 0.0f;
      for (const auto & [i, j] : patch) {
        for (int k = 0; k < 4; ++k) {
          const int a = i + di[k], b = j + dj[k];
          if (std::isfinite(elevation(a, b))) {
            sum += elevation(a, b);
            ++n;
            if (std::isfinite(variance(a, b))) {var_border = std::max(var_border, variance(a, b));}
          }
        }
      }
      if (n == 0) {continue;}
      std::vector<double> v(patch.size(), sum / n);
      auto index = [&](int a, int b) -> int {
        for (size_t q = 0; q < patch.size(); ++q) {
          if (patch[q].first == a && patch[q].second == b) {return static_cast<int>(q);}
        }
        return -1;
      };
      // Neighbour table: a patch index, or -1 with the known border height.
      struct Nb {int q; double z;};
      std::vector<std::array<Nb, 4>> nb(patch.size());
      for (size_t q = 0; q < patch.size(); ++q) {
        for (int k = 0; k < 4; ++k) {
          const int a = patch[q].first + di[k], b = patch[q].second + dj[k];
          nb[q][k] = std::isfinite(elevation(a, b)) ? Nb{-1, elevation(a, b)} : Nb{index(a, b), 0.0};
        }
      }
      // Gauss-Seidel on the discrete Laplace equation. Patches are at most max_extent squared
      // cells, so a few hundred sweeps converge far below a millimetre.
      for (int it = 0; it < 400; ++it) {
        double change = 0.0;
        for (size_t q = 0; q < patch.size(); ++q) {
          double acc = 0.0;
          for (int k = 0; k < 4; ++k) {acc += nb[q][k].q < 0 ? nb[q][k].z : v[nb[q][k].q];}
          const double nv = acc / 4.0;
          change = std::max(change, std::abs(nv - v[q]));
          v[q] = nv;
        }
        if (change < 1e-5) {break;}
      }
      for (size_t q = 0; q < patch.size(); ++q) {
        elevation(patch[q].first, patch[q].second) = static_cast<float>(v[q]);
        variance(patch[q].first, patch[q].second) = var_border + extra_variance;
      }
      filled += static_cast<int>(patch.size());
    }
  }
  return filled;
}

}  // namespace rover_terrain_mapping
