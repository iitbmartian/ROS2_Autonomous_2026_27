// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// CPU cost of one feature map: terrain features, negative obstacles and the per-cell heading
// evaluation on a synthetic, fully observed 10 x 10 m map at 5 cm (rolling hills, rocks, noise).
// Reports process CPU time per stage, which stays meaningful on a loaded machine where wall time
// does not. Usage: bench_features [repeats] [yaw_bins] [threads]

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>

#include "rover_terrain_mapping/features.hpp"
#include "rover_traversability/core/pose_evaluation.hpp"
#include "rover_traversability/core/yaw_evaluation.hpp"

namespace core = rover_traversability::core;
using rover_terrain_mapping::Grid;

static double cpuMs()
{
  return 1000.0 * static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
}

int main(int argc, char ** argv)
{
  const int repeats = argc > 1 ? std::atoi(argv[1]) : 3;
  const int bins = argc > 2 ? std::atoi(argv[2]) : 16;
  const int threads = argc > 3 ? std::atoi(argv[3]) : 1;
  const int n = 200;
  const double res = 0.05;
  std::mt19937 rng(7);
  std::normal_distribution<float> noise(0.0f, 0.01f);
  Grid z(n, n), var = Grid::Constant(n, n, 1e-4f);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double x = -i * res, y = -j * res;
      z(i, j) = static_cast<float>(0.3 * std::sin(x / 1.5) * std::cos(y / 2.0)) + noise(rng);
    }
  }
  for (int k = 0; k < 60; ++k) {          // rocks, 10 to 25 cm
    const int ci = rng() % n, cj = rng() % n, r = 2 + rng() % 3;
    const float h = 0.1f + 0.15f * (rng() % 100) / 100.0f;
    for (int i = std::max(0, ci - r); i < std::min(n, ci + r); ++i) {
      for (int j = std::max(0, cj - r); j < std::min(n, cj + r); ++j) {z(i, j) += h;}
    }
  }

  core::PhysicalParams p;
  p.wheelbase = 0.744729; p.track = 0.436; p.wheel_radius = 0.14985; p.wheel_width = 0.14;
  p.footprint_length = 1.1; p.footprint_width = 0.9; p.cg_height = 0.45;
  p.ground_clearance = 0.25; p.rocker_max = 0.35; p.step_max = 0.10;
  p.stability_margin_min = 0.17; p.roughness_max = 0.05;

  rover_terrain_mapping::FeatureParams fp;
  fp.resolution = res;
  fp.plane_radius = p.wheelbase / 2;
  fp.step_radius = p.wheel_radius;
  fp.threads = threads;
  rover_terrain_mapping::NegativeObstacleParams np;
  np.resolution = res;
  core::YawParams yp;
  yp.yaw_bins = bins;
  yp.threads = threads;

  double t_feat = 0, t_neg = 0, t_disc = 0, t_yaw = 0, wall_yaw = 0;
  for (int r = 0; r < repeats; ++r) {
    double t0 = cpuMs();
    const auto f = rover_terrain_mapping::computeFeatures(z, var, fp);
    double t1 = cpuMs();
    const Grid neg = rover_terrain_mapping::negativeObstacles(z, n / 2.0, n / 2.0, 0.75, np);
    double t2 = cpuMs();
    core::TerrainGrid g;
    g.resolution = res;
    g.elevation = z; g.variance = var; g.step = f.step; g.negative_obstacle = neg;
    g.roughness = core::discMax(f.roughness, std::hypot(p.wheelbase / 2, p.track / 2) / res);
    double t3 = cpuMs();
    core::EvalParams e;
    e.rough_radius = 0.0;
    const auto w0 = std::chrono::steady_clock::now();
    const auto y = core::evaluateYaws(g, p, e, yp);
    wall_yaw += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
    double t4 = cpuMs();
    t_feat += t1 - t0; t_neg += t2 - t1; t_disc += t3 - t2; t_yaw += t4 - t3;
    if (r == 0) {
      int known = 0;
      for (int i = 0; i < n; ++i) {for (int j = 0; j < n; ++j) {known += std::isfinite(y.best_utilisation(i, j));}}
      std::printf("cells with a heading verdict: %d of %d\n", known, n * n);
    }
  }
  std::printf("CPU ms per map (mean of %d), yaw_bins %d, threads %d:\n", repeats, bins, threads);
  std::printf("  features            %8.1f\n", t_feat / repeats);
  std::printf("  negative obstacles  %8.1f\n", t_neg / repeats);
  std::printf("  roughness disc max  %8.1f\n", t_disc / repeats);
  std::printf("  heading evaluation  %8.1f   (wall %.1f)\n", t_yaw / repeats, wall_yaw / repeats);
  std::printf("  total               %8.1f\n", (t_feat + t_neg + t_disc + t_yaw) / repeats);
  return 0;
}
