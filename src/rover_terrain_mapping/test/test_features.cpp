// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Terrain feature math against hand-computed cases.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rover_terrain_mapping/features.hpp"

using rover_terrain_mapping::FeatureParams;
using rover_terrain_mapping::Grid;
using rover_terrain_mapping::NegativeObstacleParams;

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kRes = 0.05;

FeatureParams params()
{
  FeatureParams p;
  p.resolution = kRes;
  p.plane_radius = 0.35;   // 7 cells
  p.step_radius = 0.15;    // 3 cells
  return p;
}

Grid flat(int n, float z = 0.0f) {return Grid::Constant(n, n, z);}
Grid var(int n, float v = 1e-4f) {return Grid::Constant(n, n, v);}

double deg(double r) {return r * 180.0 / M_PI;}

}  // namespace

TEST(Features, TiltedPlaneSlopeAndNormal)
{
  // z = tan(10 deg) * x. Rows run along -x, so x = -i * res about any centre.
  const int n = 41;
  Grid z(n, n);
  const double t = std::tan(10.0 * M_PI / 180.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {z(i, j) = static_cast<float>(t * (-i * kRes));}
  }
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_NEAR(deg(f.slope(20, 20)), 10.0, 1e-3);
  EXPECT_NEAR(f.normal_x(20, 20), -std::sin(10.0 * M_PI / 180.0), 1e-5);
  EXPECT_NEAR(f.normal_y(20, 20), 0.0, 1e-5);
  EXPECT_NEAR(f.normal_z(20, 20), std::cos(10.0 * M_PI / 180.0), 1e-5);
  EXPECT_NEAR(f.roughness(20, 20), 0.0, 1e-5);
  EXPECT_NEAR(f.curvature(20, 20), 0.0, 1e-5);
}

TEST(Features, StepIsMaxMinusMinInTheWheelWindow)
{
  // A 0.10 m block of 2 x 2 cells at rows 20..21, cols 20..21.
  const int n = 41;
  Grid z = flat(n);
  z.block(20, 20, 2, 2).setConstant(0.10f);
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_NEAR(f.step(20, 20), 0.10, 1e-6);
  EXPECT_NEAR(f.step(17, 20), 0.10, 1e-6);   // 3 cells away, on the window edge
  EXPECT_NEAR(f.step(16, 20), 0.0, 1e-6);    // 4 cells away, outside
}

TEST(Features, CheckerboardRoughness)
{
  // +-h alternating: the plane fit is flat and every residual is h, so roughness = h.
  // An odd count of cells in the window leaves the mean off zero by h / count, so the
  // tolerance is h / 10.
  const int n = 41;
  const float h = 0.02f;
  Grid z(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {z(i, j) = ((i + j) % 2 == 0) ? h : -h;}
  }
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_NEAR(f.roughness(20, 20), h, h / 10.0);
  EXPECT_LT(deg(f.slope(20, 20)), 1.0);
}

TEST(Features, SingleScanLineGivesNoPlane)
{
  // Only row 20 known: cells on a line. The spread test must refuse the plane instead of
  // returning an arbitrary tilt. Step still works.
  const int n = 41;
  Grid z = Grid::Constant(n, n, kNaN);
  for (int j = 0; j < n; ++j) {z(20, j) = 0.01f * (j % 3);}
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_TRUE(std::isnan(f.slope(20, 20)));
  EXPECT_TRUE(std::isnan(f.normal_z(20, 20)));
  EXPECT_NEAR(f.step(20, 20), 0.02, 1e-6);
}

TEST(Features, TooEmptyWindowGivesNoPlane)
{
  // 4 known cells in a window of 149: under the 30 percent fill.
  const int n = 41;
  Grid z = Grid::Constant(n, n, kNaN);
  z(20, 20) = z(20, 22) = z(22, 20) = z(22, 22) = 0.0f;
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_TRUE(std::isnan(f.slope(20, 20)));
}

TEST(Features, UnknownCentreStaysUnknown)
{
  const int n = 41;
  Grid z = flat(n);
  z(20, 20) = kNaN;
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_TRUE(std::isnan(f.slope(20, 20)));
  EXPECT_TRUE(std::isnan(f.step(20, 20)));
  EXPECT_FALSE(std::isnan(f.slope(20, 21)));
}

TEST(Features, SlopeStdScalesWithElevationStd)
{
  // The (a, b) covariance is sigma^2 (A^T A)^-1, so doubling sigma doubles slope_std.
  // On a flat window slope_std = sqrt((var_a + var_b) / 2) = sigma / sqrt(sum x^2) by symmetry.
  const int n = 41;
  const auto p = params();
  const auto f1 = rover_terrain_mapping::computeFeatures(flat(n), var(n, 1e-4f), p);
  const auto f2 = rover_terrain_mapping::computeFeatures(flat(n), var(n, 4e-4f), p);
  EXPECT_NEAR(f2.slope_std(20, 20) / f1.slope_std(20, 20), 2.0, 1e-4);
  double sxx = 0.0;
  for (int di = -7; di <= 7; ++di) {
    for (int dj = -7; dj <= 7; ++dj) {
      if (di * di + dj * dj <= 49) {sxx += (di * kRes) * (di * kRes);}
    }
  }
  EXPECT_NEAR(f1.slope_std(20, 20), 0.01 / std::sqrt(sxx), 1e-6);
}

TEST(Features, FitPlaneMatchesComputeFeatures)
{
  const int n = 41;
  Grid z(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {z(i, j) = 0.1f * (-i * 0.05f) + 0.05f * (-j * 0.05f);}
  }
  const auto pf = rover_terrain_mapping::fitPlane(z, var(n), 20, 20, 7.0, params());
  ASSERT_TRUE(pf.valid);
  EXPECT_NEAR(pf.a, 0.1, 1e-6);
  EXPECT_NEAR(pf.b, 0.05, 1e-6);
  EXPECT_NEAR(pf.slope, std::atan(std::hypot(0.1, 0.05)), 1e-6);
}

// --- Negative obstacles. Flat ground at 0, sensor 0.75 m up at the grid centre. Rays run along
// rows (+i) for the cases below. 0.05 m cells.

namespace
{

constexpr int kN = 241;          // 12 m square
constexpr int kC = 120;          // sensor cell

NegativeObstacleParams negParams()
{
  NegativeObstacleParams p;
  p.resolution = kRes;
  p.max_range = 7.0;
  return p;
}

Grid ground()
{
  Grid z = Grid::Zero(kN, kN);
  // Blind zone under the rover, as a roof lidar sees it.
  for (int i = kC - 10; i <= kC + 10; ++i) {
    for (int j = kC - 10; j <= kC + 10; ++j) {z(i, j) = kNaN;}
  }
  return z;
}

}  // namespace

TEST(NegativeObstacles, HoleAheadIsFlagged)
{
  // Unknown 2.0 to 2.6 m out. Row spacing there: 2^2 * 0.035 / 0.75 = 0.187 m, so a gap must
  // exceed max(0.30, 2 * 0.187) = 0.373 m. 0.6 m is a hole.
  Grid z = ground();
  z.block(kC + 40, kC - 6, 12, 13).setConstant(kNaN);
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 45, kC), 1.0f);
  EXPECT_EQ(neg(kC + 45, kC + 5), 1.0f);
  EXPECT_EQ(neg(kC + 30, kC), 0.0f);           // observed ground before it
}

TEST(NegativeObstacles, ShadowBehindRockIsNotFlagged)
{
  // A 1 m tall rock 2.0 m out hides the ground behind it. Its top sets the horizon at
  // (1.0 - 0.75) / 2.0 = 0.125, above any flat continuation, so the shadow is plain unknown.
  Grid z = ground();
  z.block(kC + 40, kC - 6, 2, 13).setConstant(1.0f);
  z.block(kC + 42, kC - 6, 12, 13).setConstant(kNaN);
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 46, kC), 0.0f);
}

TEST(NegativeObstacles, SmallGapIsNotFlagged)
{
  Grid z = ground();
  z.block(kC + 40, kC - 6, 3, 13).setConstant(kNaN);   // 0.15 m, under min_gap
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 41, kC), 0.0f);
}

TEST(NegativeObstacles, FarSamplingGapIsNotFlagged)
{
  // 0.8 m unknown at 6 m: rows land 6^2 * 0.035 / 0.75 = 1.68 m apart there, so a gap under
  // 3.36 m is expected sampling, not a hole.
  Grid z = ground();
  z.block(kC + 120, kC - 6, 16, 13).setConstant(kNaN);
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 125, kC), 0.0f);
}

TEST(NegativeObstacles, NothingInferredBeyondRange)
{
  Grid z = ground();
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_TRUE(std::isnan(neg(kC + 119, kC + 119)));   // 8.4 m diagonal, beyond 7 m
  EXPECT_TRUE(std::isnan(neg(kC, kC)));               // blind zone, never a reference
}
