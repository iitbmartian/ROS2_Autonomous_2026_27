// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Terrain feature math against hand-computed cases.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

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

TEST(Features, StepIsTrimmedRangeInTheWheelWindow)
{
  // A 0.10 m block of 2 x 2 cells at rows 20..21, cols 20..21.
  const int n = 41;
  Grid z = flat(n);
  z.block(20, 20, 2, 2).setConstant(0.10f);
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  // Step is taken relative to the chassis plane, which the block tilts slightly, so the values
  // move by under a millimetre at the block and under a centimetre on the flat cells beyond.
  // The top and bottom cell are trimmed, so the block counts once two of its cells are in the
  // 3-cell disc. From row 18, (20, 20) and (20, 21) are at 2 and sqrt(5) cells, both inside.
  // From row 17 only (20, 20) is, at 3 cells: one cell is trimmed as noise.
  EXPECT_NEAR(f.step(20, 20), 0.10, 1e-3);
  EXPECT_NEAR(f.step(18, 20), 0.10, 1e-3);
  EXPECT_LT(f.step(17, 20), 0.01);
  EXPECT_LT(f.step(16, 20), 0.01);           // 4 cells away, outside
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
  // Roof lidar: its lowest beam (-15 deg) meets flat ground from 2.80 m. Unknown from 3.0 to
  // 4.0 m. Row spacing at 3 m: 3^2 * 0.035 / 0.75 = 0.42 m, so a gap must exceed
  // max(0.30, 2 * 0.42) = 0.84 m. 1.0 m is a hole.
  Grid z = ground();
  z.block(kC + 60, kC - 6, 20, 13).setConstant(kNaN);
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 70, kC), 1.0f);
  EXPECT_EQ(neg(kC + 70, kC + 5), 1.0f);
  EXPECT_EQ(neg(kC + 58, kC), 0.0f);           // observed ground before it
}

TEST(NegativeObstacles, RoofLidarMissesANearHole)
{
  // The same lidar cannot see a 0.6 m hole 2.0 to 2.6 m out: it is inside its blind radius.
  // This is a limit of the sensor, recorded here so nobody expects otherwise.
  Grid z = ground();
  z.block(kC + 40, kC - 6, 12, 13).setConstant(kNaN);
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 45, kC), 0.0f);
}

TEST(NegativeObstacles, InvertedDomeSeesANearHole)
{
  // Beams reaching straight down: the 0.6 m hole at 2.0 m (needs 0.373 m there) is flagged.
  Grid z = ground();
  z.block(kC + 40, kC - 6, 12, 13).setConstant(kNaN);
  auto p = negParams();
  p.min_beam_angle = -M_PI / 2;
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, p);
  EXPECT_EQ(neg(kC + 45, kC), 1.0f);
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

TEST(NegativeObstacles, BlindZoneUnderTheSensorIsNotFlagged)
{
  // Observed ring 0.5 m out (say, from an earlier pose), then unknown to 2.5 m. The lowest beam
  // (-15 deg) reaches flat ground only from 0.75 / tan(15 deg) = 2.80 m, so none of this was
  // ever visible: blind zone, not a hole.
  Grid z = ground();
  z.block(kC + 11, kC - 6, 39, 13).setConstant(kNaN);
  z.block(kC + 10, kC - 6, 1, 13).setConstant(0.0f);
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_EQ(neg(kC + 30, kC), 0.0f);
  EXPECT_EQ(neg(kC + 45, kC), 0.0f);
}

TEST(NegativeObstacles, InvertedDomeSeesUnderneath)
{
  // With the beams reaching straight down (inverted L2), the same unknown run 0.55 to 2.5 m out
  // was visible past min_range 1.0 m, so it is a hole.
  Grid z = ground();
  z.block(kC + 11, kC - 6, 39, 13).setConstant(kNaN);
  z.block(kC + 10, kC - 6, 1, 13).setConstant(0.0f);
  auto p = negParams();
  p.min_beam_angle = -M_PI / 2;
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, p);
  EXPECT_EQ(neg(kC + 30, kC), 1.0f);
}

TEST(NegativeObstacles, NothingInferredBeyondRange)
{
  Grid z = ground();
  const Grid neg = rover_terrain_mapping::negativeObstacles(z, kC, kC, 0.75, negParams());
  EXPECT_TRUE(std::isnan(neg(kC + 119, kC + 119)));   // 8.4 m diagonal, beyond 7 m
  EXPECT_TRUE(std::isnan(neg(kC, kC)));               // blind zone, never a reference
}

TEST(FillFootprint, PlaneUnderRotatedTiltedRover)
{
  // 5 x 5 grid at 0.1 m, cell centres x 0.2..-0.2 down the rows and y 0.2..-0.2 across. Rover
  // at the origin, z 0.5, heading 90 deg so its 0.3 m length runs along y and its 0.2 m width
  // along x. Normal (sin 10 deg, 0, cos 10 deg): z = 0.5 - tan(10 deg) dx. Cells with
  // |dx| <= 0.1 and |dy| <= 0.15 are inside, 3 x 3 = 9, one already observed, 8 filled.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Grid z = Grid::Constant(5, 5, static_cast<float>(nan));
  Grid var = Grid::Constant(5, 5, static_cast<float>(nan));
  z(2, 1) = 0.9f;                 // x 0, y 0.1, observed, must stay
  var(2, 1) = 0.02f;
  const std::vector<double> row_x{0.2, 0.1, 0.0, -0.1, -0.2}, col_y{0.2, 0.1, 0.0, -0.1, -0.2};
  rover_terrain_mapping::FootprintPlane fp;
  fp.z = 0.5;
  fp.yaw = M_PI / 2.0;
  fp.nx = std::sin(10.0 * M_PI / 180.0);
  fp.nz = std::cos(10.0 * M_PI / 180.0);
  fp.half_length = 0.15;
  fp.half_width = 0.1;
  EXPECT_EQ(rover_terrain_mapping::fillFootprint(z, var, row_x, col_y, fp, 1e-4f), 8);
  EXPECT_FLOAT_EQ(z(2, 1), 0.9f);
  EXPECT_FLOAT_EQ(var(2, 1), 0.02f);
  EXPECT_NEAR(z(2, 2), 0.5, 1e-6);
  EXPECT_NEAR(z(1, 3), 0.482367, 1e-6);     // dx 0.1
  EXPECT_NEAR(z(3, 1), 0.517633, 1e-6);     // dx -0.1
  EXPECT_FLOAT_EQ(var(1, 3), 1e-4f);
  EXPECT_TRUE(std::isnan(z(2, 0)));         // dy 0.2, past the half length
  EXPECT_TRUE(std::isnan(z(0, 2)));         // dx 0.2, past the half width
}

TEST(Features, SmoothRampIsNotAStep)
{
  // z = tan(15 deg) * x. Raw max minus min over the 3-cell (0.15 m) disc would be
  // 0.30 * tan(15 deg) = 0.0804 m. Relative to the chassis plane it is 0.
  const int n = 41;
  Grid z(n, n);
  const double t = std::tan(15.0 * M_PI / 180.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {z(i, j) = static_cast<float>(t * (-i * kRes));}
  }
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_NEAR(f.step(20, 20), 0.0, 1e-5);
}

TEST(Features, CliffOnARampKeepsItsHeight)
{
  // 15 deg ramp along x with a 0.10 m rise between rows 20 and 21 (all columns). Detrending
  // can take off at most one cell of plane slope: the fitted gradient a is at most
  // tan(15 deg) + 0.85 * 0.10 / 0.35 = 0.511 per m, times 0.05 m, about 0.026 m. Checked at
  // every cell whose wheel disc holds two or more cells on each side of the edge (rows 19 to
  // 22). Rows 18 and 23 reach across through the single cell at offset 3, which is trimmed.
  const int n = 41;
  Grid z(n, n);
  const double t = std::tan(15.0 * M_PI / 180.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      z(i, j) = static_cast<float>(t * (-i * kRes) + (i <= 20 ? 0.10 : 0.0));
    }
  }
  const auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  for (int i = 19; i <= 22; ++i) {
    EXPECT_GE(f.step(i, 20), 0.10 - 0.026) << "row " << i;
    EXPECT_LE(f.step(i, 20), 0.10 + 1e-5) << "row " << i;
  }
  EXPECT_NEAR(f.step(30, 20), 0.0, 1e-5);   // far from the edge, plain ramp
}

TEST(Features, SingleCellSpikeIsTrimmedButTwoCellsCount)
{
  // A lone 0.10 m cell is one stray return. A pair of them is a feature under the wheel.
  const int n = 41;
  Grid z = flat(n);
  z(20, 20) = 0.10f;
  auto f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_LT(f.step(20, 20), 0.01);
  z(20, 21) = 0.10f;
  f = rover_terrain_mapping::computeFeatures(z, var(n), params());
  EXPECT_NEAR(f.step(20, 20), 0.10, 1e-3);
  // A deep single-cell hole is trimmed the same way, a pair is not.
  Grid h = flat(n);
  h(20, 20) = -0.10f;
  f = rover_terrain_mapping::computeFeatures(h, var(n), params());
  EXPECT_LT(f.step(20, 20), 0.01);
  h(21, 20) = -0.10f;
  f = rover_terrain_mapping::computeFeatures(h, var(n), params());
  EXPECT_NEAR(f.step(20, 20), 0.10, 1e-3);
}
