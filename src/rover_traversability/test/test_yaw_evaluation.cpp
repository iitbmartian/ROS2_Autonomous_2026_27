// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>

#include "rover_traversability/core/yaw_evaluation.hpp"

using namespace rover_traversability::core;   // NOLINT

namespace
{

PhysicalParams rover()
{
  PhysicalParams p;
  p.wheelbase = 0.744729; p.track = 0.436; p.wheel_radius = 0.14985; p.wheel_width = 0.14;
  p.footprint_length = 1.1; p.footprint_width = 0.9; p.cg_height = 0.45;
  p.ground_clearance = 0.25; p.rocker_max = 0.35; p.step_max = 0.10;
  p.stability_margin_min = 0.17; p.roughness_max = 0.05;
  return p;
}

TerrainGrid slopeY(double deg)
{
  TerrainGrid g;
  g.resolution = 0.05;
  g.elevation.resize(41, 41);
  for (int i = 0; i < 41; ++i) {
    for (int j = 0; j < 41; ++j) {
      g.elevation(i, j) = static_cast<float>(std::tan(deg * M_PI / 180.0) * g.y(j));
    }
  }
  return g;
}

EvalParams exact()
{
  EvalParams e;
  e.risk_kappa = 0.0;
  return e;
}

}  // namespace

TEST(YawEvaluation, FlatGroundEveryHeadingSafe)
{
  YawParams y;
  y.yaw_bins = 16;
  const auto out = evaluateYaws(slopeY(0.0), rover(), exact(), y);
  EXPECT_EQ(out.safe_yaw_mask(20, 20), static_cast<float>(0xFFFF));
  EXPECT_NEAR(out.best_utilisation(20, 20), 0.0, 1e-9);
}

TEST(YawEvaluation, TwentyDegreeSidehillOnlyUpAndDownSafe)
{
  // 20 deg rising in +y. Across the slope (yaw 0, pi) roll is 20 deg, past the 16.1 deg limit:
  // u_tip = 0.349066 / 0.281126 = 1.24, lethal. Facing up or down (yaw pi/2, 3 pi/2) it is
  // pitch: rear margin 0.691272 - 0.349066 = 0.342206, u = 0.387441, safe. 16 bins: bit 4 is
  // pi/2 and bit 12 is 3 pi/2.
  YawParams y;
  y.yaw_bins = 16;
  const auto out = evaluateYaws(slopeY(20.0), rover(), exact(), y);
  const auto mask = static_cast<unsigned>(out.safe_yaw_mask(20, 20));
  EXPECT_FALSE(mask & (1u << 0));
  EXPECT_FALSE(mask & (1u << 8));
  EXPECT_TRUE(mask & (1u << 4));
  EXPECT_TRUE(mask & (1u << 12));
  EXPECT_NEAR(out.best_utilisation(20, 20), 0.387441, 1e-4);
  EXPECT_NEAR(std::fabs(std::sin(out.best_yaw(20, 20))), 1.0, 1e-9);
}

TEST(YawEvaluation, SymmetryShortcutMatchesFullEvaluation)
{
  // A CG offset forward disables the shortcut. With offset 0 and the shortcut on, the mask must
  // equal a full 16-heading evaluation, checked here by evaluating each heading directly.
  const auto g = slopeY(12.0);
  YawParams y;
  y.yaw_bins = 16;
  const auto out = evaluateYaws(g, rover(), exact(), y);
  unsigned full = 0;
  for (int k = 0; k < 16; ++k) {
    const auto r = evaluatePose(g, g.x(20), g.y(20), 2 * M_PI * k / 16, rover(), exact());
    if (r.status == PoseStatus::kOk) {full |= 1u << k;}
  }
  EXPECT_EQ(static_cast<unsigned>(out.safe_yaw_mask(20, 20)), full);
}

TEST(YawEvaluation, UnknownCellsStayUnknown)
{
  auto g = slopeY(0.0);
  g.elevation(20, 20) = NAN;
  const auto out = evaluateYaws(g, rover(), exact(), YawParams());
  EXPECT_TRUE(std::isnan(out.best_utilisation(20, 20)));
  // Edge cells have off-grid contacts at every heading: unknown too.
  EXPECT_TRUE(std::isnan(out.best_utilisation(0, 0)));
}

TEST(YawEvaluation, DiscMax)
{
  Eigen::MatrixXf m = Eigen::MatrixXf::Zero(9, 9);
  m(4, 4) = 1.0f;
  m(0, 0) = NAN;
  const auto d = discMax(m, 2.0);
  EXPECT_EQ(d(4, 6), 1.0f);     // 2 cells away
  EXPECT_EQ(d(5, 6), 0.0f);     // sqrt(5) > 2
  EXPECT_EQ(d(0, 0), 0.0f);     // NaN centre, known neighbours
}
