// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>

#include "rover_traversability/core/speed_safety.hpp"

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
}  // namespace

TEST(SpeedSafety, PredictStraightAndArc)
{
  auto s = predictPoses({0, 0, 0}, 0.5, 0.0, 0.0, 2.0, 0.5);
  ASSERT_EQ(s.size(), 4u);
  EXPECT_NEAR(s.back().x, 1.0, 1e-9);
  EXPECT_NEAR(s.back().y, 0.0, 1e-9);
  // Quarter circle: radius v / w = 1 m, w pi/2 rad/s over 1 s ends at (1, 1) heading pi/2.
  auto a = predictPoses({0, 0, 0}, M_PI / 2, 0.0, M_PI / 2, 1.0, 0.05);
  EXPECT_NEAR(a.back().x, 1.0, 1e-3);
  EXPECT_NEAR(a.back().y, 1.0, 1e-3);
  EXPECT_NEAR(a.back().yaw, M_PI / 2, 1e-9);
  // Crab: vy only.
  auto c = predictPoses({0, 0, M_PI / 2}, 0.0, 0.2, 0.0, 1.0, 0.5);
  EXPECT_NEAR(c.back().x, -0.2, 1e-9);
}

TEST(SpeedSafety, SpeedLimitFormula)
{
  GovernorParams g;          // 0.25 m/s nominal, 0.05 floor, alpha 0.8
  EXPECT_NEAR(speedLimit(0.0, g), 0.25, 1e-12);
  EXPECT_NEAR(speedLimit(0.5, g), 0.25 * 0.6, 1e-12);      // 0.15
  EXPECT_NEAR(speedLimit(1.0, g), 0.05, 1e-12);            // 0.25 * 0.2 = 0.05, the floor
  EXPECT_NEAR(speedLimit(INFINITY, g), 0.05, 1e-12);
}

TEST(SpeedSafety, TipLimitsMatchTheBenchmark)
{
  double rl, pl;
  tipLimits(rover(), rl, pl);
  EXPECT_NEAR(rl * 180 / M_PI, 16.11, 0.01);
  EXPECT_NEAR(pl * 180 / M_PI, 29.87, 0.01);
  // Within 3 deg of the 16.11 deg roll limit: risk at 13.2 deg, none at 13.0.
  EXPECT_TRUE(tipRisk(13.2 * M_PI / 180, 0.0, rover(), 3 * M_PI / 180));
  EXPECT_FALSE(tipRisk(13.0 * M_PI / 180, 0.0, rover(), 3 * M_PI / 180));
  EXPECT_TRUE(tipRisk(0.0, -27.0 * M_PI / 180, rover(), 3 * M_PI / 180));
}

TEST(SpeedSafety, WorstUtilisationAhead)
{
  // Flat for x < 1 m, 20 deg rising in +y beyond. Driving +x from the origin for 4 s at
  // 0.5 m/s puts all four wheels on the sidehill (centre past x 1.37): across it the roll is
  // lethal.
  TerrainGrid g;
  g.resolution = 0.05;
  g.elevation.resize(121, 121);
  for (int i = 0; i < 121; ++i) {
    for (int j = 0; j < 121; ++j) {
      g.elevation(i, j) = g.x(i) > 1.0 ? static_cast<float>(std::tan(20 * M_PI / 180) * g.y(j)) : 0.0f;
    }
  }
  EvalParams e;
  e.risk_kappa = 0.0;
  const auto ahead = predictPoses({0, 0, 0}, 0.5, 0.0, 0.0, 4.0, 0.25);
  EXPECT_TRUE(std::isinf(worstUtilisation(g, rover(), e, ahead, 0.5)));
  const auto short_ahead = predictPoses({0, 0, 0}, 0.5, 0.0, 0.0, 0.5, 0.25);   // to x 0.25
  EXPECT_NEAR(worstUtilisation(g, rover(), e, short_ahead, 0.5), 0.0, 1e-9);
}
