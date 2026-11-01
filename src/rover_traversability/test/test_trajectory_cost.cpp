// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>

#include "rover_traversability/core/trajectory_cost.hpp"

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

// 8 m square, ground rising in +y at the given angle.
TerrainGrid slopeY(double deg)
{
  TerrainGrid g;
  g.resolution = 0.05;
  g.elevation.resize(161, 161);
  for (int i = 0; i < 161; ++i) {
    for (int j = 0; j < 161; ++j) {
      g.elevation(i, j) = static_cast<float>(std::tan(deg * M_PI / 180.0) * g.y(j));
    }
  }
  return g;
}

EvalParams critic()
{
  EvalParams e;
  e.risk_kappa = 0.0;
  e.check_clearance = false;
  e.check_roughness = false;
  return e;
}

// One rollout of n points along a straight line at the given heading and body velocity.
void line(Rollouts & x, Rollouts & y, Rollouts & yaw, Rollouts & vx, Rollouts & vy, int b,
  double heading, double vbx, double vby)
{
  const double dir = heading + std::atan2(vby, vbx);
  for (int t = 0; t < x.cols(); ++t) {
    const double s = 0.05 * t;
    x(b, t) = static_cast<float>(s * std::cos(dir));
    y(b, t) = static_cast<float>(s * std::sin(dir));
    yaw(b, t) = static_cast<float>(heading);
    vx(b, t) = static_cast<float>(vbx);
    vy(b, t) = static_cast<float>(vby);
  }
}

}  // namespace

TEST(TrajectoryCost, AcrossASteepSidehillIsACollisionFacingUpIsNot)
{
  // 20 deg slope: driving along x rolls 20 deg (lethal), crabbing along x while facing up the
  // slope pitches 20 deg (u 0.387). Both rollouts cover the same ground.
  const auto g = slopeY(20.0);
  Rollouts x(2, 10), y(2, 10), yaw(2, 10), vx(2, 10), vy(2, 10);
  line(x, y, yaw, vx, vy, 0, 0.0, 0.2, 0.0);           // heading +x, driving forward
  line(x, y, yaw, vx, vy, 1, M_PI / 2, 0.0, -0.2);     // heading +y, crabbing to its right = +x
  TrajectoryCostParams c;
  c.point_step = 1;
  const auto cost = trajectoryCosts(g, rover(), critic(), c, x, y, yaw, vx, vy, 0.05);
  EXPECT_GE(cost(0), 1.0e6);
  EXPECT_LT(cost(1), 1.0e3);
  // Crabbing across the slope is level travel: no uphill term. 10 points of 20 * 0.387441^2.
  EXPECT_NEAR(cost(1), 10 * 20.0 * 0.387441 * 0.387441, 1e-2);
}

TEST(TrajectoryCost, RedirectCostsTheTurnedAngle)
{
  // Flat ground, heading fixed, travel direction flips 90 deg once halfway: one redirect of
  // pi / 2, cost w_redirect * pi / 2.
  const auto g = slopeY(0.0);
  Rollouts x(1, 10), y(1, 10), yaw(1, 10), vx(1, 10), vy(1, 10);
  line(x, y, yaw, vx, vy, 0, 0.0, 0.2, 0.0);
  for (int t = 5; t < 10; ++t) {vx(0, t) = 0.0f; vy(0, t) = 0.2f;}
  TrajectoryCostParams c;
  c.point_step = 1;
  const auto cost = trajectoryCosts(g, rover(), critic(), c, x, y, yaw, vx, vy, 0.05);
  EXPECT_NEAR(cost(0), 2.0 * M_PI / 2, 1e-4);
}

TEST(TrajectoryCost, UphillCostsTimeDownhillDoesNot)
{
  // 10 deg slope rising in +y. Heading +y forward: slope along travel tan(10 deg) = 0.176327,
  // per point w_uphill * 0.176327 * dt. Heading -y forward: downhill, no time term. Terrain
  // utilisation is the same both ways (u 0 at 10 deg facing the slope).
  const auto g = slopeY(10.0);
  Rollouts x(2, 10), y(2, 10), yaw(2, 10), vx(2, 10), vy(2, 10);
  line(x, y, yaw, vx, vy, 0, M_PI / 2, 0.2, 0.0);
  line(x, y, yaw, vx, vy, 1, -M_PI / 2, 0.2, 0.0);
  TrajectoryCostParams c;
  c.point_step = 1;
  const auto cost = trajectoryCosts(g, rover(), critic(), c, x, y, yaw, vx, vy, 0.05);
  EXPECT_NEAR(cost(0), 10 * 5.0 * std::tan(10 * M_PI / 180) * 0.05, 1e-4);
  EXPECT_NEAR(cost(1), 0.0, 1e-4);
}

TEST(TrajectoryCost, UnknownPointsCostUnknownCost)
{
  auto g = slopeY(0.0);
  g.elevation.setConstant(NAN);
  Rollouts x(1, 4), y(1, 4), yaw(1, 4), vx(1, 4), vy(1, 4);
  line(x, y, yaw, vx, vy, 0, 0.0, 0.2, 0.0);
  TrajectoryCostParams c;
  c.point_step = 1;
  EXPECT_NEAR(trajectoryCosts(g, rover(), critic(), c, x, y, yaw, vx, vy, 0.05)(0), 4.0, 1e-6);
}
