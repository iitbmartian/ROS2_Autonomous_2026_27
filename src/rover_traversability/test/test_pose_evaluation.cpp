// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Pose evaluation against hand-computed cases. Rover: the sim values of rover_physical.yaml,
// wheelbase 0.744729 (hx 0.3723645), track 0.436 (hy 0.218), CG 0.45 m up and centred.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>

#include "rover_traversability/core/pose_evaluation.hpp"

using rover_traversability::core::EvalParams;
using rover_traversability::core::PhysicalParams;
using rover_traversability::core::PoseStatus;
using rover_traversability::core::TerrainGrid;

namespace
{

constexpr double kDeg = M_PI / 180.0;

PhysicalParams rover()
{
  PhysicalParams p;
  p.wheelbase = 0.744729;
  p.track = 0.436;
  p.wheel_radius = 0.14985;
  p.wheel_width = 0.14;
  p.footprint_length = 1.1;
  p.footprint_width = 0.9;
  p.cg_height = 0.45;
  p.ground_clearance = 0.25;
  p.rocker_max = 0.35;
  p.step_max = 0.10;
  p.stability_margin_min = 0.17;
  p.roughness_max = 0.05;
  return p;
}

EvalParams exact()
{
  EvalParams e;
  e.risk_kappa = 0.0;
  return e;
}

// 4 m square at 5 cm centred on the origin, elevation f(x, y) at cell centres.
TerrainGrid grid(const std::function<double(double, double)> & f)
{
  TerrainGrid g;
  g.resolution = 0.05;
  g.elevation.resize(81, 81);
  for (int i = 0; i < 81; ++i) {
    for (int j = 0; j < 81; ++j) {g.elevation(i, j) = static_cast<float>(f(g.x(i), g.y(j)));}
  }
  return g;
}

void setNear(Eigen::MatrixXf & m, const TerrainGrid & g, double x, double y, double r, float v)
{
  for (int i = 0; i < g.rows(); ++i) {
    for (int j = 0; j < g.cols(); ++j) {
      if (std::hypot(g.x(i) - x, g.y(j) - y) <= r) {m(i, j) = v;}
    }
  }
}

}  // namespace

TEST(PoseEvaluation, FlatMarginIsTheSideTipAngle)
{
  // atan(0.218 / 0.45) = 0.451126 rad (25.85 deg), smaller than front/rear
  // atan(0.37236 / 0.45) = 0.691272. Matches the benchmark's roll limit: 25.85 - 9.74 = 16.11 deg.
  EXPECT_NEAR(rover_traversability::core::flatStabilityMargin(rover()), 0.451126, 1e-6);
}

TEST(PoseEvaluation, FlatGroundUsesNothing)
{
  const auto r = evaluatePose(grid([](double, double) {return 0.0;}), 0, 0, 0.3, rover(), exact());
  EXPECT_EQ(r.status, PoseStatus::kOk);
  EXPECT_NEAR(r.u, 0.0, 1e-12);
  EXPECT_NEAR(r.margin, 0.451126, 1e-6);
}

TEST(PoseEvaluation, SidehillTenDegrees)
{
  // Ground rising to the left at 10 deg, heading +x. Downhill (right) edge:
  // margin = 0.451126 - 0.174533 = 0.276593, u_tip = 0.174533 / (0.451126 - 0.17) = 0.620835.
  const auto g = grid([](double, double y) {return std::tan(10 * kDeg) * y;});
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_NEAR(r.roll, 10 * kDeg, 1e-6);
  EXPECT_NEAR(r.margin, 0.276593, 1e-5);
  EXPECT_NEAR(r.u_tip, 0.620835, 1e-5);
  EXPECT_EQ(r.status, PoseStatus::kOk);
}

TEST(PoseEvaluation, FacingUpTheSameSlopeIsSafer)
{
  // Same 10 deg slope, heading +y (uphill): now it is pitch. Rear edge margin
  // 0.691272 - 0.174533 = 0.516739 exceeds the side margin 0.451126, so u_tip = 0.
  const auto g = grid([](double, double y) {return std::tan(10 * kDeg) * y;});
  const auto r = evaluatePose(g, 0, 0, M_PI / 2, rover(), exact());
  EXPECT_NEAR(r.pitch, 10 * kDeg, 1e-6);
  EXPECT_NEAR(r.margin, 0.451126, 1e-5);
  EXPECT_NEAR(r.u_tip, 0.0, 1e-5);
}

TEST(PoseEvaluation, UphillTwentyDegreesExact3D)
{
  // Pitch 20 deg: rear margin 0.691272 - 0.349066 = 0.342206. The side edges tilt with the
  // plane but their tip angle stays atan(hy / h) = 0.451126 (exact 3D: in the plane across the
  // tilted edge the vertical projects onto the plane normal). u_tip = (0.451126 - 0.342206) /
  // 0.281126 = 0.387441.
  const auto g = grid([](double x, double) {return std::tan(20 * kDeg) * x;});
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_NEAR(r.margin, 0.342206, 1e-5);
  EXPECT_NEAR(r.u_tip, 0.387441, 1e-5);
}

TEST(PoseEvaluation, OneWheelOnABlock)
{
  // FL contact 0.10 m up, the rest at 0. Rocker difference d = (0.1 - 0) - (0 - 0) = 0.1,
  // u_art = atan(0.1 / 0.744729) / 2 / 0.35 = 0.133477 / 0.7 = 0.190682.
  // Plane: a = 0.1 / (4 hx) = 0.067138, b = 0.1 / (4 hy) = 0.114679.
  auto g = grid([](double, double) {return 0.0;});
  // 0.1 m radius covers all four cells the contact interpolates between, and stays 0.33 m from
  // the centre, outside the 0.267 m belly disc.
  setNear(g.elevation, g, 0.3723645, 0.218, 0.10, 0.10f);
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_NEAR(r.u_art, 0.190682, 1e-5);
  EXPECT_NEAR(std::tan(r.pitch), 0.067138, 1e-5);
  EXPECT_NEAR(std::tan(r.roll), 0.114679, 1e-5);
}

TEST(PoseEvaluation, RockUnderTheBelly)
{
  // 0.20 m rock at the centre, contacts on flat ground: u_clear = 0.20 / 0.25 = 0.8.
  auto g = grid([](double, double) {return 0.0;});
  setNear(g.elevation, g, 0.0, 0.0, 0.03, 0.20f);
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_NEAR(r.clearance_height, 0.20, 1e-6);
  EXPECT_NEAR(r.u_clear, 0.8, 1e-6);
  EXPECT_EQ(r.status, PoseStatus::kOk);
  // The belly disc reaches hypot(0.37236 - 0.14985, 0.218 - 0.07) = 0.267 m. A rock 0.35 m out
  // to the side is outside it.
  auto g2 = grid([](double, double) {return 0.0;});
  setNear(g2.elevation, g2, 0.0, 0.35, 0.02, 0.20f);
  EXPECT_NEAR(evaluatePose(g2, 0, 0, 0.0, rover(), exact()).u_clear, 0.0, 1e-9);
}

TEST(PoseEvaluation, StepOverTheLimitIsLethal)
{
  // Step layer 0.12 m at the RR contact: u_step = 1.2.
  auto g = grid([](double, double) {return 0.0;});
  g.step = Eigen::MatrixXf::Zero(81, 81);
  setNear(g.step, g, -0.3723645, -0.218, 0.04, 0.12f);
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_NEAR(r.u_step, 1.2, 1e-6);
  EXPECT_EQ(r.status, PoseStatus::kLethal);
}

TEST(PoseEvaluation, NegativeObstacleUnderAWheelIsLethal)
{
  auto g = grid([](double, double) {return 0.0;});
  g.negative_obstacle = Eigen::MatrixXf::Zero(81, 81);
  setNear(g.negative_obstacle, g, 0.3723645, -0.218, 0.04, 1.0f);
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_EQ(r.status, PoseStatus::kLethal);
  EXPECT_TRUE(r.negative_contact);
}

TEST(PoseEvaluation, UnknownContactIsUnknown)
{
  auto g = grid([](double, double) {return 0.0;});
  setNear(g.elevation, g, -0.3723645, 0.218, 0.04, NAN);
  EXPECT_EQ(evaluatePose(g, 0, 0, 0.0, rover(), exact()).status, PoseStatus::kUnknown);
  // Off the grid is unknown too.
  EXPECT_EQ(evaluatePose(g, 1.9, 0, 0.0, rover(), exact()).status, PoseStatus::kUnknown);
}

TEST(PoseEvaluation, RoughnessReadsTheCentreCellByDefault)
{
  // The layer is already a support-scale residual, so 0.03 m roughness 0.3 m from the centre
  // does not count, and the same value at the centre gives u_rough = 0.03 / 0.05 = 0.6.
  auto g = grid([](double, double) {return 0.0;});
  g.roughness = Eigen::MatrixXf::Zero(81, 81);
  setNear(g.roughness, g, 0.3, 0.0, 0.02, 0.03f);
  EXPECT_NEAR(evaluatePose(g, 0, 0, 0.0, rover(), exact()).u_rough, 0.0, 1e-6);
  setNear(g.roughness, g, 0.0, 0.0, 0.02, 0.03f);
  EXPECT_NEAR(evaluatePose(g, 0, 0, 0.0, rover(), exact()).u_rough, 0.6, 1e-6);
}

TEST(PoseEvaluation, RoughnessDiscWhenAsked)
{
  // rough_radius -1: the wheel rectangle's circumscribed disc (hypot(0.37236, 0.218) = 0.431 m)
  // takes the 0.03 m cell 0.3 m away, u_rough = 0.6, and misses one 0.5 m away.
  auto g = grid([](double, double) {return 0.0;});
  g.roughness = Eigen::MatrixXf::Zero(81, 81);
  auto e = exact();
  e.rough_radius = -1.0;
  setNear(g.roughness, g, 0.5, 0.0, 0.02, 0.03f);
  EXPECT_NEAR(evaluatePose(g, 0, 0, 0.0, rover(), e).u_rough, 0.0, 1e-6);
  setNear(g.roughness, g, 0.3, 0.0, 0.02, 0.03f);
  EXPECT_NEAR(evaluatePose(g, 0, 0, 0.0, rover(), e).u_rough, 0.6, 1e-6);
}

TEST(PoseEvaluation, UncertaintyMakesItStricter)
{
  // Flat but sigma 0.1 m everywhere, kappa 1. Articulation: d_eff = 0 + sqrt(4 * 0.01) = 0.2,
  // u_art = atan(0.2 / 0.744729) / 0.7. The margin shrinks, so u_tip > 0.
  auto g = grid([](double, double) {return 0.0;});
  g.variance = Eigen::MatrixXf::Constant(81, 81, 0.01f);
  EvalParams e;
  e.risk_kappa = 1.0;
  const auto r = evaluatePose(g, 0, 0, 0.0, rover(), e);
  EXPECT_NEAR(r.u_art, std::atan(0.2 / 0.744729) / 0.7, 1e-6);
  EXPECT_GT(r.u_tip, 0.0);
  EXPECT_NEAR(r.clearance_height, 0.1, 1e-6);     // belly heights raised by sigma
  const auto r0 = evaluatePose(g, 0, 0, 0.0, rover(), exact());
  EXPECT_NEAR(r0.u, 0.0, 1e-12);
}

TEST(PoseEvaluation, CgOffsetLowersTheMarginOnThatSide)
{
  // CG 5 cm to the left: the left edge margin is atan(0.168 / 0.45) = 0.357 rad, the minimum.
  auto p = rover();
  p.cg_offset_y = 0.05;
  EXPECT_NEAR(rover_traversability::core::flatStabilityMargin(p), std::atan(0.168 / 0.45), 1e-6);
}
