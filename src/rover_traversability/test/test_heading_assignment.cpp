// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rover_traversability/core/heading_assignment.hpp"

using namespace rover_traversability::core;   // NOLINT

namespace
{
using Grid = std::vector<std::vector<double>>;
Grid flat(int n, int B, double u = 0.1) {return Grid(n, std::vector<double>(B, u));}
}

// Flat ground, travel east: hold heading 0, no crab, no reverse.
TEST(HeadingAssignment, FlatGroundFacesTravel)
{
  HeadingParams p;
  auto r = assignHeadings(flat(5, 16), std::vector<double>(5, 0.0), p);
  ASSERT_TRUE(r.feasible);
  for (int b : r.bin) {EXPECT_EQ(b, 0);}
}

// Travel north on flat ground: bin 4 (pi/2).
TEST(HeadingAssignment, FlatGroundNorth)
{
  auto r = assignHeadings(flat(3, 16), std::vector<double>(3, M_PI / 2), HeadingParams{});
  for (double y : r.yaw) {EXPECT_NEAR(y, M_PI / 2, 1e-9);}
}

// Side slope: the travel heading (0) and its reverse (8) are unsafe on poses 1..3, the
// cross-slope headings 4 and 12 are safe. Swerve crabs through, holding 4 or 12.
TEST(HeadingAssignment, SwerveCrabsAcrossUnsafeTravelHeading)
{
  Grid u = flat(5, 16);
  for (int i = 1; i <= 3; ++i) {
    for (int b = 0; b < 16; ++b) {if (b != 4 && b != 12) {u[i][b] = 1.5;}}
  }
  auto r = assignHeadings(u, std::vector<double>(5, 0.0), HeadingParams{});
  ASSERT_TRUE(r.feasible);
  for (int i = 1; i <= 3; ++i) {EXPECT_TRUE(r.bin[i] == 4 || r.bin[i] == 12) << i;}
}

// Same ground with crabbing off (Ackermann): no allowed heading is safe on pose 1.
TEST(HeadingAssignment, AckermannReportsInfeasible)
{
  Grid u = flat(5, 16);
  for (int i = 1; i <= 3; ++i) {
    for (int b = 0; b < 16; ++b) {if (b != 4 && b != 12) {u[i][b] = 1.5;}}
  }
  HeadingParams p; p.allow_crab = false;
  auto r = assignHeadings(u, std::vector<double>(5, 0.0), p);
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.first_infeasible, 1);
}

// Ackermann may reverse: forward unsafe, reverse safe, so it picks bin 8 there.
TEST(HeadingAssignment, AckermannReversesWhenForwardUnsafe)
{
  Grid u = flat(3, 16);
  for (int i = 0; i < 3; ++i) {u[i][0] = 1.2;}
  HeadingParams p; p.allow_crab = false;
  auto r = assignHeadings(u, std::vector<double>(3, 0.0), p);
  ASSERT_TRUE(r.feasible);
  for (int b : r.bin) {EXPECT_EQ(b, 8);}
}

// Turn cost: a lone pose where bin 1 is a little cheaper is not worth two turns.
// Cost with bin 0 everywhere: 3 * 0.1 = 0.3. Switching pose 1 to bin 1 saves 0.1 in u, adds
// crab 0.5 * sin(pi/8) = 0.191 and turns 2 * 0.3 = 0.6. Stay at 0.
TEST(HeadingAssignment, TurnCostKeepsHeadingSteady)
{
  Grid u = flat(3, 16);
  u[1][1] = 0.0;
  auto r = assignHeadings(u, std::vector<double>(3, 0.0), HeadingParams{});
  EXPECT_EQ(r.bin[1], 0);
}

// Unknown cells (NaN) are unsafe.
TEST(HeadingAssignment, UnknownIsUnsafe)
{
  Grid u = flat(2, 4, NAN);
  auto r = assignHeadings(u, {0.0, 0.0}, HeadingParams{4});
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.first_infeasible, 0);
}

TEST(HeadingAssignment, TravelDirections)
{
  auto t = travelDirections({0, 1, 1}, {0, 0, 1});
  ASSERT_EQ(t.size(), 3u);
  EXPECT_NEAR(t[0], 0.0, 1e-12);
  EXPECT_NEAR(t[1], M_PI / 2, 1e-12);
  EXPECT_NEAR(t[2], M_PI / 2, 1e-12);
  EXPECT_TRUE(travelDirections({}, {}).empty());
}

TEST(HeadingAssignment, EmptyPathIsFeasible)
{
  auto r = assignHeadings({}, {}, HeadingParams{});
  EXPECT_TRUE(r.feasible);
  EXPECT_TRUE(r.yaw.empty());
}
