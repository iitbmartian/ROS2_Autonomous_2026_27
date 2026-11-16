// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>

#include "rover_traversability/core/heading_aware_search.hpp"

using namespace rover_traversability::core;   // NOLINT

namespace
{

// 12 m x 8 m at 0.2 m, origin at (0, -4), all free.
SearchGrid open()
{
  SearchGrid g;
  g.nx = 60; g.ny = 40; g.resolution = 0.2; g.origin_x = 0.0; g.origin_y = -4.0;
  g.cost.assign(g.nx * g.ny, 0);
  return g;
}

// Band x 5 to 7 where only the given bins are safe, like a ramp up along x (bins 0 and 8).
void band(SearchGrid & g, uint16_t allowed)
{
  g.mask.assign(g.nx * g.ny, 0xFFFF);
  for (int iy = 0; iy < g.ny; ++iy) {
    for (int ix = 25; ix < 35; ++ix) {g.mask[iy * g.nx + ix] = allowed;}
  }
}

bool inBand(const SearchPose & s) {return s.x >= 5.0 && s.x < 7.0;}

}  // namespace

TEST(HeadingAwareSearch, StraightOnOpenGround)
{
  auto r = headingAwareSearch(open(), SearchParams{}, {1.0, 0.1, 0.0}, 6.0, 0.1);
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.path.back().x, 6.0, 1e-9);
  for (const auto & s : r.path) {EXPECT_NEAR(std::sin(s.yaw), 0.0, 1e-9);}
}

TEST(HeadingAwareSearch, AvoidsLethalCells)
{
  auto g = open();
  for (int iy = 10; iy < 40; ++iy) {g.cost[iy * g.nx + 20] = 254;}   // wall at x 4, y > -2
  auto r = headingAwareSearch(g, SearchParams{}, {1.0, 1.0, 0.0}, 8.0, 1.0);
  ASSERT_TRUE(r.found);
  bool below = false;
  for (const auto & s : r.path) {
    int i;
    ASSERT_TRUE(g.cell(s.x, s.y, i));
    EXPECT_LT(g.cost[i], 253);
    below |= s.y < -2.0;
  }
  EXPECT_TRUE(below);
}

// The goal lies across the band and 3 m to the side. Without the mask the path enters the band
// diagonally. With only bins 0 and 8 safe there, every pose in the band faces along x.
TEST(HeadingAwareSearch, LinesUpBeforeAHeadingRestrictedBand)
{
  SearchParams p;
  auto free = headingAwareSearch(open(), p, {1.0, 2.5, 0.0}, 10.0, -0.5);
  ASSERT_TRUE(free.found);
  bool diagonal = false;
  for (const auto & s : free.path) {diagonal |= inBand(s) && std::abs(std::sin(s.yaw)) > 0.1;}
  EXPECT_TRUE(diagonal) << "test proves nothing: the unconstrained path is already aligned";

  auto g = open();
  band(g, (1u << 0) | (1u << 8));
  auto r = headingAwareSearch(g, p, {1.0, 2.5, 0.0}, 10.0, -0.5);
  ASSERT_TRUE(r.found);
  int in = 0;
  for (const auto & s : r.path) {
    if (inBand(s)) {++in; EXPECT_NEAR(std::sin(s.yaw), 0.0, 1e-9) << s.x << " " << s.y;}
  }
  EXPECT_GT(in, 3);
  EXPECT_NEAR(r.path.back().x, 10.0, 1e-9);
}

// Only cross-band headings are safe in a band spanning the whole map: a car cannot cross it.
TEST(HeadingAwareSearch, NoPathWhenNoAllowedHeadingCrosses)
{
  auto g = open();
  band(g, (1u << 4) | (1u << 12));
  SearchParams p;
  p.max_iterations = 200000;
  auto r = headingAwareSearch(g, p, {1.0, 0.0, 0.0}, 10.0, 0.0);
  EXPECT_FALSE(r.found);
  EXPECT_FALSE(r.timed_out);
}

// Facing away from a goal straight behind, a short reverse is cheaper than a turn around.
TEST(HeadingAwareSearch, ReversesToAGoalJustBehind)
{
  auto r = headingAwareSearch(open(), SearchParams{}, {5.0, 0.1, 0.0}, 4.0, 0.1);
  ASSERT_TRUE(r.found);
  for (const auto & s : r.path) {EXPECT_NEAR(std::sin(s.yaw), 0.0, 1e-9);}
  EXPECT_LT(r.path.size(), 8u);
}

TEST(HeadingAwareSearch, StartOrGoalOutsideTheGrid)
{
  EXPECT_FALSE(headingAwareSearch(open(), SearchParams{}, {-1.0, 0.0, 0.0}, 5.0, 0.0).found);
  EXPECT_FALSE(headingAwareSearch(open(), SearchParams{}, {1.0, 0.0, 0.0}, 50.0, 0.0).found);
}

// Full global costmap size (100 m at 0.2 m) with scattered 1 m rocks: a 127 m diagonal plan
// must finish inside the planner's 2 s limit.
TEST(HeadingAwareSearch, PlansAcrossTheFullMapInTime)
{
  SearchGrid g;
  g.nx = 500; g.ny = 500; g.resolution = 0.2;
  g.cost.assign(g.nx * g.ny, 0);
  unsigned seed = 7;
  for (int k = 0; k < 400; ++k) {
    seed = seed * 1103515245u + 12345u; const int cx = 10 + (seed >> 8) % 480;
    seed = seed * 1103515245u + 12345u; const int cy = 10 + (seed >> 8) % 480;
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {g.cost[(cy + dy) * g.nx + cx + dx] = 254;}
    }
  }
  g.cost[25 * g.nx + 25] = 0;
  SearchParams p;
  auto r = headingAwareSearch(g, p, {5.1, 5.1, 0.0}, 95.0, 95.0);
  EXPECT_TRUE(r.found) << "iterations " << r.iterations << (r.timed_out ? " timed out" : "");
}
