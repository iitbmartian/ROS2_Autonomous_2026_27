// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <vector>

#include "rover_traversability/is_path_lethal_free.hpp"

using rover_traversability::firstLethalPose;

// 4 x 3 grid at 0.5 m, origin (-1, 0). Cell (cx, cy) covers x in [-1 + 0.5 cx, ...).
static std::vector<uint8_t> grid()
{
  std::vector<uint8_t> d(12, 0);
  d[1 * 4 + 2] = 254;   // cell (2, 1): x [0, 0.5), y [0.5, 1.0), lethal
  d[0 * 4 + 1] = 253;   // cell (1, 0): inscribed, must pass
  d[2 * 4 + 3] = 255;   // cell (3, 2): unknown, must pass
  return d;
}

TEST(IsPathLethalFree, FreePathPasses)
{
  EXPECT_EQ(firstLethalPose(grid(), 4, 3, 0.5, -1.0, 0.0, {-0.9, -0.4}, {0.1, 0.1}), -1);
}

TEST(IsPathLethalFree, InscribedAndUnknownPass)
{
  // (-0.3, 0.2) -> cell (1, 0) inscribed, (0.7, 1.2) -> cell (3, 2) unknown.
  EXPECT_EQ(firstLethalPose(grid(), 4, 3, 0.5, -1.0, 0.0, {-0.3, 0.7}, {0.2, 1.2}), -1);
}

TEST(IsPathLethalFree, FindsFirstLethal)
{
  // pose 1 (0.2, 0.7) -> cell (2, 1) lethal, pose 2 also lethal, the first is reported.
  EXPECT_EQ(firstLethalPose(grid(), 4, 3, 0.5, -1.0, 0.0, {-0.9, 0.2, 0.4}, {0.1, 0.7, 0.9}), 1);
}

TEST(IsPathLethalFree, OutsideGridSkipped)
{
  // x = -1.2 and x = 1.1 are off the 2 m wide grid, y = 1.6 is off its 1.5 m height.
  EXPECT_EQ(firstLethalPose(grid(), 4, 3, 0.5, -1.0, 0.0, {-1.2, 1.1, 0.2}, {0.7, 0.7, 1.6}), -1);
}

TEST(IsPathLethalFree, BadGridIsFree)
{
  EXPECT_EQ(firstLethalPose({}, 4, 3, 0.5, -1.0, 0.0, {0.2}, {0.7}), -1);
  EXPECT_EQ(firstLethalPose(grid(), 4, 3, 0.0, -1.0, 0.0, {0.2}, {0.7}), -1);
}
