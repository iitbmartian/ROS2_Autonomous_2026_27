// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include "rover_terrain_mapping/grid_map_msg.hpp"

using rover_terrain_mapping::geometryOf;
using rover_terrain_mapping::readLayer;
using rover_terrain_mapping::writeLayer;

namespace
{

grid_map_msgs::msg::GridMap emptyMsg(int rows, int cols, double res)
{
  grid_map_msgs::msg::GridMap m;
  m.info.resolution = res;
  m.info.length_x = rows * res;
  m.info.length_y = cols * res;
  m.info.pose.position.x = 2.0;
  m.info.pose.position.y = -1.0;
  return m;
}

}  // namespace

TEST(GridMapMsg, RoundTrip)
{
  auto m = emptyMsg(3, 4, 0.5);
  Eigen::MatrixXf a(3, 4);
  a << 0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23;
  writeLayer(m, "elevation", a);
  Eigen::MatrixXf b;
  ASSERT_TRUE(readLayer(m, "elevation", b));
  EXPECT_EQ(a, b);
  EXPECT_FALSE(readLayer(m, "missing", b));
}

TEST(GridMapMsg, CircularStartIndexUnwraps)
{
  // Stored data is the unwrapped grid rotated by one row and two columns.
  auto m = emptyMsg(3, 4, 0.5);
  Eigen::MatrixXf unwrapped(3, 4);
  unwrapped << 0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23;
  Eigen::MatrixXf stored(3, 4);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {stored((i + 1) % 3, (j + 2) % 4) = unwrapped(i, j);}
  }
  writeLayer(m, "z", stored);
  m.outer_start_index = 1;
  m.inner_start_index = 2;
  Eigen::MatrixXf got;
  ASSERT_TRUE(readLayer(m, "z", got));
  EXPECT_EQ(got, unwrapped);
}

TEST(GridMapMsg, GeometryCellCentres)
{
  // 3 x 4 cells of 0.5 m centred at (2, -1): row 0 is the +x edge, its centre at
  // 2 + 0.75 - 0.25 = 2.5. Column 0 is the +y edge, centre -1 + 1.0 - 0.25 = -0.25.
  const auto g = geometryOf(emptyMsg(3, 4, 0.5));
  EXPECT_EQ(g.rows, 3);
  EXPECT_EQ(g.cols, 4);
  EXPECT_DOUBLE_EQ(g.x(0), 2.5);
  EXPECT_DOUBLE_EQ(g.y(0), -0.25);
  EXPECT_DOUBLE_EQ(g.row(2.5), 0.0);
  EXPECT_DOUBLE_EQ(g.col(-0.25), 0.0);
  EXPECT_DOUBLE_EQ(g.row(g.x(1.7)), 1.7);
}
