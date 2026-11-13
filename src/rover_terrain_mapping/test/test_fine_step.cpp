// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Fine local step carried into the coarse global map, against hand-computed cases.
// Coarse: 4 x 4 cells of 0.2 m centred at (0, 0), so row 0 spans x 0.2 to 0.4, row 3 spans
// x -0.4 to -0.2, column 0 spans y 0.2 to 0.4. Fine: 0.05 m, 4 x 4 fine cells per coarse cell.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rover_terrain_mapping/fine_step.hpp"

using rover_terrain_mapping::applyFineStep;
using rover_terrain_mapping::CellGrid;
using rover_terrain_mapping::fuseFineStep;
using rover_terrain_mapping::Planar;

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

CellGrid coarse() {return CellGrid{4, 4, 0.2, 0.0, 0.0};}
CellGrid fine(int n, double cx = 0.0, double cy = 0.0) {return CellGrid{n, n, 0.05, cx, cy};}

}  // namespace

TEST(FineStep, CellIndexing)
{
  const CellGrid g = coarse();
  EXPECT_EQ(g.row(0.39), 0);
  EXPECT_EQ(g.row(0.01), 1);
  EXPECT_EQ(g.row(-0.01), 2);
  EXPECT_EQ(g.row(-0.39), 3);
  EXPECT_EQ(g.row(0.41), -1);
  EXPECT_EQ(g.row(-0.41), -1);
  EXPECT_EQ(g.col(0.39), 0);
  EXPECT_NEAR(g.x(0), 0.3, 1e-9);
  EXPECT_NEAR(g.y(3), -0.3, 1e-9);
}

TEST(FineStep, SingleRockCellReachesItsCoarseCell)
{
  // A 0.12 m step in one fine cell, everything else flat, same extent and frame.
  Eigen::MatrixXf f = Eigen::MatrixXf::Zero(16, 16);
  f(5, 9) = 0.12f;   // fine centre x = 0.4 - 5.5 * 0.05 = 0.125, y = 0.4 - 9.5 * 0.05 = -0.075
  Eigen::MatrixXf mem;
  fuseFineStep(mem, coarse(), f, fine(16), Planar{});
  EXPECT_FLOAT_EQ(mem(1, 2), 0.12f);   // x 0 to 0.2, y -0.2 to 0
  EXPECT_FLOAT_EQ(mem(0, 0), 0.0f);
  EXPECT_FLOAT_EQ(mem(3, 3), 0.0f);
}

TEST(FineStep, FullyCoveredCellIsReplaced)
{
  Eigen::MatrixXf mem = Eigen::MatrixXf::Constant(4, 4, 0.3f);
  Eigen::MatrixXf f = Eigen::MatrixXf::Zero(16, 16);
  fuseFineStep(mem, coarse(), f, fine(16), Planar{});
  EXPECT_FLOAT_EQ(mem(1, 1), 0.0f);    // rock gone on a full close look
}

TEST(FineStep, PartlyCoveredCellKeepsTheOldRock)
{
  // A 2 x 2 fine map (0.1 m) centred at (0.05, 0.05) covers a quarter of coarse cell (1, 1),
  // under the 0.5 cover needed to replace, so the 0.3 seen earlier stays.
  Eigen::MatrixXf mem = Eigen::MatrixXf::Constant(4, 4, kNaN);
  mem(1, 1) = 0.3f;
  Eigen::MatrixXf f = Eigen::MatrixXf::Zero(2, 2);
  fuseFineStep(mem, coarse(), f, fine(2, 0.05, 0.05), Planar{});
  EXPECT_FLOAT_EQ(mem(1, 1), 0.3f);
  EXPECT_TRUE(std::isnan(mem(0, 0)));   // never seen
}

TEST(FineStep, UnknownFineCellsAreIgnored)
{
  Eigen::MatrixXf mem = Eigen::MatrixXf::Constant(4, 4, 0.3f);
  Eigen::MatrixXf f = Eigen::MatrixXf::Constant(16, 16, kNaN);
  fuseFineStep(mem, coarse(), f, fine(16), Planar{});
  EXPECT_FLOAT_EQ(mem(2, 2), 0.3f);
}

TEST(FineStep, TransformMovesTheRock)
{
  // Fine frame at (0.2, 0) rotated +90 deg in the coarse frame. A fine cell at fine (0.125,
  // -0.075) lands at coarse (0.2 + 0.075, 0.125) = (0.275, 0.125): row 0, column 1.
  Eigen::MatrixXf f = Eigen::MatrixXf::Constant(16, 16, kNaN);
  f(5, 9) = 0.12f;
  Eigen::MatrixXf mem;
  fuseFineStep(mem, coarse(), f, fine(16), Planar{0.2, 0.0, M_PI / 2.0});
  EXPECT_FLOAT_EQ(mem(0, 1), 0.12f);
  EXPECT_TRUE(std::isnan(mem(1, 2)));
}

TEST(FineStep, ApplyTakesTheLargerAndKeepsUnknown)
{
  Eigen::MatrixXf step(1, 3), mem(1, 3);
  step << 0.02f, 0.2f, kNaN;
  mem << 0.12f, 0.05f, 0.3f;
  applyFineStep(step, mem);
  EXPECT_FLOAT_EQ(step(0, 0), 0.12f);
  EXPECT_FLOAT_EQ(step(0, 1), 0.2f);
  EXPECT_TRUE(std::isnan(step(0, 2)));
}
