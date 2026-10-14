// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// DiscDilation against a brute-force disc reference and hand-computed cases.
// If this fails, the dilation is wrong. Do not "fix" it by relaxing the test.

#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <vector>

#include "rover_traversability/core/disc_dilation.hpp"

using rover_traversability::core::DiscDilation;

namespace
{

constexpr unsigned char kNoInfo = DiscDilation::kNoInformation;

std::vector<unsigned char> reference(
  const std::vector<unsigned char> & src, int size_x, int size_y, int r,
  int min_i, int min_j, int max_i, int max_j)
{
  std::vector<unsigned char> out = src;
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      unsigned char worst = 0;
      for (int dj = -r; dj <= r; ++dj) {
        for (int di = -r; di <= r; ++di) {
          if (di * di + dj * dj > r * r) {continue;}
          const int ii = i + di, jj = j + dj;
          if (ii < 0 || ii >= size_x || jj < 0 || jj >= size_y) {continue;}
          const unsigned char v = src[jj * size_x + ii];
          if (v == kNoInfo) {continue;}
          worst = std::max(worst, v);
        }
      }
      if (worst > 0) {out[j * size_x + i] = worst;}
    }
  }
  return out;
}

int countEqual(const std::vector<unsigned char> & g, unsigned char v)
{
  return static_cast<int>(std::count(g.begin(), g.end(), v));
}

}  // namespace

TEST(DiscDilation, MatchesBruteForceOnRandomGrids)
{
  std::mt19937 rng(20260830);
  int cases = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const int sx = 40 + static_cast<int>(rng() % 40);
    const int sy = 40 + static_cast<int>(rng() % 40);
    const int r = 1 + static_cast<int>(rng() % 10);
    DiscDilation d;
    d.setRadius(r * 0.05, 0.05);
    ASSERT_EQ(d.radiusCells(), r);

    std::vector<unsigned char> g(static_cast<size_t>(sx) * sy, 0);
    for (auto & c : g) {
      const int roll = rng() % 100;
      if (roll < 55) {
        c = 0;
      } else if (roll < 70) {
        c = kNoInfo;
      } else if (roll < 90) {
        c = static_cast<unsigned char>(rng() % 253);
      } else {
        c = 254;
      }
    }
    const auto src = g;

    // Windows flush against every edge and in the middle.
    int min_i, min_j, max_i, max_j;
    switch (trial % 5) {
      case 0: min_i = 0; min_j = 0; max_i = sx; max_j = sy; break;
      case 1: min_i = 0; min_j = 0; max_i = sx / 2; max_j = sy / 2; break;
      case 2: min_i = sx / 2; min_j = sy / 2; max_i = sx; max_j = sy; break;
      case 3: min_i = sx / 4; min_j = sy / 4; max_i = 3 * sx / 4; max_j = 3 * sy / 4; break;
      default: min_i = 1; min_j = sy - 3; max_i = sx - 1; max_j = sy; break;
    }

    d.apply(g.data(), sx, sy, min_i, min_j, max_i, max_j);
    const auto ref = reference(src, sx, sy, r, min_i, min_j, max_i, max_j);
    ASSERT_EQ(g, ref) << "trial " << trial << " r=" << r;
    ++cases;
  }
  EXPECT_EQ(cases, 400);
}

TEST(DiscDilation, DiscNotSquare)
{
  // r = 10. (7,7) is 9.90 away (inside), (8,8) is 11.3 (outside), (10,0) is exactly 10 (inside).
  const int n = 41;
  DiscDilation d;
  d.setRadius(10.0, 1.0);
  std::vector<unsigned char> g(n * n, 0);
  g[20 * n + 20] = 254;
  d.apply(g.data(), n, n, 0, 0, n, n);
  EXPECT_EQ(g[(20 + 7) * n + (20 + 7)], 254);
  EXPECT_EQ(g[(20 + 8) * n + (20 + 8)], 0);
  EXPECT_EQ(g[20 * n + 30], 254);
}

TEST(DiscDilation, RadiusOneIsPlusShape)
{
  const int n = 7;
  DiscDilation d;
  d.setRadius(0.05, 0.05);
  std::vector<unsigned char> g(n * n, 0);
  g[3 * n + 3] = 254;
  d.apply(g.data(), n, n, 0, 0, n, n);
  EXPECT_EQ(countEqual(g, 254), 5);
  EXPECT_EQ(g[3 * n + 2], 254);
  EXPECT_EQ(g[3 * n + 4], 254);
  EXPECT_EQ(g[2 * n + 3], 254);
  EXPECT_EQ(g[4 * n + 3], 254);
  EXPECT_EQ(g[2 * n + 2], 0);
}

TEST(DiscDilation, RadiusTwoIsThirteenCells)
{
  // di^2 + dj^2 <= 4: 1 + 4 (axis, 1) + 4 (axis, 2) + 4 (diagonal 1,1) = 13 cells.
  const int n = 9;
  DiscDilation d;
  d.setRadius(0.10, 0.05);
  std::vector<unsigned char> g(n * n, 0);
  g[4 * n + 4] = 254;
  d.apply(g.data(), n, n, 0, 0, n, n);
  EXPECT_EQ(countEqual(g, 254), 13);
  EXPECT_EQ(g[5 * n + 5], 254);   // (1,1)
  EXPECT_EQ(g[6 * n + 5], 0);     // (1,2), 1 + 4 = 5 > 4
}

TEST(DiscDilation, NoInformationStaysUnknownOutOfRange)
{
  const int n = 9;
  DiscDilation d;
  d.setRadius(0.05, 0.05);
  std::vector<unsigned char> g(n * n, kNoInfo);
  g[4 * n + 4] = 100;
  d.apply(g.data(), n, n, 0, 0, n, n);
  EXPECT_EQ(g[4 * n + 5], 100);   // neighbour picks up the cost
  EXPECT_EQ(g[0], kNoInfo);       // far corner stays unknown
  EXPECT_EQ(countEqual(g, 100), 5);
  EXPECT_EQ(countEqual(g, kNoInfo), n * n - 5);
}

TEST(DiscDilation, HigherCostWinsWhereDiscsOverlap)
{
  const int n = 9;
  DiscDilation d;
  d.setRadius(0.10, 0.05);
  std::vector<unsigned char> g(n * n, 0);
  g[4 * n + 2] = 50;
  g[4 * n + 5] = 200;
  d.apply(g.data(), n, n, 0, 0, n, n);
  EXPECT_EQ(g[4 * n + 3], 200);   // 2 from the 200 cell and 1 from the 50 cell
  EXPECT_EQ(g[4 * n + 2], 50);    // 3 from the 200 cell, outside r = 2
  EXPECT_EQ(g[4 * n + 0], 50);    // 2 from the 50 cell only
}

TEST(DiscDilation, ZeroRadiusIsNoOp)
{
  const int n = 5;
  DiscDilation d;
  d.setRadius(0.0, 0.05);
  EXPECT_EQ(d.radiusCells(), 0);
  std::vector<unsigned char> g(n * n, 0);
  g[12] = 254;
  const auto before = g;
  d.apply(g.data(), n, n, 0, 0, n, n);
  EXPECT_EQ(g, before);
}

TEST(DiscDilation, RadiusRoundsToNearestCell)
{
  DiscDilation d;
  d.setRadius(0.12, 0.05);   // 2.4 cells
  EXPECT_EQ(d.radiusCells(), 2);
  d.setRadius(0.13, 0.05);   // 2.6 cells
  EXPECT_EQ(d.radiusCells(), 3);
}
