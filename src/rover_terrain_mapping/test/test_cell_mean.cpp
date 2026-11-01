// Copyright 2026 Mars Rover Team, IIT Bombay.
// PATCH 3 to vendored FastDEM: the per-scan cell measurement is the mean of the cell's points
// when mapping.cell_mean is set, the lowest point otherwise (upstream).

#include <gtest/gtest.h>

#include "fastdem/mapping/elevation_mapping.hpp"

namespace
{

float elevationAfterOneScan(bool cell_mean)
{
  fastdem::ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  fastdem::config::Mapping cfg;
  cfg.cell_mean = cell_mean;
  fastdem::ElevationMapping mapping(map, cfg);
  fastdem::PointCloud cloud;
  // Three returns in the 1 m cell at the origin: mean 0.01, lowest -0.03.
  cloud.add(0.1f, 0.1f, 0.00f);
  cloud.add(0.2f, 0.2f, 0.06f);
  cloud.add(0.3f, 0.3f, -0.03f);
  mapping.estimate(mapping.rasterize(cloud));
  return map.elevationAt(nanogrid::Position(0.2, 0.2));
}

}  // namespace

TEST(CellMean, MeanOfCellPoints)
{
  EXPECT_NEAR(elevationAfterOneScan(true), 0.01f, 1e-6);
}

TEST(CellMean, UpstreamTakesLowestPoint)
{
  EXPECT_NEAR(elevationAfterOneScan(false), -0.03f, 1e-6);
}

TEST(CellMean, MeanVarianceShrinksWithPointCount)
{
  // Second scan: one point at 0.10 with variance 0.0004 against a first scan of four points
  // at 0 with variance 0.0004 each, so a measurement variance of 0.0001. Kalman gain
  // 0.0001 / (0.0001 + 0.0004) = 0.2, estimate 0.02.
  fastdem::ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  fastdem::config::Mapping cfg;
  cfg.cell_mean = true;
  cfg.kalman.min_variance = 1e-6f;
  fastdem::ElevationMapping mapping(map, cfg);
  fastdem::PointCloud first, second;
  for (int k = 1; k <= 4; ++k) {first.add(0.1f * k, 0.1f, 0.0f);}
  second.add(0.2f, 0.2f, 0.10f);
  first.useCovariance();
  second.useCovariance();
  for (size_t i = 0; i < first.size(); ++i) {first.covariance(i)(2, 2) = 0.0004f;}
  second.covariance(0)(2, 2) = 0.0004f;
  mapping.estimate(mapping.rasterize(first));
  mapping.estimate(mapping.rasterize(second));
  EXPECT_NEAR(map.elevationAt(nanogrid::Position(0.2, 0.2)), 0.02f, 1e-6);
}
