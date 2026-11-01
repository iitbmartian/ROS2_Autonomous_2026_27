// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include "rover_terrain_mapping/depth_range_cut.hpp"

using rover_terrain_mapping::rangeCut;
using rover_terrain_mapping::RangeCutParams;

TEST(DepthRangeCut, KeepsOnlyTheRangeBand)
{
  fastdem::PointCloud c;
  c.setFrameId("camera_optical_frame");
  c.setTimestamp(42);
  c.add(0.0f, 0.0f, 0.1f);   // 0.1 m, too close
  c.add(0.0f, 0.0f, 1.0f);   // kept
  c.add(3.0f, 0.0f, 2.0f);   // 3.61 m, kept
  c.add(0.0f, 0.0f, 6.0f);   // too far
  RangeCutParams p;
  p.voxel_size = 0.0f;
  const auto out = rangeCut(c, p);
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(out.frameId(), "camera_optical_frame");
  EXPECT_EQ(out.timestamp(), 42u);
}

TEST(DepthRangeCut, VoxelMergesPointsInOneCell)
{
  // Ten points inside one 5 cm voxel and one far apart: two points out, the first at the
  // centroid of the ten.
  fastdem::PointCloud c;
  for (int k = 0; k < 10; ++k) {c.add(1.01f + 0.001f * k, 0.01f, 0.01f);}
  c.add(2.01f, 0.01f, 0.01f);
  RangeCutParams p;
  const auto out = rangeCut(c, p);
  ASSERT_EQ(out.size(), 2u);
}
