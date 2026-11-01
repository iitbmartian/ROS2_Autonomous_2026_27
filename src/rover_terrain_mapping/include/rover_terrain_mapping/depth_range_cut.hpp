// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// depth_range_cut: the Orin-side reduction of the RGB-D cloud before it crosses the network to
// the NUC (SPEC.md, compute split). Points beyond max_range carry noise growing with range
// squared and add little, so they are dropped, and the rest is voxel-downsampled.

#ifndef ROVER_TERRAIN_MAPPING__DEPTH_RANGE_CUT_HPP_
#define ROVER_TERRAIN_MAPPING__DEPTH_RANGE_CUT_HPP_

#include "fastdem/point_types.hpp"

namespace rover_terrain_mapping
{

struct RangeCutParams
{
  float min_range{0.3f};     // m from the sensor origin
  float max_range{4.0f};     // m
  float voxel_size{0.05f};   // m, 0 disables downsampling
};

// Keeps points with min_range <= range <= max_range (range from the cloud's own origin, the
// sensor), then keeps one centroid per voxel. Frame and stamp are preserved.
fastdem::PointCloud rangeCut(fastdem::PointCloud cloud, const RangeCutParams & p);

}  // namespace rover_terrain_mapping

#endif  // ROVER_TERRAIN_MAPPING__DEPTH_RANGE_CUT_HPP_
