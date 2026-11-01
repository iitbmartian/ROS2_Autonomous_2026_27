// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_terrain_mapping/depth_range_cut.hpp"

#include <string>
#include <utility>

#include "nanopcl/filters/crop.hpp"
#include "nanopcl/filters/downsample.hpp"

namespace rover_terrain_mapping
{

fastdem::PointCloud rangeCut(fastdem::PointCloud cloud, const RangeCutParams & p)
{
  const std::string frame = cloud.frameId();
  const auto stamp = cloud.timestamp();
  auto out = nanopcl::filters::cropRange(std::move(cloud), p.min_range, p.max_range);
  if (p.voxel_size > 0.0f) {
    out = nanopcl::filters::voxelGrid(std::move(out), p.voxel_size);
  }
  out.setFrameId(frame);
  out.setTimestamp(stamp);
  return out;
}

}  // namespace rover_terrain_mapping
