// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Shared by the speed governor and the safety monitor: the latest feature map as a TerrainGrid,
// with its pose-evaluation cache and stamp.

#ifndef ROVER_TRAVERSABILITY__NODES__TERRAIN_MAP_INPUT_HPP_
#define ROVER_TRAVERSABILITY__NODES__TERRAIN_MAP_INPUT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_traversability/core/pose_evaluation.hpp"
#include "rover_traversability/grid_map_msg.hpp"
#include "rover_traversability/layer_names.hpp"

namespace rover_traversability
{

struct TerrainSnapshot
{
  core::TerrainGrid grid;
  core::EvalCache cache;
  Eigen::MatrixXf best_utilisation, safe_yaw_mask;
  std::string frame;
  rclcpp::Time stamp;
};

inline std::shared_ptr<TerrainSnapshot> snapshotFrom(
  const grid_map_msgs::msg::GridMap & msg, const core::PhysicalParams & p,
  const core::EvalParams & e, rcl_clock_type_t clock_type)
{
  auto t = std::make_shared<TerrainSnapshot>();
  if (!readLayer(msg, layers::kElevation, t->grid.elevation)) {return nullptr;}
  const auto geo = geometryOf(msg);
  t->grid.resolution = geo.resolution;
  t->grid.center_x = geo.center_x;
  t->grid.center_y = geo.center_y;
  readLayer(msg, layers::kVariance, t->grid.variance);
  readLayer(msg, layers::kStep, t->grid.step);
  readLayer(msg, layers::kRoughness, t->grid.roughness);
  readLayer(msg, layers::kNegativeObstacle, t->grid.negative_obstacle);
  readLayer(msg, layers::kBestUtilisation, t->best_utilisation);
  readLayer(msg, layers::kSafeYawMask, t->safe_yaw_mask);
  t->cache = core::makeEvalCache(t->grid, p, e);
  t->frame = msg.header.frame_id;
  t->stamp = rclcpp::Time(msg.header.stamp, clock_type);
  return t;
}

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__NODES__TERRAIN_MAP_INPUT_HPP_
