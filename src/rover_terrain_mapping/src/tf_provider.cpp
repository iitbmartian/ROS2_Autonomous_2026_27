// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_terrain_mapping/tf_provider.hpp"

#include <chrono>
#include <string>
#include <utility>

#include "tf2_eigen/tf2_eigen.hpp"

namespace rover_terrain_mapping
{

TfProvider::TfProvider(
  rclcpp::Node & node, std::string base_frame, std::string world_frame, double wait_timeout_s)
: logger_(node.get_logger()),
  clock_(node.get_clock()),
  base_frame_(std::move(base_frame)),
  world_frame_(std::move(world_frame)),
  wait_timeout_s_(wait_timeout_s),
  buffer_(std::make_unique<tf2_ros::Buffer>(node.get_clock())),
  listener_(std::make_unique<tf2_ros::TransformListener>(*buffer_))
{
}

std::optional<Eigen::Isometry3d> TfProvider::lookup(
  const std::string & target, const std::string & source, const tf2::TimePoint & t) const
{
  try {
    const auto tf = buffer_->lookupTransform(
      target, source, t, tf2::durationFromSec(wait_timeout_s_));
    return tf2::transformToEigen(tf);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "terrain_mapping TF: %s", e.what());
    return std::nullopt;
  }
}

std::optional<Eigen::Isometry3d> TfProvider::getExtrinsic(const std::string & sensor_frame) const
{
  if (auto it = extrinsics_.find(sensor_frame); it != extrinsics_.end()) {
    return it->second;
  }
  auto T = lookup(base_frame_, sensor_frame, tf2::TimePointZero);
  if (T) {
    extrinsics_[sensor_frame] = *T;
  }
  return T;
}

std::optional<Eigen::Isometry3d> TfProvider::getPoseAt(uint64_t timestamp_ns) const
{
  return lookup(
    world_frame_, base_frame_, tf2::TimePoint(std::chrono::nanoseconds(timestamp_ns)));
}

}  // namespace rover_terrain_mapping
