// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// FastDEM transform provider backed by tf2: the sensor extrinsic (base <- sensor) and the
// robot pose (map frame <- base) at a scan's stamp.

#ifndef ROVER_TERRAIN_MAPPING__TF_PROVIDER_HPP_
#define ROVER_TERRAIN_MAPPING__TF_PROVIDER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "fastdem/transform_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rover_terrain_mapping
{

class TfProvider : public fastdem::Calibration, public fastdem::Odometry
{
public:
  TfProvider(
    rclcpp::Node & node, std::string base_frame, std::string world_frame,
    double wait_timeout_s);

  std::string getBaseFrame() const override {return base_frame_;}
  std::string getWorldFrame() const override {return world_frame_;}

  // Cached per sensor frame. The sensors are rigidly mounted, so the first lookup holds.
  std::optional<Eigen::Isometry3d> getExtrinsic(const std::string & sensor_frame) const override;

  // Pose at the scan stamp, interpolated by tf2. No fallback to the latest pose: a scan placed
  // with the wrong pose writes wrong terrain.
  std::optional<Eigen::Isometry3d> getPoseAt(uint64_t timestamp_ns) const override;

private:
  std::optional<Eigen::Isometry3d> lookup(
    const std::string & target, const std::string & source, const tf2::TimePoint & t) const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string base_frame_;
  std::string world_frame_;
  double wait_timeout_s_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  mutable std::unordered_map<std::string, Eigen::Isometry3d> extrinsics_;
};

}  // namespace rover_terrain_mapping

#endif  // ROVER_TERRAIN_MAPPING__TF_PROVIDER_HPP_
