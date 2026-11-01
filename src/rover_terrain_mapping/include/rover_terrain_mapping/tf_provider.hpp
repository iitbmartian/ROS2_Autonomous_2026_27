// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// FastDEM transform provider backed by tf2: the sensor extrinsic (base <- sensor) and the
// robot pose (map frame <- base) at a scan's stamp.

#ifndef ROVER_TERRAIN_MAPPING__TF_PROVIDER_HPP_
#define ROVER_TERRAIN_MAPPING__TF_PROVIDER_HPP_

#include <memory>
#include <mutex>
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
  // with the wrong pose writes wrong terrain. With an IMU attitude set (setImuAttitude), roll
  // and pitch come from the IMU instead of TF, which keeps the map gravity-aligned when the
  // odometry is planar (an EKF in two_d_mode). Position and yaw always come from TF.
  std::optional<Eigen::Isometry3d> getPoseAt(uint64_t timestamp_ns) const override;

  // Latest IMU orientation (world <- imu frame) and the IMU's frame, from the node's IMU
  // callback. Only roll and pitch are used, so the IMU's yaw reference does not matter.
  void setImuAttitude(const Eigen::Quaterniond & q_world_imu, const std::string & imu_frame,
    uint64_t stamp_ns);
  // An IMU attitude older than this (s) is not used and the pose falls back to TF entirely.
  void setImuTimeout(double s) {imu_timeout_s_ = s;}

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

  mutable std::mutex imu_mutex_;
  bool have_imu_{false};
  Eigen::Quaterniond q_world_imu_{Eigen::Quaterniond::Identity()};
  std::string imu_frame_;
  uint64_t imu_stamp_ns_{0};
  double imu_timeout_s_{0.5};
};

// Replaces the roll and pitch of T_world_base with those of R_world_base_from_imu, keeping
// T_world_base's position and yaw (ZYX convention). Exposed for tests.
Eigen::Isometry3d withRollPitch(
  const Eigen::Isometry3d & T_world_base, const Eigen::Matrix3d & R_world_base_from_imu);

}  // namespace rover_terrain_mapping

#endif  // ROVER_TERRAIN_MAPPING__TF_PROVIDER_HPP_
