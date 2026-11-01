// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_terrain_mapping/tf_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
  auto T = lookup(
    world_frame_, base_frame_, tf2::TimePoint(std::chrono::nanoseconds(timestamp_ns)));
  if (!T) {
    return T;
  }
  Eigen::Quaterniond q;
  std::string imu_frame;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!have_imu_) {
      return T;
    }
    const double age = std::abs(static_cast<double>(timestamp_ns) -
      static_cast<double>(imu_stamp_ns_)) * 1e-9;
    if (age > imu_timeout_s_) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
        "terrain_mapping: IMU attitude %.2f s from the scan, using TF attitude", age);
      return T;
    }
    q = q_world_imu_;
    imu_frame = imu_frame_;
  }
  // R_world_base = R_world_imu * R_imu_base. getExtrinsic gives base <- imu, the mount from the
  // static TF, so R_imu_base is its transpose.
  const auto T_base_imu = getExtrinsic(imu_frame);
  if (!T_base_imu) {
    return T;
  }
  const Eigen::Matrix3d R = q.toRotationMatrix() * T_base_imu->linear().transpose();
  return withRollPitch(*T, R);
}

void TfProvider::setImuAttitude(
  const Eigen::Quaterniond & q_world_imu, const std::string & imu_frame, uint64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  q_world_imu_ = q_world_imu.normalized();
  imu_frame_ = imu_frame;
  imu_stamp_ns_ = stamp_ns;
  have_imu_ = true;
}

Eigen::Isometry3d withRollPitch(
  const Eigen::Isometry3d & T_world_base, const Eigen::Matrix3d & R_world_base_from_imu)
{
  // ZYX: R = Rz(yaw) Ry(pitch) Rx(roll). Yaw from TF, roll and pitch from the IMU.
  const Eigen::Matrix3d & Rt = T_world_base.linear();
  const double yaw = std::atan2(Rt(1, 0), Rt(0, 0));
  const Eigen::Matrix3d & Ri = R_world_base_from_imu;
  const double pitch = std::asin(std::clamp(-Ri(2, 0), -1.0, 1.0));
  const double roll = std::atan2(Ri(2, 1), Ri(2, 2));
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  out.translation() = T_world_base.translation();
  return out;
}

}  // namespace rover_terrain_mapping
