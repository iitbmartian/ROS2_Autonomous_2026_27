// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Geometry>

#include "rover_terrain_mapping/tf_provider.hpp"

using rover_terrain_mapping::withRollPitch;

namespace
{
Eigen::Matrix3d rpy(double r, double p, double y)
{
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
}  // namespace

TEST(TfProvider, RollPitchFromImuYawAndPositionFromTf)
{
  // Planar odometry: yaw 0.7, no tilt, at (1, 2, 0.1). IMU: roll 0.1, pitch -0.2, and a yaw of
  // 2.0 that must be ignored.
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = rpy(0.0, 0.0, 0.7);
  T.translation() << 1.0, 2.0, 0.1;
  const auto out = withRollPitch(T, rpy(0.1, -0.2, 2.0));
  EXPECT_TRUE(out.linear().isApprox(rpy(0.1, -0.2, 0.7), 1e-12));
  EXPECT_TRUE(out.translation().isApprox(T.translation()));
}

TEST(TfProvider, IdentityImuFlattensATiltedPose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = rpy(0.3, 0.2, -1.0);
  const auto out = withRollPitch(T, Eigen::Matrix3d::Identity());
  EXPECT_TRUE(out.linear().isApprox(rpy(0.0, 0.0, -1.0), 1e-12));
}
