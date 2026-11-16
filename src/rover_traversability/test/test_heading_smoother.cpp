// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// HeadingSmoother end to end: pluginlib load, a 20 deg sidehill map on a topic, a straight path
// across the slope. Swerve must face up or down the slope (crab), Ackermann must keep the
// travel direction or its reverse.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "nav2_core/smoother.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rover_traversability/core/physical_params.hpp"
#include "rover_traversability/core/pose_evaluation.hpp"
#include "rover_traversability/grid_map_msg.hpp"
#include "rover_traversability/layer_names.hpp"
#include "tf2/utils.h"

using rover_traversability::core::TerrainGrid;

namespace
{

constexpr double kSlopeDeg = 20.0;

grid_map_msgs::msg::GridMap sidehill(const rclcpp::Time & stamp)
{
  grid_map_msgs::msg::GridMap m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.info.resolution = 0.05;
  m.info.length_x = 4.05; m.info.length_y = 4.05;
  m.outer_start_index = 0; m.inner_start_index = 0;
  const auto g = rover_traversability::geometryOf(m);
  Eigen::MatrixXf e(g.rows ? g.rows : 81, g.cols ? g.cols : 81);
  for (int i = 0; i < e.rows(); ++i) {
    for (int j = 0; j < e.cols(); ++j) {
      e(i, j) = static_cast<float>(std::tan(kSlopeDeg * M_PI / 180.0) * g.y(j));
    }
  }
  rover_traversability::writeLayer(m, rover_traversability::layers::kElevation, e);
  return m;
}

nav_msgs::msg::Path straightX()
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  for (int k = -5; k <= 5; ++k) {
    geometry_msgs::msg::PoseStamped s;
    s.header.frame_id = "map";
    s.pose.position.x = 0.1 * k;
    s.pose.orientation.w = 1.0;
    p.poses.push_back(s);
  }
  return p;
}

nav_msgs::msg::Path runSmoother(bool allow_crab)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides(
    {{"heading.map_topic", "/test_heading_map"}, {"heading.allow_crab", allow_crab}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("heading_test", o);
  pluginlib::ClassLoader<nav2_core::Smoother> loader("nav2_core", "nav2_core::Smoother");
  auto sm = loader.createSharedInstance("rover_traversability/HeadingSmoother");
  sm->configure(node, "heading", nullptr, nullptr, nullptr);
  sm->activate();
  // A plain node publishes: a lifecycle publisher drops messages until activated.
  auto src = std::make_shared<rclcpp::Node>("heading_test_map");
  auto pub = src->create_publisher<grid_map_msgs::msg::GridMap>(
    "/test_heading_map", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node->get_node_base_interface());
  ex.add_node(src);
  nav_msgs::msg::Path path;
  for (int t = 0; t < 40; ++t) {
    pub->publish(sidehill(node->now()));
    ex.spin_some(std::chrono::milliseconds(50));
    path = straightX();
    sm->smooth(path, rclcpp::Duration::from_seconds(1.0));
    if (std::abs(tf2::getYaw(path.poses[5].pose.orientation)) > 1e-6 || !allow_crab) {
      // For crabbing a changed heading shows the map arrived. Without crabbing the heading
      // may legitimately stay 0, so give the map a few cycles.
      if (allow_crab || t >= 10) {break;}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  sm->deactivate();
  sm->cleanup();
  return path;
}

}  // namespace

class HeadingSmootherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

// Precondition from the real physical parameter file: across the slope is unsafe, up or down
// the slope is safe. Without it the smoother tests below prove nothing.
TEST_F(HeadingSmootherTest, PreconditionSidehillHeadings)
{
  const auto p = rover_traversability::core::loadPhysicalParams(
    rover_traversability::core::defaultPhysicalParamsPath());
  TerrainGrid g;
  g.resolution = 0.05;
  g.elevation.resize(81, 81);
  for (int i = 0; i < 81; ++i) {
    for (int j = 0; j < 81; ++j) {
      g.elevation(i, j) = static_cast<float>(std::tan(kSlopeDeg * M_PI / 180.0) * g.y(j));
    }
  }
  rover_traversability::core::EvalParams e;
  e.check_clearance = false; e.check_roughness = false;
  using rover_traversability::core::PoseStatus;
  EXPECT_EQ(rover_traversability::core::evaluatePose(g, 0, 0, 0.0, p, e).status, PoseStatus::kLethal);
  EXPECT_EQ(
    rover_traversability::core::evaluatePose(g, 0, 0, M_PI / 2, p, e).status, PoseStatus::kOk);
}

TEST_F(HeadingSmootherTest, SwerveFacesAcrossTheTravelOnASidehill)
{
  const auto path = runSmoother(true);
  for (const auto & s : path.poses) {
    EXPECT_NEAR(std::abs(std::sin(tf2::getYaw(s.pose.orientation))), 1.0, 1e-6);
  }
}

TEST_F(HeadingSmootherTest, AckermannKeepsTravelOrReverse)
{
  const auto path = runSmoother(false);
  for (const auto & s : path.poses) {
    EXPECT_NEAR(std::abs(std::sin(tf2::getYaw(s.pose.orientation))), 0.0, 1e-6);
  }
}
