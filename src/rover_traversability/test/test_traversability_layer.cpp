// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// TraversabilityLayer loaded through pluginlib into a LayeredCostmap and fed synthetic GridMaps.
// Expected costs are computed by hand from the prototype's cost blend, which Phase 0 keeps:
//   cost = floor(252 * (0.5 s^2 + 0.2 r^2 + 0.3 h^2) / 1.0), s = slope / 0.45, r = rough / 0.05,
//   h = step / 0.15, lethal (254) when any channel is over its limit, 200 when any is NaN.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "rover_traversability/layer_names.hpp"

namespace layers = rover_traversability::layers;

namespace
{

constexpr char kTopic[] = "/test_traversability/features";
// Coordinates in the tests name cell corners. Half a cell moves them to the cell centre, where
// grid map and costmap agree on the cell.
constexpr double kHalf = 0.025;

class LayerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"trav.map_topic", std::string(kTopic)},
        {"trav.map_timeout", 1.0},
      });
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_costmap", "", opts);
    pub_node_ = std::make_shared<rclcpp::Node>("test_map_publisher");
    pub_ = pub_node_->create_publisher<grid_map_msgs::msg::GridMap>(kTopic, 10);
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

    // 3 x 3 m costmap at 0.05 m with its origin at (0, 0), in odom.
    lcm_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, true);
    lcm_->resizeMap(60, 60, 0.05, 0.0, 0.0);

    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
    layer_ = loader_->createSharedInstance("rover_traversability/TraversabilityLayer");
    lcm_->addPlugin(layer_);
    layer_->initialize(lcm_.get(), "trav", tf_.get(), node_->weak_from_this(), nullptr);

    // 0.4 m square: inscribed radius 0.2 m, so the dilation is 4 cells.
    std::vector<geometry_msgs::msg::Point> fp(4);
    fp[0].x = 0.2; fp[0].y = 0.2;
    fp[1].x = 0.2; fp[1].y = -0.2;
    fp[2].x = -0.2; fp[2].y = -0.2;
    fp[3].x = -0.2; fp[3].y = 0.2;
    lcm_->setFootprint(fp);
  }

  void TearDown() override
  {
    lcm_.reset();
    layer_.reset();
  }

  // Flat 3 x 3 m feature map covering the costmap, cells aligned with costmap cells.
  grid_map::GridMap flatMap() const
  {
    grid_map::GridMap m({layers::kSlope, layers::kRoughness, layers::kStep});
    m.setFrameId("odom");
    m.setGeometry(grid_map::Length(3.0, 3.0), 0.05, grid_map::Position(1.5, 1.5));
    for (const auto & l : m.getLayers()) {
      m[l].setConstant(0.0f);
    }
    return m;
  }

  void set(grid_map::GridMap & m, const char * layer, double x, double y, float v) const
  {
    m.atPosition(layer, grid_map::Position(x + kHalf, y + kHalf)) = v;
  }

  // Publishes until the layer has taken the map and the costmap went current, or the deadline.
  bool feed(grid_map::GridMap & m, const rclcpp::Time & stamp, bool expect_current)
  {
    auto msg = grid_map::GridMapRosConverter::toMessage(m);
    msg->header.stamp = stamp;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      pub_->publish(*msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      lcm_->updateMap(1.5, 1.5, 0.0);
      if (lcm_->isCurrent() == expect_current) {
        return true;
      }
    }
    return lcm_->isCurrent() == expect_current;
  }

  unsigned char cost(double x, double y) const
  {
    unsigned int mx, my;
    auto * c = lcm_->getCostmap();
    EXPECT_TRUE(c->worldToMap(x + kHalf, y + kHalf, mx, my));
    return c->getCost(mx, my);
  }

  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> lcm_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_costmap_2d::Layer>> loader_;
  std::shared_ptr<nav2_costmap_2d::Layer> layer_;
};

}  // namespace

TEST_F(LayerFixture, LoadsThroughPluginlib)
{
  ASSERT_NE(layer_, nullptr);
  EXPECT_EQ(layer_->getName(), "trav");
  // Clearing recoveries must never wipe terrain.
  EXPECT_FALSE(layer_->isClearable());
}

TEST_F(LayerFixture, NotCurrentWithoutData)
{
  for (int k = 0; k < 5; ++k) {
    lcm_->updateMap(1.5, 1.5, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_FALSE(lcm_->isCurrent());
}

TEST_F(LayerFixture, HandComputedCosts)
{
  auto m = flatMap();
  set(m, layers::kSlope, 0.5, 2.5, 0.2f);   // ramp, far from everything else
  set(m, layers::kStep, 1.5, 1.5, 0.3f);    // step over the 0.15 m limit
  const float nan = std::numeric_limits<float>::quiet_NaN();
  set(m, layers::kSlope, 2.5, 0.5, nan);    // unknown
  set(m, layers::kRoughness, 2.5, 0.5, nan);
  set(m, layers::kStep, 2.5, 0.5, nan);
  ASSERT_TRUE(feed(m, node_->now(), true));

  using nav2_costmap_2d::LETHAL_OBSTACLE;
  // floor(252 * 0.5 * (0.2 / 0.45)^2) = floor(24.89) = 24.
  EXPECT_EQ(cost(0.5, 2.5), 24);
  EXPECT_EQ(cost(0.5 + 0.20, 2.5), 24);    // dilated 4 cells
  EXPECT_EQ(cost(0.5 + 0.25, 2.5), 0);     // 5 cells, outside
  EXPECT_EQ(cost(1.5, 1.5), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(1.5 + 0.20, 1.5), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(1.5, 1.5 - 0.20), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(1.5 + 0.15, 1.5 + 0.10), LETHAL_OBSTACLE);   // (3,2): 9 + 4 = 13 <= 16
  EXPECT_EQ(cost(1.5 + 0.15, 1.5 + 0.15), 0);                 // (3,3): 18 > 16
  EXPECT_EQ(cost(1.5 + 0.25, 1.5), 0);
  EXPECT_EQ(cost(2.5, 0.5), 200);
  EXPECT_EQ(cost(0.5, 0.5), 0);            // flat
}

TEST_F(LayerFixture, StaleMapIsNotCurrent)
{
  auto m = flatMap();
  ASSERT_TRUE(feed(m, node_->now(), true));
  // Same map, stamped 2 s ago with a 1 s timeout: the layer must drop to not current.
  EXPECT_TRUE(feed(m, node_->now() - rclcpp::Duration::from_seconds(2.0), false));
}

TEST_F(LayerFixture, GoesStaleWhenDataStops)
{
  auto m = flatMap();
  ASSERT_TRUE(feed(m, node_->now(), true));
  std::this_thread::sleep_for(std::chrono::milliseconds(1300));
  lcm_->updateMap(1.5, 1.5, 0.0);
  EXPECT_FALSE(lcm_->isCurrent());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
