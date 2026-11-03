// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// TraversabilityLayer with cost_model utilisation (SPEC.md cell cost), fed synthetic feature
// maps. Costmap 4 x 4 m at 0.05 m from (0, 0), robot at its centre (2, 2), footprint 0.4 m square
// (inscribed 0.2 m = 4 cells, circumscribed 0.283 m). Robot speed 0 (no odometry received), so
// the braking distance is the 0.25 m margin.

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
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace
{

constexpr char kTopic[] = "/test_traversability_v2/features";
constexpr double kHalf = 0.025;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

class V2 : public ::testing::Test
{
protected:
  // Overrides added by derived fixtures.
  virtual std::vector<rclcpp::Parameter> extraParams() const {return {};}

  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> params{
      {"trav.map_topic", std::string(kTopic)},
      {"trav.cost_model", std::string("utilisation")},
      {"trav.odom_topic", std::string("/test_traversability_v2/odom_unused")},
      {"trav.map_timeout", 5.0},
      {"trav.escape_radius", 0.3},
    };
    for (const auto & p : extraParams()) {params.push_back(p);}
    opts.parameter_overrides(params);
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_costmap_v2", "", opts);
    pub_node_ = std::make_shared<rclcpp::Node>("test_map_publisher_v2");
    pub_ = pub_node_->create_publisher<grid_map_msgs::msg::GridMap>(kTopic, 10);
    tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    lcm_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, true);
    lcm_->resizeMap(80, 80, 0.05, 0.0, 0.0);
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
    layer_ = loader_->createSharedInstance("rover_traversability/TraversabilityLayer");
    lcm_->addPlugin(layer_);
    layer_->initialize(lcm_.get(), "trav", tf_.get(), node_->weak_from_this(), nullptr);
    std::vector<geometry_msgs::msg::Point> fp(4);
    fp[0].x = 0.2; fp[0].y = 0.2;
    fp[1].x = 0.2; fp[1].y = -0.2;
    fp[2].x = -0.2; fp[2].y = -0.2;
    fp[3].x = -0.2; fp[3].y = 0.2;
    lcm_->setFootprint(fp);
  }

  void TearDown() override {lcm_.reset(); layer_.reset();}

  // Every cell safe at every heading with u = 0, unless changed.
  grid_map::GridMap benign() const
  {
    grid_map::GridMap m({layers::kBestUtilisation, layers::kSafeYawMask});
    m.setFrameId("odom");
    m.setGeometry(grid_map::Length(4.0, 4.0), 0.05, grid_map::Position(2.0, 2.0));
    m[layers::kBestUtilisation].setConstant(0.0f);
    m[layers::kSafeYawMask].setConstant(65535.0f);
    return m;
  }

  void set(grid_map::GridMap & m, const char * l, double x, double y, float v) const
  {
    m.atPosition(l, grid_map::Position(x + kHalf, y + kHalf)) = v;
  }

  void feed(grid_map::GridMap & m)
  {
    auto msg = grid_map::GridMapRosConverter::toMessage(m);
    msg->header.stamp = node_->now();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      pub_->publish(*msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      lcm_->updateMap(2.0, 2.0, 0.0);
      if (lcm_->isCurrent()) {return;}
    }
    FAIL() << "layer never went current";
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

// Point footprint in Nav2, as the 3D overlays configure it: utilisation verdicts are already
// configuration space, so lethal cells stay single cells, and the braking band starts at the
// physical footprint (rover_physical.yaml: 1.1 x 0.9 m, circumscribed 0.711 m).
class V2Point : public V2
{
protected:
  std::vector<rclcpp::Parameter> extraParams() const override
  {
    return {{"trav.dilate_terrain_lethal", false},
      {"trav.braking_from_physical_footprint", true}};
  }
};

}  // namespace

TEST_F(V2, GradedCostIsQuadraticInUtilisation)
{
  // u = 0.5 -> floor(252 * 0.25) = 63. u = 1.0 -> 252, still not lethal.
  auto m = benign();
  set(m, layers::kBestUtilisation, 0.5, 0.5, 0.5f);
  set(m, layers::kBestUtilisation, 3.5, 0.5, 1.0f);
  feed(m);
  EXPECT_EQ(cost(0.5, 0.5), 63);
  EXPECT_EQ(cost(3.5, 0.5), 252);
  EXPECT_EQ(cost(1.0, 1.0), 0);
}

TEST_F(V2, GradedCostIsNotDilated)
{
  auto m = benign();
  set(m, layers::kBestUtilisation, 0.5, 0.5, 0.5f);
  feed(m);
  EXPECT_EQ(cost(0.55, 0.5), 0);
}

TEST_F(V2, NoSafeHeadingIsLethalAndDilated)
{
  // Mask 0 at (0.5, 3.5): lethal, grown by the inscribed radius (4 cells) only.
  auto m = benign();
  set(m, layers::kSafeYawMask, 0.5, 3.5, 0.0f);
  feed(m);
  EXPECT_EQ(cost(0.5, 3.5), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0.7, 3.5), LETHAL_OBSTACLE);     // 4 cells
  EXPECT_EQ(cost(0.75, 3.5), 0);                  // 5 cells
  // u over 1 with some heading marked safe still counts as lethal.
  auto m2 = benign();
  set(m2, layers::kBestUtilisation, 3.5, 3.5, 1.2f);
  feed(m2);
  EXPECT_EQ(cost(3.5, 3.5), LETHAL_OBSTACLE);
}

TEST_F(V2, UnknownIsLethalOnlyWithinBrakingDistanceOutsideTheFootprint)
{
  // Robot at (2, 2), circumscribed radius 0.283 m, braking distance 0.25 m (speed 0):
  // unknown cells between 0.283 and 0.533 m from the centre are lethal, beyond that 200,
  // and under the footprint 200 (blind zone exemption). Unknown cost is not dilated.
  auto m = benign();
  for (auto x : {2.0, 2.4, 3.0}) {
    set(m, layers::kBestUtilisation, x, 2.0, kNaN);
  }
  feed(m);
  EXPECT_EQ(cost(2.0, 2.0), 200);                 // under the footprint
  EXPECT_EQ(cost(2.4, 2.0), LETHAL_OBSTACLE);     // 0.4 + half a cell from the centre
  EXPECT_EQ(cost(3.0, 2.0), 200);                 // 1.0 m out
  EXPECT_EQ(cost(3.05, 2.0), 0);                  // the unknown cost did not spread
  // The braking-band lethal cell is not dilated toward the rover: 4 cells (0.2 m) closer, still
  // under the footprint's circumscribed circle, the cell is free.
  EXPECT_EQ(cost(2.2, 2.0), 0);
}

TEST_F(V2Point, TerrainLethalIsNotDilated)
{
  auto m = benign();
  set(m, layers::kSafeYawMask, 0.5, 3.5, 0.0f);
  feed(m);
  EXPECT_EQ(cost(0.5, 3.5), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0.55, 3.5), 0);                  // the next cell
}

TEST_F(V2Point, BrakingBandStartsAtThePhysicalFootprint)
{
  // Robot at (2, 2). Nav2's footprint here is the fixture's 0.4 m square (circumscribed 0.283),
  // smaller than the physical 0.711 m, so the band runs from 0.711 to 0.961 m (speed 0).
  // Cell centres sit half a cell (0.025 m) beyond the x values below.
  auto m = benign();
  for (auto x : {2.6, 2.8, 3.0}) {
    set(m, layers::kBestUtilisation, x, 2.0, kNaN);
  }
  feed(m);
  EXPECT_EQ(cost(2.6, 2.0), 200);                 // 0.625 m, inside the physical footprint
  EXPECT_EQ(cost(2.8, 2.0), LETHAL_OBSTACLE);     // 0.825 m, in the band
  EXPECT_EQ(cost(3.0, 2.0), 200);                 // 1.025 m, beyond it
}

TEST_F(V2, LethalNextToTheRoverIsCappedSoItCanLeave)
{
  // Robot at (2, 2), escape radius 0.3 m. Mask 0 at (2.4, 2.0), centre 0.425 m out: lethal, and
  // dilated by 4 cells to x 2.2..2.6. Dilated cells within 0.3 m (x 2.2, 2.25) drop to 252,
  // the ones beyond (x 2.3, centre 0.325 m) stay lethal, and so does the source cell.
  auto m = benign();
  set(m, layers::kSafeYawMask, 2.4, 2.0, 0.0f);
  feed(m);
  EXPECT_EQ(cost(2.4, 2.0), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(2.3, 2.0), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(2.25, 2.0), 252);
  EXPECT_EQ(cost(2.2, 2.0), 252);
}

TEST_F(V2Point, RoverOnALethalCellCanStillLeave)
{
  // The rover's own cell and its neighbour judged unsafe (u 1.5): 252 so planners can start,
  // a cell 0.5 m away keeps lethal.
  auto m = benign();
  set(m, layers::kBestUtilisation, 2.0, 2.0, 1.5f);
  set(m, layers::kBestUtilisation, 2.1, 2.0, 1.5f);
  set(m, layers::kBestUtilisation, 2.5, 2.0, 1.5f);
  feed(m);
  EXPECT_EQ(cost(2.0, 2.0), 252);
  EXPECT_EQ(cost(2.1, 2.0), 252);
  EXPECT_EQ(cost(2.5, 2.0), LETHAL_OBSTACLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
