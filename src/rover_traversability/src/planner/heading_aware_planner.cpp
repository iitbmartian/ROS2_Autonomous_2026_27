// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// HeadingAwarePlanner, a Nav2 global planner plugin: core::headingAwareSearch on the global
// costmap, with the safe-heading mask of the terrain feature map. The costmap gives each cell the
// cost of its best heading, so Smac Hybrid-A* plans car paths that reach a ramp or a sidehill
// turning, at a heading that would tip the rover. This planner only lets a state hold a body
// heading the terrain allows at its cell. Without a fresh map in the costmap's frame it plans on
// the costmap alone, like a plain Hybrid-A*.

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rover_traversability/core/heading_aware_search.hpp"
#include "rover_traversability/grid_map_msg.hpp"
#include "rover_traversability/layer_names.hpp"
#include "tf2/utils.h"

namespace rover_traversability
{

class HeadingAwarePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  override
  {
    auto node = parent.lock();
    name_ = name;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    auto p = [&](const std::string & k, auto def) {
        nav2_util::declare_parameter_if_not_declared(
          node, name_ + "." + k, rclcpp::ParameterValue(def));
        return node->get_parameter(name_ + "." + k);
      };
    map_topic_ = p("map_topic", std::string("/terrain/global_features")).as_string();
    map_timeout_ = p("map_timeout", 5.0).as_double();
    sp_.bins = p("yaw_bins", 16).as_int();
    sp_.turning_radius = p("minimum_turning_radius", 0.8).as_double();
    sp_.allow_reverse = p("allow_reverse", true).as_bool();
    sp_.reverse_penalty = p("reverse_penalty", 2.0).as_double();
    sp_.non_straight_penalty = p("non_straight_penalty", 1.2).as_double();
    sp_.cost_penalty = p("cost_penalty", 2.0).as_double();
    sp_.allow_unknown = p("allow_unknown", true).as_bool();
    sp_.unknown_cost = p("unknown_cost", 200.0).as_double();
    sp_.tolerance = p("tolerance", 0.5).as_double();
    sp_.heuristic_weight = p("heuristic_weight", 1.0).as_double();
    sp_.max_iterations = p("max_iterations", 500000).as_int();
    sp_.max_time = p("max_planning_time", 2.0).as_double();
    use_mask_ = p("use_heading_mask", true).as_bool();
    sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic_, rclcpp::QoS(1).reliable(),
      [this](grid_map_msgs::msg::GridMap::SharedPtr m) {
        auto s = std::make_shared<MaskMap>();
        if (!readLayer(*m, layers::kSafeYawMask, s->mask)) {return;}
        s->geo = geometryOf(*m);
        s->frame = m->header.frame_id;
        s->stamp = rclcpp::Time(m->header.stamp, clock_->get_clock_type());
        std::lock_guard<std::mutex> l(mu_);
        map_ = s;
      });
    RCLCPP_INFO(
      logger_, "HeadingAwarePlanner: %s, %d bins, radius %.2f m, reverse %s, mask %s",
      map_topic_.c_str(), sp_.bins, sp_.turning_radius, sp_.allow_reverse ? "on" : "off",
      use_mask_ ? "on" : "off");
  }

  void cleanup() override {sub_.reset();}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    core::SearchGrid g;
    {
      auto * cm = costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(cm->getMutex()));
      g.nx = static_cast<int>(cm->getSizeInCellsX());
      g.ny = static_cast<int>(cm->getSizeInCellsY());
      g.resolution = cm->getResolution();
      g.origin_x = cm->getOriginX();
      g.origin_y = cm->getOriginY();
      g.cost.assign(cm->getCharMap(), cm->getCharMap() + g.nx * g.ny);
    }
    const std::string frame = costmap_ros_->getGlobalFrameID();
    fillMask(g, frame);
    if (cancel_checker && cancel_checker()) {throw nav2_core::PlannerCancelled("cancelled");}

    const auto r = core::headingAwareSearch(
      g, sp_, {start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)},
      goal.pose.position.x, goal.pose.position.y);
    if (r.timed_out) {
      throw nav2_core::PlannerTimedOut("HeadingAwarePlanner: no path within the time limit");
    }
    if (!r.found) {
      throw nav2_core::NoValidPathCouldBeFound(
              "HeadingAwarePlanner: no path with a safe heading at every pose");
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = clock_->now();
    for (const auto & s : r.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = s.x;
      ps.pose.position.y = s.y;
      ps.pose.orientation.z = std::sin(0.5 * s.yaw);
      ps.pose.orientation.w = std::cos(0.5 * s.yaw);
      path.poses.push_back(ps);
    }
    return path;
  }

private:
  struct MaskMap
  {
    Eigen::MatrixXf mask;
    GridGeometry geo;
    std::string frame;
    rclcpp::Time stamp;
  };

  // The map's mask sampled at each costmap cell centre. Cells it does not cover, or covers
  // with unknown, allow every heading and keep only their costmap cost.
  void fillMask(core::SearchGrid & g, const std::string & frame)
  {
    if (!use_mask_) {return;}
    std::shared_ptr<MaskMap> m;
    {std::lock_guard<std::mutex> l(mu_); m = map_;}
    if (!m || m->frame != frame || (clock_->now() - m->stamp).seconds() > map_timeout_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "HeadingAwarePlanner: no fresh mask in '%s', heading-blind plan",
        frame.c_str());
      return;
    }
    g.mask.assign(g.cost.size(), 0xFFFF);
    for (int iy = 0; iy < g.ny; ++iy) {
      const double y = g.origin_y + (iy + 0.5) * g.resolution;
      const int c = static_cast<int>(std::lround(m->geo.col(y)));
      if (c < 0 || c >= m->mask.cols()) {continue;}
      for (int ix = 0; ix < g.nx; ++ix) {
        const double x = g.origin_x + (ix + 0.5) * g.resolution;
        const int r = static_cast<int>(std::lround(m->geo.row(x)));
        if (r < 0 || r >= m->mask.rows()) {continue;}
        const float v = m->mask(r, c);
        if (std::isfinite(v)) {g.mask[iy * g.nx + ix] = static_cast<uint16_t>(v);}
      }
    }
  }

  std::string name_, map_topic_;
  double map_timeout_{5.0};
  bool use_mask_{true};
  core::SearchParams sp_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("HeadingAwarePlanner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  std::mutex mu_;
  std::shared_ptr<MaskMap> map_;
};

}  // namespace rover_traversability

PLUGINLIB_EXPORT_CLASS(rover_traversability::HeadingAwarePlanner, nav2_core::GlobalPlanner)
