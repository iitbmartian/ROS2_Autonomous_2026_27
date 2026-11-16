// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// HeadingSmoother, a Nav2 smoother plugin. Writes a body heading into every pose of a planned
// path, chosen on the live terrain map by core::assignHeadings: the 2D planner gives positions
// only, and without a heading plan the controller has to find a safe heading on its own, live,
// which on side slopes it often does not in time. Swerve (allow_crab true) may hold any safe
// heading, Ackermann (false) only the travel direction or its reverse. The path is returned with
// the headings even when a pose has no safe heading, and that pose is logged, since the planner
// cost already marks cells with no safe heading as lethal and such a pose means a stale map.

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rover_traversability/core/heading_assignment.hpp"
#include "rover_traversability/core/physical_params.hpp"
#include "rover_traversability/core/pose_evaluation.hpp"
#include "../nodes/terrain_map_input.hpp"
#include "tf2/utils.h"

namespace rover_traversability
{

class HeadingSmoother : public nav2_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override
  {
    auto node = parent.lock();
    name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    auto p = [&](const std::string & k, auto def) {
        nav2_util::declare_parameter_if_not_declared(node, name_ + "." + k, rclcpp::ParameterValue(def));
        return node->get_parameter(name_ + "." + k);
      };
    map_topic_ = p("map_topic", std::string("/terrain/global_features")).as_string();
    map_timeout_ = p("map_timeout", 5.0).as_double();
    hp_.bins = p("yaw_bins", 16).as_int();
    hp_.w_util = p("util_weight", 1.0).as_double();
    hp_.w_crab = p("crab_weight", 0.5).as_double();
    hp_.w_reverse = p("reverse_weight", 1.0).as_double();
    hp_.w_turn = p("turn_weight", 0.3).as_double();
    hp_.allow_crab = p("allow_crab", true).as_bool();
    eval_.risk_kappa = p("risk_kappa", 1.0).as_double();
    const std::string phys_path =
      p("physical_params_file", core::defaultPhysicalParamsPath()).as_string();
    phys_ = core::loadPhysicalParams(phys_path);
    const auto warn = core::placeholderWarning(phys_);
    if (!warn.empty()) {RCLCPP_WARN(logger_, "HeadingSmoother: %s", warn.c_str());}
    // Only the heading-dependent checks. Clearance and roughness do not depend on yaw and the
    // costmap applies them at every cell already.
    eval_.check_clearance = false;
    eval_.check_roughness = false;
    sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic_, rclcpp::QoS(1).reliable(),
      [this](grid_map_msgs::msg::GridMap::SharedPtr m) {
        auto s = snapshotFrom(*m, phys_, eval_, clock_->get_clock_type());
        std::lock_guard<std::mutex> l(mu_);
        if (s) {snap_ = s;}
      });
    RCLCPP_INFO(
      logger_, "HeadingSmoother: %s, %d bins, crab %s (w %.2f), reverse %.2f, turn %.2f",
      map_topic_.c_str(), hp_.bins, hp_.allow_crab ? "on" : "off", hp_.w_crab, hp_.w_reverse,
      hp_.w_turn);
  }

  void cleanup() override {sub_.reset();}
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration &) override
  {
    const size_t n = path.poses.size();
    if (n < 2) {return true;}
    std::shared_ptr<TerrainSnapshot> s;
    {std::lock_guard<std::mutex> l(mu_); s = snap_;}
    // Without a fresh map in the path's frame the path is left as planned, never failed: the
    // heading plan is an aid and the controller still evaluates every rollout on the terrain.
    if (!s || s->frame != path.header.frame_id ||
      (clock_->now() - s->stamp).seconds() > map_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "HeadingSmoother: no fresh map in '%s', path unchanged",
        path.header.frame_id.c_str());
      return true;
    }
    std::vector<double> x(n), y(n);
    for (size_t i = 0; i < n; ++i) {
      x[i] = path.poses[i].pose.position.x; y[i] = path.poses[i].pose.position.y;
    }
    const auto travel = core::travelDirections(x, y);
    std::vector<std::vector<double>> util(n, std::vector<double>(hp_.bins, NAN));
    for (size_t i = 0; i < n; ++i) {
      for (int b = 0; b < hp_.bins; ++b) {
        const double yaw = 2.0 * M_PI * b / hp_.bins;
        const auto r = core::evaluatePose(s->grid, x[i], y[i], yaw, phys_, eval_, &s->cache);
        util[i][b] = r.status == core::PoseStatus::kOk ? r.u :
          r.status == core::PoseStatus::kLethal ? INFINITY : NAN;
      }
    }
    // Unknown cells ahead are normal on a frontier. Treat them as flat and safe at every
    // heading, so they do not pull the heading plan, and let the map decide when it arrives.
    for (auto & row : util) {
      bool any = false;
      for (double v : row) {any |= !std::isnan(v);}
      if (!any) {std::fill(row.begin(), row.end(), 0.0);}
    }
    const auto r = core::assignHeadings(util, travel, hp_);
    for (size_t i = 0; i < n; ++i) {
      auto & q = path.poses[i].pose.orientation;
      q.x = 0.0; q.y = 0.0; q.z = std::sin(0.5 * r.yaw[i]); q.w = std::cos(0.5 * r.yaw[i]);
    }
    if (!r.feasible) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000, "HeadingSmoother: pose %d of %zu has no safe allowed heading",
        r.first_infeasible, n);
    }
    return true;
  }

private:
  std::string name_, map_topic_;
  double map_timeout_{5.0};
  rclcpp::Logger logger_{rclcpp::get_logger("HeadingSmoother")};
  rclcpp::Clock::SharedPtr clock_;
  core::HeadingParams hp_;
  core::EvalParams eval_;
  core::PhysicalParams phys_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  std::mutex mu_;
  std::shared_ptr<TerrainSnapshot> snap_;
};

}  // namespace rover_traversability

PLUGINLIB_EXPORT_CLASS(rover_traversability::HeadingSmoother, nav2_core::Smoother)
