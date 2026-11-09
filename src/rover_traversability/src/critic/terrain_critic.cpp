// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/terrain_critic.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "rover_traversability/core/physical_params.hpp"
#include "rover_traversability/core/yaw_evaluation.hpp"
#include "rover_traversability/grid_map_msg.hpp"
#include "rover_traversability/layer_names.hpp"

namespace rover_traversability
{

void TerrainCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(map_topic_, "map_topic", std::string("/terrain/local_features"));
  getParam(map_timeout_, "map_timeout", 2.0);
  getParam(eval_.risk_kappa, "risk_kappa", 1.0);
  getParam(cost_.w_terrain, "cost_weight", 20.0);
  getParam(cost_.collision_cost, "collision_cost", 1.0e6);
  getParam(cost_.unknown_cost, "unknown_cost", 1.0);
  getParam(cost_.u_free, "u_free", 0.0);
  getParam(cost_.u_tip_lethal, "u_tip_lethal", 1.0);
  getParam(cost_.w_redirect, "redirect_weight", 2.0);
  getParam(cost_.w_uphill, "uphill_weight", 5.0);
  getParam(cost_.point_step, "trajectory_point_step", 3);
  getParam(cost_.threads, "threads", 4);
  std::string phys_path;
  getParam(phys_path, "physical_params_file", core::defaultPhysicalParamsPath());
  // The costmap applies the yaw-independent checks at every cell already.
  eval_.check_clearance = false;
  eval_.check_roughness = false;

  phys_ = core::loadPhysicalParams(phys_path);
  const auto warn = core::placeholderWarning(phys_);
  if (!warn.empty()) {RCLCPP_WARN(logger_, "TerrainCritic: %s", warn.c_str());}

  auto node = parent_.lock();
  clock_ = node->get_clock();
  frame_ = costmap_ros_->getGlobalFrameID();
  sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    // Reliable, newest only: a 2 MB map over best effort loses whole maps to one lost fragment.
    map_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&TerrainCritic::onMap, this, std::placeholders::_1));
  RCLCPP_INFO(
    logger_, "TerrainCritic: %s, weight %.1f, redirect %.1f, uphill %.1f, every %d points",
    map_topic_.c_str(), cost_.w_terrain, cost_.w_redirect, cost_.w_uphill, cost_.point_step);
}

void TerrainCritic::onMap(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  if (msg->header.frame_id != frame_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "TerrainCritic: map in '%s', controller in '%s', ignored",
      msg->header.frame_id.c_str(), frame_.c_str());
    return;
  }
  auto t = std::make_shared<Terrain>();
  const auto geo = geometryOf(*msg);
  t->grid.resolution = geo.resolution;
  t->grid.center_x = geo.center_x;
  t->grid.center_y = geo.center_y;
  if (!readLayer(*msg, layers::kElevation, t->grid.elevation)) {return;}
  readLayer(*msg, layers::kVariance, t->grid.variance);
  readLayer(*msg, layers::kStep, t->grid.step);
  readLayer(*msg, layers::kNegativeObstacle, t->grid.negative_obstacle);
  t->cache = core::makeEvalCache(t->grid, phys_, eval_);
  t->stamp = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
  std::lock_guard<std::mutex> lock(mutex_);
  terrain_ = t;
}

void TerrainCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {return;}
  std::shared_ptr<const Terrain> t;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    t = terrain_;
  }
  // No terrain or stale terrain: the costmap layer already stops the controller then.
  if (!t || (clock_->now() - t->stamp).seconds() > map_timeout_) {return;}

  const auto & tr = data.trajectories;
  const int batch = static_cast<int>(tr.x.shape(0)), steps = static_cast<int>(tr.x.shape(1));
  using Map = Eigen::Map<const core::Rollouts>;
  // MPPI's xtensors are row-major batch x steps, so they map without copying.
  const Map x(tr.x.data(), batch, steps), y(tr.y.data(), batch, steps);
  const Map yaw(tr.yaws.data(), batch, steps);
  const Map vx(data.state.vx.data(), batch, steps), vy(data.state.vy.data(), batch, steps);
  const Eigen::VectorXf c = core::trajectoryCosts(
    t->grid, phys_, eval_, cost_, x, y, yaw, vx, vy, data.model_dt, &t->cache);
  for (int b = 0; b < batch; ++b) {data.costs(b) += c(b);}
}

}  // namespace rover_traversability

PLUGINLIB_EXPORT_CLASS(rover_traversability::TerrainCritic, mppi::critics::CriticFunction)
