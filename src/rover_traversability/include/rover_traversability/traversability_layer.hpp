// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Nav2 costmap plugin that converts a robot centric 2.5D elevation map
// (grid_map_msgs/GridMap) into traversability costs for the Nav2 planners.
//
// Design note: this layer does NOT compute terrain geometry. Slope, roughness
// and step height are expected to already exist as layers in the incoming
// GridMap (the feature map contract in SPEC.md, layer names in layer_names.hpp).
// This layer only thresholds, blends and resamples. Keep it cheap, it runs on
// the costmap update thread.
//
// Ported from the prototype's mrt_traversability_layer. Phase 3 replaces the
// cost blend with the utilisation model of SPEC.md.

#ifndef ROVER_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_
#define ROVER_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <atomic>
#include "nav2_util/node_thread.hpp"
#include "rover_traversability/core/disc_dilation.hpp"

namespace rover_traversability
{

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer() = default;
  ~TraversabilityLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  void matchSize() override;

  // A 4-wheel independent steering rover can change its swept footprint as the
  // wheels rotate, which changes the inscribed radius the dilation depends on.
  void onFootprintChanged() override;
  void activate() override;
  void deactivate() override;

  // Terrain data must never be wiped by a clearing recovery behaviour.
  // Clearing it would make the rover blind to a cliff it just backed away from.
  bool isClearable() override {return false;}

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  // Blend the three hazard channels into a single 0 to 254 cost (cost_model legacy).
  unsigned char computeCost(float slope_rad, float roughness_m, float step_m) const;

  // Cost from the per-cell heading evaluation (cost_model utilisation, SPEC.md cell cost):
  // lethal when no heading is safe, max_soft_cost * u^2 otherwise. Unknown cells are lethal
  // within braking distance of the footprint edge and unknown_cost beyond it.
  // braking is set when the cell is lethal because of the braking rule.
  unsigned char utilisationCost(float u, float mask, double dist_to_robot, bool & braking) const;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Widen lethal and high cost cells by the inscribed radius so that the
  // planner does not clip a hazard with the edge of the footprint. The Nav2
  // InflationLayer only propagates from cells that equal LETHAL_OBSTACLE, so
  // graded costs need their own dilation. The disc dilation lives in
  // traversability_core, see core/disc_dilation.hpp.
  core::DiscDilation dilation_;
  std::vector<unsigned char> lethal_buf_;      // utilisation mode: lethal cells only, dilated
  // Cells made lethal by the braking rule. They are defined relative to the rover's own outline
  // already, so dilating them again by the inscribed radius would push lethal cost into the
  // rover itself (seen in sim: the planner's start cell became occupied).
  std::vector<unsigned char> braking_mask_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::atomic<double> speed_{0.0};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;

  std::shared_ptr<grid_map::GridMap> latest_map_;
  rclcpp::Time latest_map_stamp_;
  std::mutex map_mutex_;

  // Bounds touched on the previous cycle, so that stale terrain gets cleared
  // from the master grid when the elevation map moves with the rover.
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};
  bool has_last_bounds_{false};

  // Parameters
  std::string cost_model_{"legacy"};           // legacy (prototype blend) or utilisation
  std::string utilisation_layer_, mask_layer_, odom_topic_;
  double braking_decel_{0.5};                  // m/s^2
  double braking_margin_{0.25};                // m beyond the stopping distance
  bool unknown_lethal_in_braking_{true};       // off for the global costmap: a local safety rule
  // Utilisation verdicts already place the whole rover at the cell, so with a point footprint
  // in Nav2 the lethal cells must not grow again.
  bool dilate_terrain_lethal_{true};
  // The braking band starts at the larger of Nav2's circumscribed radius and the physical
  // footprint's (rover_physical.yaml), so a point footprint in Nav2 keeps the rule's meaning.
  bool braking_from_physical_footprint_{false};
  double physical_circumscribed_{0.0};
  std::string map_topic_;
  std::string slope_layer_, roughness_layer_, step_layer_;
  bool use_precomputed_traversability_{false};
  std::string traversability_layer_;

  double max_slope_{0.45};        // rad, hard limit, about 26 deg
  double max_roughness_{0.05};    // m, RMS residual to the fitted plane
  double max_step_{0.15};         // m, wheel scale peak to peak

  double slope_weight_{0.5};
  double roughness_weight_{0.2};
  double step_weight_{0.3};

  unsigned char unknown_cost_{200};
  unsigned char max_soft_cost_{252};
  double lethal_dilation_radius_{0.0};   // set from footprint if left at 0
  bool use_explicit_dilation_radius_{false};
  // s. A map older than this makes the layer not current, with no opt-out: a
  // terrain layer without fresh data must stop the controller, not pass stale costs.
  double map_timeout_{1.0};
  int combination_method_{1};            // 0 overwrite, 1 max
};

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_
