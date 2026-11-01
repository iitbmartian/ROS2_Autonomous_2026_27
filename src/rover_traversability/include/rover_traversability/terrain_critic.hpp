// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// TerrainCritic: MPPI critic scoring every rollout on the terrain at its actual yaw (SPEC.md,
// controller cost). The scoring is core::trajectoryCosts. This plugin feeds it MPPI's rollouts
// and the latest /terrain/local_features map, in the controller's (odom) frame.

#ifndef ROVER_TRAVERSABILITY__TERRAIN_CRITIC_HPP_
#define ROVER_TRAVERSABILITY__TERRAIN_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_traversability/core/pose_evaluation.hpp"
#include "rover_traversability/core/trajectory_cost.hpp"

namespace rover_traversability
{

class TerrainCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  void onMap(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  struct Terrain
  {
    core::TerrainGrid grid;
    core::EvalCache cache;
    rclcpp::Time stamp;
  };

  core::PhysicalParams phys_;
  core::EvalParams eval_;
  core::TrajectoryCostParams cost_;
  double map_timeout_{1.0};
  std::string map_topic_;
  std::string frame_;
  std::mutex mutex_;
  std::shared_ptr<const Terrain> terrain_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__TERRAIN_CRITIC_HPP_
