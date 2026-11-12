// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Path validity check for the 3D tree, in place of Nav2's IsPathValid. Jazzy's planner_server
// rejects a path when any pose centre sits on a LETHAL or an INSCRIBED cell. Smac Hybrid checks
// the real footprint and does put pose centres on INSCRIBED cells beside rocks, so beside a rock
// every new path was "invalid" at once and the tree replanned at its 2 Hz cap, flipping the path
// between the two sides of the rock (rock_field bag 2026-11-11 09:23, 1898 plans in 1230 s). Here a
// path is invalid only when a pose centre is on a LETHAL cell, which Smac never plans through, so a
// path turns invalid only when the map changes under it. Unknown cells pass, as in Nav2.

#ifndef ROVER_TRAVERSABILITY__IS_PATH_LETHAL_FREE_HPP_
#define ROVER_TRAVERSABILITY__IS_PATH_LETHAL_FREE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_traversability
{

// Index of the first pose whose centre lies on a cell of cost lethal, or -1. Poses outside the
// grid are skipped. data is row-major, size_x by size_y, origin is the grid's lower-left corner.
int firstLethalPose(
  const std::vector<uint8_t> & data, unsigned size_x, unsigned size_y, double resolution,
  double origin_x, double origin_y, const std::vector<double> & xs, const std::vector<double> & ys,
  uint8_t lethal = 254);

// BT condition: input path, costmap topic (default global_costmap/costmap_raw). SUCCESS while
// the path is lethal-free or no costmap has arrived yet, FAILURE otherwise.
class IsPathLethalFree : public BT::ConditionNode
{
public:
  IsPathLethalFree(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
  bool warned_frame_{false};
};

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__IS_PATH_LETHAL_FREE_HPP_
