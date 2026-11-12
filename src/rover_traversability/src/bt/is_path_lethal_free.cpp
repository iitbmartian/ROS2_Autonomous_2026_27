// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/is_path_lethal_free.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"

namespace rover_traversability
{

int firstLethalPose(
  const std::vector<uint8_t> & data, unsigned size_x, unsigned size_y, double resolution,
  double origin_x, double origin_y, const std::vector<double> & xs, const std::vector<double> & ys,
  uint8_t lethal)
{
  if (resolution <= 0.0 || data.size() < static_cast<size_t>(size_x) * size_y) {return -1;}
  for (size_t i = 0; i < xs.size() && i < ys.size(); ++i) {
    const double cx = std::floor((xs[i] - origin_x) / resolution);
    const double cy = std::floor((ys[i] - origin_y) / resolution);
    if (cx < 0 || cy < 0 || cx >= size_x || cy >= size_y) {continue;}
    if (data[static_cast<size_t>(cy) * size_x + static_cast<size_t>(cx)] == lethal) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

IsPathLethalFree::IsPathLethalFree(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  std::string topic = "global_costmap/costmap_raw";
  getInput("costmap_topic", topic);
  // Own callback group and executor, spun in tick(), as Nav2's own BT condition nodes do.
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group_;
  sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    topic, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav2_msgs::msg::Costmap::SharedPtr m) {costmap_ = m;}, opts);
}

BT::PortsList IsPathLethalFree::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw", "nav2_msgs/Costmap"),
  };
}

BT::NodeStatus IsPathLethalFree::tick()
{
  executor_.spin_some();
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {return BT::NodeStatus::FAILURE;}
  if (!costmap_) {return BT::NodeStatus::SUCCESS;}   // no map yet, keep the path
  if (path.header.frame_id != costmap_->header.frame_id) {
    if (!warned_frame_) {
      RCLCPP_WARN(node_->get_logger(), "IsPathLethalFree: path in '%s', costmap in '%s', not checked",
        path.header.frame_id.c_str(), costmap_->header.frame_id.c_str());
      warned_frame_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }
  std::vector<double> xs, ys;
  xs.reserve(path.poses.size());
  ys.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    xs.push_back(p.pose.position.x);
    ys.push_back(p.pose.position.y);
  }
  const auto & md = costmap_->metadata;
  const int i = firstLethalPose(costmap_->data, md.size_x, md.size_y, md.resolution,
      md.origin.position.x, md.origin.position.y, xs, ys);
  if (i < 0) {return BT::NodeStatus::SUCCESS;}
  RCLCPP_INFO(node_->get_logger(), "IsPathLethalFree: pose %d of %zu (%.2f, %.2f) is lethal, replan",
    i, xs.size(), xs[i], ys[i]);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_traversability

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rover_traversability::IsPathLethalFree>("IsPathLethalFree");
}
