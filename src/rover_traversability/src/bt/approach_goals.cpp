// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/approach_goals.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"

namespace rover_traversability
{

std::vector<geometry_msgs::msg::PoseStamped> approachGoals(
  const geometry_msgs::msg::PoseStamped & goal, const geometry_msgs::msg::PoseStamped & current,
  double approach_distance, double min_distance)
{
  const auto & g = goal.pose.position;
  const auto & c = current.pose.position;
  if (approach_distance <= 0.0 || std::hypot(g.x - c.x, g.y - c.y) <= min_distance) {
    return {goal};
  }
  const auto & q = goal.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  geometry_msgs::msg::PoseStamped pre = goal;
  pre.pose.position.x = g.x - approach_distance * std::cos(yaw);
  pre.pose.position.y = g.y - approach_distance * std::sin(yaw);
  return {pre, goal};
}

ApproachGoals::ApproachGoals(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config) {}

BT::PortsList ApproachGoals::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
    BT::InputPort<double>("approach_distance", 1.5, "m behind the goal along its heading"),
    BT::InputPort<double>("min_distance", 3.0, "m, nearer than this the goal is used alone"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
  };
}

BT::NodeStatus ApproachGoals::tick()
{
  geometry_msgs::msg::PoseStamped goal, current;
  if (!getInput("goal", goal) || !getInput("current_pose", current)) {
    return BT::NodeStatus::FAILURE;
  }
  double d = 1.5, m = 3.0;
  getInput("approach_distance", d);
  getInput("min_distance", m);
  setOutput("goals", approachGoals(goal, current, d, m));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_traversability

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rover_traversability::ApproachGoals>("ApproachGoals");
}
