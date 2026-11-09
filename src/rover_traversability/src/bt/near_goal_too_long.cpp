// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/near_goal_too_long.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_traversability
{

bool NearGoalTimer::update(double t, double distance, double radius, double timeout)
{
  if (distance > radius) {
    inside_ = false;
    return false;
  }
  if (!inside_) {
    inside_ = true;
    since_ = t;
    return false;
  }
  if (t - since_ < timeout) {return false;}
  since_ = t;   // fire once, then count a fresh dwell
  return true;
}

NearGoalTooLong::NearGoalTooLong(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config) {}

BT::PortsList NearGoalTooLong::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
    BT::InputPort<double>("radius", 1.0, "m from the goal"),
    BT::InputPort<double>("timeout", 25.0, "s inside the radius before it fires"),
    BT::OutputPort<bool>("stuck"),
  };
}

BT::NodeStatus NearGoalTooLong::tick()
{
  geometry_msgs::msg::PoseStamped goal, current;
  if (!getInput("goal", goal) || !getInput("current_pose", current)) {
    return BT::NodeStatus::SUCCESS;   // nothing to judge, let the controller run
  }
  double radius = 1.0, timeout = 25.0;
  getInput("radius", radius);
  getInput("timeout", timeout);
  const auto & g = goal.pose.position;
  if (g.x != goal_x_ || g.y != goal_y_) {
    goal_x_ = g.x;
    goal_y_ = g.y;
    timer_.reset();
  }
  const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  const double t = node->now().seconds();
  const auto & c = current.pose.position;
  if (timer_.update(t, std::hypot(g.x - c.x, g.y - c.y), radius, timeout)) {
    RCLCPP_WARN(node->get_logger(), "NearGoalTooLong: %.0f s within %.1f m of the goal, re-approach",
      timeout, radius);
    setOutput("stuck", true);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_traversability

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rover_traversability::NearGoalTooLong>("NearGoalTooLong");
}
