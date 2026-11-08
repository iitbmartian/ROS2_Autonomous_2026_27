// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Approach from behind, for a car-like rover. Plans through a pre-goal pose approach_distance
// behind the goal along its heading, so the last stretch is straight and the rover arrives lined
// up. A car cannot fix a sideways offset at the goal without a parking manoeuvre, which MPPI's
// short horizon does not find (far_south, 2026-11-07 bag).

#ifndef ROVER_TRAVERSABILITY__APPROACH_GOALS_HPP_
#define ROVER_TRAVERSABILITY__APPROACH_GOALS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rover_traversability
{

// [pre-goal, goal] while the rover is farther than min_distance from the goal, else [goal].
// The pre-goal keeps the goal's frame, stamp and orientation.
std::vector<geometry_msgs::msg::PoseStamped> approachGoals(
  const geometry_msgs::msg::PoseStamped & goal, const geometry_msgs::msg::PoseStamped & current,
  double approach_distance, double min_distance);

// BT node: inputs goal, current_pose, approach_distance (m, 1.5), min_distance (m, 3.0),
// output goals for ComputePathThroughPoses.
class ApproachGoals : public BT::SyncActionNode
{
public:
  ApproachGoals(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__APPROACH_GOALS_HPP_
