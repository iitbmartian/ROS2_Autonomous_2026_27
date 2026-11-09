// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Stuck-near-goal detector for a car-like rover. A sideways offset at the goal cannot be closed
// without a parking manoeuvre, and MPPI then saws back and forth a few centimetres for minutes
// (marsyard behind_left bag 2026-11-09: 0.27 m to the side, 18 min). Each saw moves more than the
// progress checker's 0.10 m, so the controller never fails and no recovery runs. This fires once
// the rover has stayed within radius of the goal for longer than timeout, so the tree can back up
// and come in again.

#ifndef ROVER_TRAVERSABILITY__NEAR_GOAL_TOO_LONG_HPP_
#define ROVER_TRAVERSABILITY__NEAR_GOAL_TOO_LONG_HPP_

#include <cmath>
#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace rover_traversability
{

// Time spent inside the radius. update() returns true once when the dwell passes timeout, then
// starts over. Leaving the radius, or a new goal (reset), clears the dwell.
class NearGoalTimer
{
public:
  bool update(double t, double distance, double radius, double timeout);
  void reset() {inside_ = false;}

private:
  bool inside_{false};
  double since_{0.0};
};

// BT condition: inputs goal, current_pose, radius (m, 1.0), timeout (s, 25, ROS time so sim time
// in simulation). SUCCESS while the rover is fine, FAILURE when it fires, and then output stuck is
// set true for the recovery branch to read.
class NearGoalTooLong : public BT::ConditionNode
{
public:
  NearGoalTooLong(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  NearGoalTimer timer_;
  double goal_x_{NAN}, goal_y_{NAN};
};

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__NEAR_GOAL_TOO_LONG_HPP_
