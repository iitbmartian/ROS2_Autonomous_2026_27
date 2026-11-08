// Copyright 2026 Mars Rover Team, IIT Bombay.
// Hand cases for the approach-from-behind pre-goal.

#include <gtest/gtest.h>

#include <cmath>

#include "rover_traversability/approach_goals.hpp"

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
}  // namespace

TEST(ApproachGoals, FarGoalGetsAPreGoalBehindItAlongItsHeading)
{
  // far_south: goal (30, -20) at yaw -0.6, rover at the start. 1.5 m behind:
  // (30 - 1.5 cos(-0.6), -20 - 1.5 sin(-0.6)) = (28.762, -19.153).
  const auto g = rover_traversability::approachGoals(pose(30, -20, -0.6), pose(0, 0, 0), 1.5, 3.0);
  ASSERT_EQ(g.size(), 2u);
  EXPECT_NEAR(g[0].pose.position.x, 28.762, 1e-3);
  EXPECT_NEAR(g[0].pose.position.y, -19.153, 1e-3);
  EXPECT_DOUBLE_EQ(g[0].pose.orientation.z, g[1].pose.orientation.z);
  EXPECT_EQ(g[0].header.frame_id, "map");
  EXPECT_DOUBLE_EQ(g[1].pose.position.x, 30.0);
}

TEST(ApproachGoals, NearGoalIsUsedAlone)
{
  // 2.9 m away with min_distance 3: no pre-goal, so a rover already close does not loop back.
  const auto g = rover_traversability::approachGoals(pose(10, 0, 0), pose(7.1, 0, 0), 1.5, 3.0);
  ASSERT_EQ(g.size(), 1u);
  EXPECT_DOUBLE_EQ(g[0].pose.position.x, 10.0);
}

TEST(ApproachGoals, ZeroDistanceDisables)
{
  const auto g = rover_traversability::approachGoals(pose(10, 0, 1.0), pose(0, 0, 0), 0.0, 3.0);
  ASSERT_EQ(g.size(), 1u);
}

TEST(ApproachGoals, BehindMeansAgainstTheHeading)
{
  // Goal facing +y at (0, 5): the pre-goal is at (0, 3.5), below it.
  const auto g = rover_traversability::approachGoals(pose(0, 5, M_PI / 2), pose(0, -10, 0), 1.5, 3.0);
  ASSERT_EQ(g.size(), 2u);
  EXPECT_NEAR(g[0].pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(g[0].pose.position.y, 3.5, 1e-9);
}
