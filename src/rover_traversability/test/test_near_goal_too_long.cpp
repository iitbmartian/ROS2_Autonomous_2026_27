// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include "rover_traversability/near_goal_too_long.hpp"

using rover_traversability::NearGoalTimer;

TEST(NearGoalTimer, FiresAfterTheDwellInsideTheRadius)
{
  NearGoalTimer t;
  EXPECT_FALSE(t.update(0.0, 0.3, 1.0, 25.0));    // enters at t 0
  EXPECT_FALSE(t.update(24.9, 0.3, 1.0, 25.0));
  EXPECT_TRUE(t.update(25.0, 0.3, 1.0, 25.0));
}

TEST(NearGoalTimer, FiresOnceThenCountsAFreshDwell)
{
  NearGoalTimer t;
  t.update(0.0, 0.3, 1.0, 25.0);
  EXPECT_TRUE(t.update(25.0, 0.3, 1.0, 25.0));
  EXPECT_FALSE(t.update(26.0, 0.3, 1.0, 25.0));
  EXPECT_FALSE(t.update(49.9, 0.3, 1.0, 25.0));
  EXPECT_TRUE(t.update(50.0, 0.3, 1.0, 25.0));
}

TEST(NearGoalTimer, LeavingTheRadiusClearsTheDwell)
{
  NearGoalTimer t;
  t.update(0.0, 0.3, 1.0, 25.0);
  EXPECT_FALSE(t.update(20.0, 1.5, 1.0, 25.0));   // backed out
  EXPECT_FALSE(t.update(21.0, 0.8, 1.0, 25.0));   // re-enters at 21
  EXPECT_FALSE(t.update(45.0, 0.8, 1.0, 25.0));
  EXPECT_TRUE(t.update(46.0, 0.8, 1.0, 25.0));
}

TEST(NearGoalTimer, ResetForANewGoal)
{
  NearGoalTimer t;
  t.update(0.0, 0.3, 1.0, 25.0);
  t.reset();
  EXPECT_FALSE(t.update(30.0, 0.3, 1.0, 25.0));   // dwell starts again at 30
  EXPECT_TRUE(t.update(55.0, 0.3, 1.0, 25.0));
}
