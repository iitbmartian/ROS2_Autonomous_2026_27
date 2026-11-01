// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// The TerrainCritic plugin export: MPPI finds it by name through pluginlib. The scoring is tested
// in test_trajectory_cost.

#include <gtest/gtest.h>

#include <memory>

#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"

TEST(TerrainCriticPlugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
    "nav2_mppi_controller", "mppi::critics::CriticFunction");
  std::shared_ptr<mppi::critics::CriticFunction> critic;
  ASSERT_NO_THROW(critic = loader.createSharedInstance("rover_traversability::TerrainCritic"));
  EXPECT_NE(critic, nullptr);
}
