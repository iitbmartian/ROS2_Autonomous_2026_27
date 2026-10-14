// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "rover_traversability/core/physical_params.hpp"

using rover_traversability::core::loadPhysicalParams;
using rover_traversability::core::placeholderWarning;

namespace
{
const std::string kFixtures = TEST_FIXTURE_DIR;
const std::string kShipped = SHIPPED_PHYSICAL_YAML;
}  // namespace

TEST(PhysicalParams, ReadsValuesAndPlaceholders)
{
  const auto p = loadPhysicalParams(kFixtures + "/physical_two_placeholders.yaml");
  EXPECT_DOUBLE_EQ(p.wheelbase, 1.0);
  EXPECT_DOUBLE_EQ(p.track, 0.5);
  EXPECT_DOUBLE_EQ(p.cg_height, 0.3);
  EXPECT_DOUBLE_EQ(p.cg_offset_y, -0.02);
  EXPECT_DOUBLE_EQ(p.mass, 40.0);
  EXPECT_DOUBLE_EQ(p.step_max, 0.08);
  EXPECT_DOUBLE_EQ(p.stability_margin_min, 0.2);
  // The marker on a line of its own is not attributed to ground_clearance below it.
  EXPECT_EQ(p.placeholders, (std::vector<std::string>{"cg_height", "step_max"}));
}

TEST(PhysicalParams, MissingKeyThrowsWithItsName)
{
  try {
    loadPhysicalParams(kFixtures + "/physical_missing_mass.yaml");
    FAIL() << "expected a throw";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("'mass'"), std::string::npos) << e.what();
  }
}

TEST(PhysicalParams, NonNumericValueThrows)
{
  EXPECT_THROW(
    loadPhysicalParams(kFixtures + "/physical_bad_number.yaml"), std::runtime_error);
}

TEST(PhysicalParams, MissingFileThrows)
{
  EXPECT_THROW(loadPhysicalParams(kFixtures + "/does_not_exist.yaml"), std::runtime_error);
}

TEST(PhysicalParams, ShippedFileMatchesSpec)
{
  const auto p = loadPhysicalParams(kShipped);
  EXPECT_DOUBLE_EQ(p.wheelbase, 0.744729);
  EXPECT_DOUBLE_EQ(p.track, 0.436);
  EXPECT_DOUBLE_EQ(p.wheel_radius, 0.14985);
  EXPECT_DOUBLE_EQ(p.footprint_length, 1.1);
  EXPECT_DOUBLE_EQ(p.footprint_width, 0.9);
  EXPECT_DOUBLE_EQ(p.cg_height, 0.45);
  EXPECT_DOUBLE_EQ(p.step_max, 0.10);
  EXPECT_DOUBLE_EQ(p.stability_margin_min, 0.17);
  const std::vector<std::string> expected = {
    "cg_height", "cg_offset_x", "cg_offset_y", "mass", "ground_clearance", "rocker_max",
    "step_max", "stability_margin_min"};
  EXPECT_EQ(p.placeholders, expected);
  EXPECT_NE(placeholderWarning(p).find("stability_margin_min"), std::string::npos);
}

TEST(PhysicalParams, NoWarningWithoutPlaceholders)
{
  rover_traversability::core::PhysicalParams p;
  EXPECT_TRUE(placeholderWarning(p).empty());
}
