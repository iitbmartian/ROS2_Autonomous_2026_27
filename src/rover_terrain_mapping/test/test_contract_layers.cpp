// Copyright 2026 Mars Rover Team, IIT Bombay.

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>

#include "fastdem/postprocess/feature_extraction.hpp"
#include "rover_terrain_mapping/contract_layers.hpp"
#include "rover_traversability/layer_names.hpp"

namespace L = rover_traversability::layers;
using rover_terrain_mapping::toElevationContract;
using rover_terrain_mapping::toFeatureContract;

namespace
{

fastdem::ElevationMap featureMap()
{
  fastdem::ElevationMap m(1.0, 1.0, 0.1, "odom");
  m.setTimestamp(123456789ULL);
  m.get(fastdem::layer::elevation).setConstant(0.5f);
  m.add(fastdem::layer::variance, 0.01f);
  m.add(fastdem::layer::n_points, 3.0f);
  m.add(fastdem::layer::slope, 30.0f);      // degrees, as FastDEM publishes it
  m.add(fastdem::layer::step, 0.07f);
  m.add(fastdem::layer::roughness, 0.02f);
  m.add(fastdem::layer::curvature, 0.1f);
  m.add(fastdem::layer::normal_x, 0.5f);
  m.add(fastdem::layer::normal_y, 0.0f);
  m.add(fastdem::layer::normal_z, 0.866f);
  return m;
}

}  // namespace

TEST(ContractLayers, SlopeDegreesBecomeRadians)
{
  auto src = featureMap();
  src.get(fastdem::layer::slope)(0, 1) = 0.0f;
  src.get(fastdem::layer::slope)(0, 2) = NAN;
  const auto out = toFeatureContract(src);
  EXPECT_NEAR(out.get(L::kSlope)(0, 0), 0.5235988f, 1e-6);   // 30 deg
  EXPECT_FLOAT_EQ(out.get(L::kSlope)(0, 1), 0.0f);
  EXPECT_TRUE(std::isnan(out.get(L::kSlope)(0, 2)));
}

TEST(ContractLayers, OtherFeaturesPassThroughUnchanged)
{
  const auto out = toFeatureContract(featureMap());
  EXPECT_FLOAT_EQ(out.get(L::kStep)(1, 1), 0.07f);
  EXPECT_FLOAT_EQ(out.get(L::kRoughness)(1, 1), 0.02f);
  EXPECT_FLOAT_EQ(out.get(L::kCurvature)(1, 1), 0.1f);
  EXPECT_FLOAT_EQ(out.get(L::kNormalX)(1, 1), 0.5f);
  EXPECT_FLOAT_EQ(out.get(L::kNormalZ)(1, 1), 0.866f);
  EXPECT_FLOAT_EQ(out.get(L::kElevation)(1, 1), 0.5f);
}

TEST(ContractLayers, NamesAreContractNamesOnly)
{
  const auto & names = rover_terrain_mapping::featureContractLayers();
  const std::set<std::string> got(names.begin(), names.end());
  const std::set<std::string> want = {
    L::kElevation, L::kSlope, L::kStep, L::kRoughness, L::kCurvature,
    L::kNormalX, L::kNormalY, L::kNormalZ};
  EXPECT_EQ(got, want);
  const auto out = toFeatureContract(featureMap());
  for (const auto & n : names) {
    EXPECT_TRUE(out.exists(n)) << n;
    EXPECT_NE(n.front(), '_') << n;
  }
}

TEST(ContractLayers, ElevationLayersRenamed)
{
  const auto out = toElevationContract(featureMap());
  EXPECT_FLOAT_EQ(out.get(L::kVariance)(0, 0), 0.01f);
  EXPECT_FLOAT_EQ(out.get(L::kObservations)(0, 0), 3.0f);
  EXPECT_FLOAT_EQ(out.get(L::kElevation)(0, 0), 0.5f);
}

TEST(ContractLayers, MissingSourceLayerIsNaN)
{
  fastdem::ElevationMap m(1.0, 1.0, 0.1, "odom");
  const auto out = toElevationContract(m);
  EXPECT_TRUE(std::isnan(out.get(L::kVariance)(0, 0)));
}

TEST(ContractLayers, KeepsStampFrameAndGeometry)
{
  const auto src = featureMap();
  const auto out = toFeatureContract(src);
  EXPECT_EQ(out.getTimestamp(), 123456789ULL);
  EXPECT_EQ(out.getFrameId(), "odom");
  EXPECT_FLOAT_EQ(out.getResolution(), src.getResolution());
  EXPECT_EQ(out.getSize()(0), src.getSize()(0));
}
