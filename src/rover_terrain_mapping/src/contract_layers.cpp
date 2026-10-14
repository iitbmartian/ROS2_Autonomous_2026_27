// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_terrain_mapping/contract_layers.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "fastdem/postprocess/feature_extraction.hpp"
#include "rover_traversability/layer_names.hpp"

namespace rover_terrain_mapping
{

namespace L = rover_traversability::layers;

namespace
{

using Renames = std::vector<std::pair<std::string, std::string>>;   // {fastdem, contract}

const Renames & elevationRenames()
{
  static const Renames r = {
    {fastdem::layer::elevation, L::kElevation},
    {fastdem::layer::variance, L::kVariance},
    {fastdem::layer::n_points, L::kObservations},
  };
  return r;
}

const Renames & featureRenames()
{
  static const Renames r = {
    {fastdem::layer::elevation, L::kElevation},
    {fastdem::layer::slope, L::kSlope},
    {fastdem::layer::step, L::kStep},
    {fastdem::layer::roughness, L::kRoughness},
    {fastdem::layer::curvature, L::kCurvature},
    {fastdem::layer::normal_x, L::kNormalX},
    {fastdem::layer::normal_y, L::kNormalY},
    {fastdem::layer::normal_z, L::kNormalZ},
  };
  return r;
}

std::vector<std::string> contractNames(const Renames & r)
{
  std::vector<std::string> names;
  for (const auto & p : r) {
    names.push_back(p.second);
  }
  return names;
}

fastdem::ElevationMap copyRenamed(const fastdem::ElevationMap & src, const Renames & renames)
{
  // snapshot({}) copies geometry, frame, position, start index and stamp, and no layer data.
  fastdem::ElevationMap out = src.snapshot({});
  for (const auto & [from, to] : renames) {
    const bool have = src.exists(from);
    if (out.exists(to)) {
      if (have) {
        out.get(to) = src.get(from);
      } else {
        out.get(to).setConstant(NAN);
      }
    } else if (have) {
      out.add(to, src.get(from));
    } else {
      out.add(to, NAN);
    }
  }
  return out;
}

}  // namespace

const std::vector<std::string> & elevationContractLayers()
{
  static const std::vector<std::string> names = contractNames(elevationRenames());
  return names;
}

const std::vector<std::string> & featureContractLayers()
{
  static const std::vector<std::string> names = contractNames(featureRenames());
  return names;
}

fastdem::ElevationMap toElevationContract(const fastdem::ElevationMap & src)
{
  return copyRenamed(src, elevationRenames());
}

fastdem::ElevationMap toFeatureContract(const fastdem::ElevationMap & src)
{
  fastdem::ElevationMap out = copyRenamed(src, featureRenames());
  // FastDEM publishes slope in degrees (feature_extraction.cpp). NaN stays NaN.
  out.get(L::kSlope) *= static_cast<float>(M_PI / 180.0);
  return out;
}

}  // namespace rover_terrain_mapping
