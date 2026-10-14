// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Conversion from FastDEM's layers to the SPEC.md map contract (layer names in
// rover_traversability/layer_names.hpp, slope in radians).

#ifndef ROVER_TERRAIN_MAPPING__CONTRACT_LAYERS_HPP_
#define ROVER_TERRAIN_MAPPING__CONTRACT_LAYERS_HPP_

#include <string>
#include <vector>

#include "fastdem/elevation_map.hpp"

namespace rover_terrain_mapping
{

// Layers of the elevation map published on /terrain/local, in contract names.
const std::vector<std::string> & elevationContractLayers();

// Layers of the feature map published on /terrain/local_features, in contract names.
const std::vector<std::string> & featureContractLayers();

// Copy of the elevation layers of a FastDEM map under contract names:
// elevation, variance, observations (FastDEM's n_points). Missing source layers come out as NaN.
fastdem::ElevationMap toElevationContract(const fastdem::ElevationMap & src);

// PHASE 0 BRIDGE, replaced by terrain_features_node in Phase 2.
// Copy of FastDEM's feature layers under contract names, plus elevation. FastDEM's slope is in
// degrees and becomes radians here, its _normal_* layers become normal_*. Every other value
// passes through unchanged. FastDEM's definitions differ from SPEC.md (see PROGRESS.md).
fastdem::ElevationMap toFeatureContract(const fastdem::ElevationMap & src);

}  // namespace rover_terrain_mapping

#endif  // ROVER_TERRAIN_MAPPING__CONTRACT_LAYERS_HPP_
