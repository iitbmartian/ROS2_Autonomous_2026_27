// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Layer names of the elevation and feature map contract in SPEC.md. Every node and plugin uses
// these constants, never string literals, so a rename happens in one place.

#ifndef ROVER_TRAVERSABILITY__LAYER_NAMES_HPP_
#define ROVER_TRAVERSABILITY__LAYER_NAMES_HPP_

namespace rover_traversability
{
namespace layers
{

// Every elevation map.
constexpr const char * kElevation = "elevation";            // m, gravity-aligned
constexpr const char * kVariance = "variance";              // m^2
constexpr const char * kObservations = "observations";      // count
constexpr const char * kLastSeen = "last_seen";             // s, stamp of the last update

// Every feature map, in addition to the above.
constexpr const char * kSlope = "slope";                    // rad
constexpr const char * kRoughness = "roughness";            // m
constexpr const char * kStep = "step";                      // m
constexpr const char * kNormalX = "normal_x";
constexpr const char * kNormalY = "normal_y";
constexpr const char * kNormalZ = "normal_z";
constexpr const char * kCurvature = "curvature";            // dimensionless
constexpr const char * kNegativeObstacle = "negative_obstacle";  // 0 or 1
constexpr const char * kSafeYawMask = "safe_yaw_mask";      // bit k set: yaw bin k is safe
constexpr const char * kBestYaw = "best_yaw";               // rad
constexpr const char * kBestUtilisation = "best_utilisation";  // 0 benign, 1 at the limit

}  // namespace layers
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__LAYER_NAMES_HPP_
