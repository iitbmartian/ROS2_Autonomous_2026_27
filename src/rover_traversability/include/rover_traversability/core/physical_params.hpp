// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Loader for config/rover_physical.yaml, the single source of physical parameters.

#ifndef ROVER_TRAVERSABILITY__CORE__PHYSICAL_PARAMS_HPP_
#define ROVER_TRAVERSABILITY__CORE__PHYSICAL_PARAMS_HPP_

#include <string>
#include <vector>

namespace rover_traversability
{
namespace core
{

struct PhysicalParams
{
  double wheelbase{0.0};
  double track{0.0};
  double wheel_radius{0.0};
  double wheel_width{0.0};
  double footprint_length{0.0};
  double footprint_width{0.0};
  double cg_height{0.0};
  double cg_offset_x{0.0};
  double cg_offset_y{0.0};
  double mass{0.0};
  double ground_clearance{0.0};
  double rocker_max{0.0};
  double step_max{0.0};
  double stability_margin_min{0.0};

  // Keys whose line carries the "# PLACEHOLDER" marker, in file order.
  std::vector<std::string> placeholders;
};

// Every key the file must define.
const std::vector<std::string> & physicalParamKeys();

// Throws std::runtime_error naming the file and key when the file is unreadable, a key is
// missing, or a value is not a number.
PhysicalParams loadPhysicalParams(const std::string & path);

// Path of the installed rover_physical.yaml (share/rover_traversability/config).
std::string defaultPhysicalParamsPath();

// One line for a startup warning, empty when nothing is a placeholder.
std::string placeholderWarning(const PhysicalParams & params);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__PHYSICAL_PARAMS_HPP_
