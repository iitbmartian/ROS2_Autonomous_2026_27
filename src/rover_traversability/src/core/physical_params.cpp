// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/physical_params.hpp"

#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace rover_traversability
{
namespace core
{

namespace
{

std::map<std::string, double PhysicalParams::*> fieldMap()
{
  return {
    {"wheelbase", &PhysicalParams::wheelbase},
    {"track", &PhysicalParams::track},
    {"wheel_radius", &PhysicalParams::wheel_radius},
    {"wheel_width", &PhysicalParams::wheel_width},
    {"footprint_length", &PhysicalParams::footprint_length},
    {"footprint_width", &PhysicalParams::footprint_width},
    {"cg_height", &PhysicalParams::cg_height},
    {"cg_offset_x", &PhysicalParams::cg_offset_x},
    {"cg_offset_y", &PhysicalParams::cg_offset_y},
    {"mass", &PhysicalParams::mass},
    {"ground_clearance", &PhysicalParams::ground_clearance},
    {"rocker_max", &PhysicalParams::rocker_max},
    {"step_max", &PhysicalParams::step_max},
    {"stability_margin_min", &PhysicalParams::stability_margin_min},
  };
}

}  // namespace

const std::vector<std::string> & physicalParamKeys()
{
  static const std::vector<std::string> keys = {
    "wheelbase", "track", "wheel_radius", "wheel_width", "footprint_length", "footprint_width",
    "cg_height", "cg_offset_x", "cg_offset_y", "mass", "ground_clearance", "rocker_max",
    "step_max", "stability_margin_min"};
  return keys;
}

PhysicalParams loadPhysicalParams(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("rover_physical: cannot read " + path);
  }
  std::stringstream buf;
  buf << in.rdbuf();
  const std::string text = buf.str();

  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("rover_physical: " + path + ": " + e.what());
  }

  PhysicalParams p;
  const auto fields = fieldMap();
  for (const auto & key : physicalParamKeys()) {
    if (!root[key]) {
      throw std::runtime_error("rover_physical: " + path + ": missing key '" + key + "'");
    }
    try {
      p.*(fields.at(key)) = root[key].as<double>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error("rover_physical: " + path + ": key '" + key + "' is not a number");
    }
  }

  // yaml-cpp drops comments, so the marker is read from the raw line that defines the key.
  static const std::regex key_line(R"(^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:.*#.*\bPLACEHOLDER\b)");
  std::istringstream lines(text);
  std::string line;
  std::smatch m;
  while (std::getline(lines, line)) {
    if (std::regex_search(line, m, key_line) && fields.count(m[1].str())) {
      p.placeholders.push_back(m[1].str());
    }
  }
  return p;
}

std::string defaultPhysicalParamsPath()
{
  return ament_index_cpp::get_package_share_directory("rover_traversability") +
         "/config/rover_physical.yaml";
}

std::string placeholderWarning(const PhysicalParams & params)
{
  if (params.placeholders.empty()) {
    return "";
  }
  std::string s = "rover_physical.yaml: placeholder values in use, not measured on the rover:";
  for (const auto & k : params.placeholders) {
    s += " " + k;
  }
  return s;
}

}  // namespace core
}  // namespace rover_traversability
