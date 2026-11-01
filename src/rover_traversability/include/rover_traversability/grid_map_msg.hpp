// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// grid_map_msgs/GridMap to and from plain Eigen matrices, without grid_map_core. grid_map_core
// adds Eigen plugin definitions to every target that uses it, which must not reach FastDEM's
// Eigen code in this package.
//
// Message layout (grid_map): each layer is a Float32MultiArray, column-major, dim[0] the column
// index (size_y cells), dim[1] the row index (size_x cells). The map is a circular buffer:
// unwrapped cell (i, j) is stored at ((i + outer_start_index) % rows,
// (j + inner_start_index) % cols). Rows run along -x and columns along -y from the +x, +y corner.

#ifndef ROVER_TRAVERSABILITY__GRID_MAP_MSG_HPP_
#define ROVER_TRAVERSABILITY__GRID_MAP_MSG_HPP_

#include <string>

#include <Eigen/Core>

#include "grid_map_msgs/msg/grid_map.hpp"

namespace rover_traversability
{

struct GridGeometry
{
  int rows{0}, cols{0};
  double resolution{0.0};
  double center_x{0.0}, center_y{0.0};   // map frame position of the map centre
  double length_x{0.0}, length_y{0.0};

  // Fractional cell coordinates of a map-frame point (cell centres at whole numbers).
  double row(double x) const {return (length_x / 2.0 - (x - center_x)) / resolution - 0.5;}
  double col(double y) const {return (length_y / 2.0 - (y - center_y)) / resolution - 0.5;}
  double x(double row) const {return center_x + length_x / 2.0 - (row + 0.5) * resolution;}
  double y(double col) const {return center_y + length_y / 2.0 - (col + 0.5) * resolution;}
};

GridGeometry geometryOf(const grid_map_msgs::msg::GridMap & msg);

// Unwrapped layer, or false when the layer is missing or malformed.
bool readLayer(
  const grid_map_msgs::msg::GridMap & msg, const std::string & name, Eigen::MatrixXf & out);

// Appends an unwrapped layer to msg. The message must carry start indices 0.
void writeLayer(
  grid_map_msgs::msg::GridMap & msg, const std::string & name, const Eigen::MatrixXf & data);

}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__GRID_MAP_MSG_HPP_
