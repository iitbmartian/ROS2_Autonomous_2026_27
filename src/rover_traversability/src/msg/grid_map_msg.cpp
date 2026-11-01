// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/grid_map_msg.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace rover_traversability
{

GridGeometry geometryOf(const grid_map_msgs::msg::GridMap & msg)
{
  GridGeometry g;
  g.resolution = msg.info.resolution;
  g.length_x = msg.info.length_x;
  g.length_y = msg.info.length_y;
  g.center_x = msg.info.pose.position.x;
  g.center_y = msg.info.pose.position.y;
  g.rows = static_cast<int>(std::lround(g.length_x / g.resolution));
  g.cols = static_cast<int>(std::lround(g.length_y / g.resolution));
  return g;
}

bool readLayer(
  const grid_map_msgs::msg::GridMap & msg, const std::string & name, Eigen::MatrixXf & out)
{
  const auto it = std::find(msg.layers.begin(), msg.layers.end(), name);
  if (it == msg.layers.end()) {
    return false;
  }
  const auto & arr = msg.data[static_cast<size_t>(it - msg.layers.begin())];
  if (arr.layout.dim.size() != 2) {
    return false;
  }
  const int cols = static_cast<int>(arr.layout.dim[0].size);
  const int rows = static_cast<int>(arr.layout.dim[1].size);
  if (rows <= 0 || cols <= 0 || arr.data.size() != static_cast<size_t>(rows) * cols) {
    return false;
  }
  const int r0 = static_cast<int>(msg.outer_start_index) % rows;
  const int c0 = static_cast<int>(msg.inner_start_index) % cols;
  out.resize(rows, cols);
  for (int j = 0; j < cols; ++j) {
    const int sc = (j + c0) % cols;
    for (int i = 0; i < rows; ++i) {
      const int sr = (i + r0) % rows;
      out(i, j) = arr.data[static_cast<size_t>(sc) * rows + sr];
    }
  }
  return true;
}

void writeLayer(
  grid_map_msgs::msg::GridMap & msg, const std::string & name, const Eigen::MatrixXf & data)
{
  std_msgs::msg::Float32MultiArray arr;
  arr.layout.dim.resize(2);
  arr.layout.dim[0].label = "column_index";
  arr.layout.dim[0].size = static_cast<uint32_t>(data.cols());
  arr.layout.dim[0].stride = static_cast<uint32_t>(data.rows() * data.cols());
  arr.layout.dim[1].label = "row_index";
  arr.layout.dim[1].size = static_cast<uint32_t>(data.rows());
  arr.layout.dim[1].stride = static_cast<uint32_t>(data.rows());
  // Eigen's default storage is column-major, the same order as the message.
  arr.data.assign(data.data(), data.data() + data.size());
  msg.layers.push_back(name);
  msg.data.push_back(std::move(arr));
}

}  // namespace rover_traversability
