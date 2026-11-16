// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Hybrid-A* over (x, y, body heading) that checks the heading, not only the cell. A costmap holds
// one cost per cell, the cost of the best heading there, so a 2D or Hybrid-A* planner on it will
// happily enter a ramp or a sidehill at a heading that would tip the rover. Here every state's
// body heading must be one the terrain allows at that cell (the safe_yaw_mask bit for its bin),
// so a car-like rover lines up with the fall line before it reaches the slope. Motion primitives
// are arcs at the minimum turning radius and straights, forward and reverse. Pure C++, no ROS.

#ifndef ROVER_TRAVERSABILITY__CORE__HEADING_AWARE_SEARCH_HPP_
#define ROVER_TRAVERSABILITY__CORE__HEADING_AWARE_SEARCH_HPP_

#include <cstdint>
#include <vector>

namespace rover_traversability
{
namespace core
{

struct SearchGrid
{
  int nx{0}, ny{0};               // cells along x and y, index = iy * nx + ix
  double resolution{0.2};         // m
  double origin_x{0.0}, origin_y{0.0};   // m, corner of cell (0, 0)
  std::vector<uint8_t> cost;      // Nav2 costs, 255 unknown, >= lethal blocks
  // Bit k set: body heading 2 pi k / bins is safe in that cell. Empty means every heading.
  std::vector<uint16_t> mask;
  bool cell(double x, double y, int & idx) const;
};

struct SearchParams
{
  int bins{16};                    // heading bins, must match the mask's
  double turning_radius{0.8};      // m
  bool allow_reverse{true};
  double reverse_penalty{2.0};     // multiplies the length of a reverse move
  double non_straight_penalty{1.2};  // multiplies the length of an arc
  double cost_penalty{2.0};        // length *= 1 + cost_penalty * cost / 252
  double unknown_cost{200.0};      // cost used for 255 cells when they are allowed
  bool allow_unknown{true};
  int lethal{253};                 // costs at or above it block
  double tolerance{0.5};           // m, a state this close to the goal ends the search
  double heuristic_weight{1.0};
  int max_iterations{500000};
  double max_time{2.0};            // s
};

struct SearchPose {double x, y, yaw;};

struct SearchResult
{
  std::vector<SearchPose> path;    // start to goal, body yaw
  bool found{false};
  bool timed_out{false};
  int iterations{0};
};

// The start state is accepted whatever its cell, the rover is already there.
SearchResult headingAwareSearch(
  const SearchGrid & g, const SearchParams & p, const SearchPose & start,
  double goal_x, double goal_y);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__HEADING_AWARE_SEARCH_HPP_
