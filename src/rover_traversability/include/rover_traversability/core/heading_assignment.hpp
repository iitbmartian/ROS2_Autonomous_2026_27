// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Body heading along a planned path. The 2D planner gives positions only and its cell cost
// already assumes the best heading at every cell. This picks the heading the rover should
// hold at each pose so that every pose is safe and the heading changes little, by dynamic
// programming over yaw bins (exact for the bin set). Used for swerve, where heading and
// travel direction are independent, and for Ackermann with crabbing disallowed, where it
// only chooses forward or reverse and reports a path that no allowed heading can drive.

#ifndef ROVER_TRAVERSABILITY__CORE__HEADING_ASSIGNMENT_HPP_
#define ROVER_TRAVERSABILITY__CORE__HEADING_ASSIGNMENT_HPP_

#include <vector>

namespace rover_traversability
{
namespace core
{

struct HeadingParams
{
  int bins{16};
  double w_util{1.0};      // per unit utilisation
  double w_crab{0.5};      // per unit |sin(heading - travel)|, sideways travel
  double w_reverse{1.0};   // per unit max(0, -cos(heading - travel)), backwards travel
  double w_turn{0.3};      // per bin of heading change between consecutive poses
  bool allow_crab{true};   // false: only headings within half a bin of travel or its reverse
};

struct HeadingResult
{
  std::vector<double> yaw;   // rad, one per pose
  std::vector<int> bin;
  bool feasible{false};      // every pose got a safe (finite u <= 1) allowed heading
  int first_infeasible{-1};  // index of the first pose with no safe heading, -1 if none
};

// util[i][b]: utilisation of pose i at heading b * 2 pi / bins. Values above 1, NaN or inf
// mean unsafe or unknown. travel[i]: direction of travel at pose i, rad.
HeadingResult assignHeadings(
  const std::vector<std::vector<double>> & util, const std::vector<double> & travel,
  const HeadingParams & p);

// Direction of travel per pose from consecutive positions. The last pose copies the previous
// one, a single pose gets 0.
std::vector<double> travelDirections(const std::vector<double> & x, const std::vector<double> & y);

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__HEADING_ASSIGNMENT_HPP_
