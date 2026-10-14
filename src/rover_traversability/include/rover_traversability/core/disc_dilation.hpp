// Copyright 2026 Mars Rover Team, IIT Bombay.
//
// Grey-scale dilation of a cost grid by a true disc structuring element.
//
// The disc is decomposed into vertical chords:
//   dilate(in)(i, j) = max over di of V_h(di)(i + di, j),  h(di) = floor(sqrt(r^2 - di^2))
// where V_h is the source dilated vertically by a chord of half height h, grown incrementally as
//   V_h = max(V_h-1, in(j - h), in(j + h)).
//
// Do not replace this with a separable horizontal-then-vertical max filter. That dilates by the
// bounding square, over-inflating by sqrt(2) along the diagonals, and closes gaps the rover can
// drive through.

#ifndef ROVER_TRAVERSABILITY__CORE__DISC_DILATION_HPP_
#define ROVER_TRAVERSABILITY__CORE__DISC_DILATION_HPP_

#include <vector>

namespace rover_traversability
{
namespace core
{

class DiscDilation
{
public:
  // Value treated as "no data". It contributes nothing to the max and is kept where nothing in
  // range has a cost.
  static constexpr unsigned char kNoInformation = 255;

  // Rebuilds the chord kernel. The radius in cells is round(radius_m / resolution_m).
  void setRadius(double radius_m, double resolution_m);

  int radiusCells() const {return radius_cells_;}

  // Dilates buffer (row-major, index j * size_x + i) in place over the window
  // [min_i, max_i) x [min_j, max_j). Cells up to the radius outside the window are read as the
  // source, so a window on the edge of a rolling costmap is not under-inflated on its leading
  // edge. Only cells inside the window are written.
  void apply(
    unsigned char * buffer, int size_x, int size_y,
    int min_i, int min_j, int max_i, int max_j);

private:
  int radius_cells_{0};
  // chord_offsets_by_h_[h] holds every column offset di whose chord half height is h.
  std::vector<std::vector<int>> chord_offsets_by_h_;
  // Reused across calls so apply does no heap allocation in steady state.
  std::vector<unsigned char> in_, v_, out_;
};

}  // namespace core
}  // namespace rover_traversability

#endif  // ROVER_TRAVERSABILITY__CORE__DISC_DILATION_HPP_
