// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/disc_dilation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rover_traversability
{
namespace core
{

void DiscDilation::setRadius(double radius_m, double resolution_m)
{
  if (radius_m <= 0.0 || resolution_m <= 0.0) {
    radius_cells_ = 0;
    chord_offsets_by_h_.clear();
    return;
  }

  radius_cells_ = static_cast<int>(std::lround(radius_m / resolution_m));
  chord_offsets_by_h_.assign(radius_cells_ + 1, std::vector<int>{});

  const double r2 = static_cast<double>(radius_cells_) * radius_cells_;
  for (int di = -radius_cells_; di <= radius_cells_; ++di) {
    const double rem = std::max(0.0, r2 - static_cast<double>(di) * di);
    const int h = static_cast<int>(std::floor(std::sqrt(rem)));
    chord_offsets_by_h_[h].push_back(di);
  }
}

void DiscDilation::apply(
  unsigned char * buffer, int size_x, int size_y,
  int min_i, int min_j, int max_i, int max_j)
{
  const int r = radius_cells_;
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(size_x, max_i);
  max_j = std::min(size_y, max_j);
  if (r <= 0 || min_i >= max_i || min_j >= max_j) {
    return;
  }

  auto idx = [size_x](int i, int j) {
      return static_cast<size_t>(j) * static_cast<size_t>(size_x) + static_cast<size_t>(i);
    };

  // Padded source window, clamped to the grid.
  const int i0 = std::max(0, min_i - r);
  const int i1 = std::min(size_x, max_i + r);
  const int j0 = std::max(0, min_j - r);
  const int j1 = std::min(size_y, max_j + r);
  const int W = i1 - i0;
  const int Wb = max_i - min_i;
  const int Hb = max_j - min_j;

  // Snapshot the source. No-data folds to 0 so it never wins a max.
  in_.assign(static_cast<size_t>(W) * static_cast<size_t>(j1 - j0), 0);
  for (int j = j0; j < j1; ++j) {
    const size_t row = static_cast<size_t>(j - j0) * W;
    for (int i = i0; i < i1; ++i) {
      const unsigned char v = buffer[idx(i, j)];
      in_[row + static_cast<size_t>(i - i0)] = (v == kNoInformation) ? 0 : v;
    }
  }

  // v_ holds V_h over the window rows only. It starts as V_0 = source.
  v_.assign(static_cast<size_t>(W) * static_cast<size_t>(Hb), 0);
  for (int j = min_j; j < max_j; ++j) {
    const size_t vrow = static_cast<size_t>(j - min_j) * W;
    const size_t srow = static_cast<size_t>(j - j0) * W;
    std::copy_n(&in_[srow], W, &v_[vrow]);
  }

  out_.assign(static_cast<size_t>(Wb) * static_cast<size_t>(Hb), 0);

  for (int h = 0; h <= r; ++h) {
    // Grow every chord by one cell at each end, O(1) per cell per h.
    if (h > 0) {
      for (int j = min_j; j < max_j; ++j) {
        const size_t vrow = static_cast<size_t>(j - min_j) * W;
        const int ja = j - h;
        const int jb = j + h;
        const bool have_a = (ja >= j0);
        const bool have_b = (jb < j1);
        const size_t arow = have_a ? static_cast<size_t>(ja - j0) * W : 0;
        const size_t brow = have_b ? static_cast<size_t>(jb - j0) * W : 0;
        for (int c = 0; c < W; ++c) {
          unsigned char m = v_[vrow + c];
          if (have_a) {m = std::max(m, in_[arow + c]);}
          if (have_b) {m = std::max(m, in_[brow + c]);}
          v_[vrow + c] = m;
        }
      }
    }

    // Each of the 2r+1 column offsets is folded in exactly once, at its own chord height, so the
    // structuring element is exactly di^2 + dj^2 <= r^2.
    for (const int di : chord_offsets_by_h_[h]) {
      for (int j = min_j; j < max_j; ++j) {
        const size_t vrow = static_cast<size_t>(j - min_j) * W;
        const size_t orow = static_cast<size_t>(j - min_j) * Wb;
        for (int i = min_i; i < max_i; ++i) {
          const int si = i + di;
          if (si < i0 || si >= i1) {continue;}
          const unsigned char v = v_[vrow + static_cast<size_t>(si - i0)];
          unsigned char & o = out_[orow + static_cast<size_t>(i - min_i)];
          if (v > o) {o = v;}
        }
      }
    }
  }

  // A zero result means nothing in range had a cost, so the original value stays. That keeps
  // no-data cells as no-data.
  for (int j = min_j; j < max_j; ++j) {
    const size_t orow = static_cast<size_t>(j - min_j) * Wb;
    for (int i = min_i; i < max_i; ++i) {
      const unsigned char m = out_[orow + static_cast<size_t>(i - min_i)];
      if (m > 0) {
        buffer[idx(i, j)] = m;
      }
    }
  }
}

}  // namespace core
}  // namespace rover_traversability
