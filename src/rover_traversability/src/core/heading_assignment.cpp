// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/heading_assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rover_traversability
{
namespace core
{

namespace
{
// Large but finite, so an unsafe pose still yields a full answer and the caller can see where.
constexpr double kUnsafe = 1e6;
}

HeadingResult assignHeadings(
  const std::vector<std::vector<double>> & util, const std::vector<double> & travel,
  const HeadingParams & p)
{
  HeadingResult out;
  const int n = static_cast<int>(std::min(util.size(), travel.size()));
  const int B = std::max(1, p.bins);
  if (n == 0) {out.feasible = true; return out;}
  const double step = 2.0 * M_PI / B;
  // Half a bin plus a little, so the bin nearest the travel direction is always allowed.
  const double crab_tol = std::sin(0.5 * step) + 1e-9;

  auto node = [&](int i, int b, bool & safe) {
      const double d = b * step - travel[i];
      const double s = std::abs(std::sin(d)), c = std::cos(d);
      double u = static_cast<size_t>(b) < util[i].size() ? util[i][b] : NAN;
      safe = std::isfinite(u) && u <= 1.0;
      double cost = safe ? p.w_util * u : kUnsafe;
      if (!p.allow_crab && s > crab_tol) {safe = false; cost = kUnsafe;}
      return cost + p.w_crab * s + p.w_reverse * std::max(0.0, -c);
    };

  std::vector<double> cur(B), nxt(B);
  std::vector<std::vector<int>> back(n, std::vector<int>(B, 0));
  std::vector<std::vector<char>> ok(n, std::vector<char>(B, 0));
  for (int b = 0; b < B; ++b) {bool s; cur[b] = node(0, b, s); ok[0][b] = s;}
  for (int i = 1; i < n; ++i) {
    for (int b = 0; b < B; ++b) {
      bool s; const double c = node(i, b, s); ok[i][b] = s;
      double best = std::numeric_limits<double>::infinity(); int arg = 0;
      for (int a = 0; a < B; ++a) {
        int d = std::abs(a - b); d = std::min(d, B - d);
        const double v = cur[a] + p.w_turn * d;
        if (v < best) {best = v; arg = a;}
      }
      nxt[b] = best + c; back[i][b] = arg;
    }
    std::swap(cur, nxt);
  }
  int b = static_cast<int>(std::min_element(cur.begin(), cur.end()) - cur.begin());
  out.bin.assign(n, 0); out.yaw.assign(n, 0.0);
  for (int i = n - 1; i >= 0; --i) {
    out.bin[i] = b; out.yaw[i] = std::remainder(b * step, 2.0 * M_PI);
    if (i > 0) {b = back[i][b];}
  }
  out.feasible = true;
  for (int i = 0; i < n; ++i) {
    if (!ok[i][out.bin[i]]) {out.feasible = false; out.first_infeasible = i; break;}
  }
  return out;
}

std::vector<double> travelDirections(const std::vector<double> & x, const std::vector<double> & y)
{
  const size_t n = std::min(x.size(), y.size());
  std::vector<double> t(n, 0.0);
  for (size_t i = 0; i + 1 < n; ++i) {t[i] = std::atan2(y[i + 1] - y[i], x[i + 1] - x[i]);}
  if (n >= 2) {t[n - 1] = t[n - 2];}
  return t;
}

}  // namespace core
}  // namespace rover_traversability
