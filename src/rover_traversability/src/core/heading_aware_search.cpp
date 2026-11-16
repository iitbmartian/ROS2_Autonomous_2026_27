// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/heading_aware_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace rover_traversability
{
namespace core
{

bool SearchGrid::cell(double x, double y, int & idx) const
{
  const int ix = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int iy = static_cast<int>(std::floor((y - origin_y) / resolution));
  if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) {return false;}
  idx = iy * nx + ix;
  return true;
}

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// Cost factor of one cell, or inf when it blocks.
double cellFactor(const SearchGrid & g, const SearchParams & p, int idx)
{
  const int c = g.cost[idx];
  if (c == 255) {
    return p.allow_unknown ? 1.0 + p.cost_penalty * p.unknown_cost / 252.0 : kInf;
  }
  if (c >= p.lethal) {return kInf;}
  return 1.0 + p.cost_penalty * c / 252.0;
}

// Cost to go from every cell to the goal over the 8-connected grid, ignoring heading. It
// never exceeds the hybrid cost, since every hybrid move is at least as long and as costly.
std::vector<double> costToGo(const SearchGrid & g, const SearchParams & p, int goal)
{
  std::vector<double> h(g.cost.size(), kInf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
  h[goal] = 0.0;
  q.push({0.0, goal});
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!q.empty()) {
    auto [d, i] = q.top();
    q.pop();
    if (d > h[i]) {continue;}
    const int ix = i % g.nx, iy = i / g.nx;
    const double fi = cellFactor(g, p, i);
    for (int k = 0; k < 8; ++k) {
      const int jx = ix + dx[k], jy = iy + dy[k];
      if (jx < 0 || jy < 0 || jx >= g.nx || jy >= g.ny) {continue;}
      const int j = jy * g.nx + jx;
      const double fj = cellFactor(g, p, j);
      if (!std::isfinite(fj)) {continue;}
      // The goal cell may itself be costly, so its own factor does not block the expansion.
      const double f = 0.5 * ((std::isfinite(fi) ? fi : fj) + fj);
      const double nd = d + f * g.resolution * (k < 4 ? 1.0 : M_SQRT2);
      if (nd < h[j]) {h[j] = nd; q.push({nd, j});}
    }
  }
  return h;
}

bool headingOk(const SearchGrid & g, int idx, int bin)
{
  return g.mask.empty() || ((g.mask[idx] >> bin) & 1u);
}

struct Node
{
  SearchPose s;
  int bin;
  double g;
  int parent;
};

// Pose after signed arc length len at curvature kappa (0 straight).
SearchPose move(const SearchPose & a, double kappa, double len)
{
  if (kappa == 0.0) {
    return {a.x + len * std::cos(a.yaw), a.y + len * std::sin(a.yaw), a.yaw};
  }
  const double phi = kappa * len;
  return {a.x + (std::sin(a.yaw + phi) - std::sin(a.yaw)) / kappa,
    a.y - (std::cos(a.yaw + phi) - std::cos(a.yaw)) / kappa, a.yaw + phi};
}

}  // namespace

SearchResult headingAwareSearch(
  const SearchGrid & g, const SearchParams & p, const SearchPose & start,
  double goal_x, double goal_y)
{
  SearchResult out;
  const auto t0 = std::chrono::steady_clock::now();
  int start_idx, goal_idx;
  if (!g.cell(start.x, start.y, start_idx) || !g.cell(goal_x, goal_y, goal_idx)) {return out;}
  const auto h = costToGo(g, p, goal_idx);

  const int B = std::max(4, p.bins);
  const double step = 2.0 * M_PI / B;
  const double len = p.turning_radius * step;
  const double kappa = 1.0 / p.turning_radius;
  auto binOf = [&](double yaw) {
      int b = static_cast<int>(std::lround(yaw / step)) % B;
      return b < 0 ? b + B : b;
    };

  std::vector<Node> nodes;
  nodes.reserve(1 << 16);
  std::vector<double> best(g.cost.size() * B, kInf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  const int b0 = binOf(start.yaw);
  nodes.push_back({{start.x, start.y, b0 * step}, b0, 0.0, -1});
  best[static_cast<size_t>(start_idx) * B + b0] = 0.0;
  open.push({p.heuristic_weight * h[start_idx], 0});

  // (curvature sign, direction): straight, left, right, forward then reverse.
  const int curv[3] = {0, 1, -1};
  int goal_node = -1;
  while (!open.empty()) {
    const int ni = open.top().second;
    open.pop();
    const Node n = nodes[ni];
    int nidx;
    g.cell(n.s.x, n.s.y, nidx);
    if (n.g > best[static_cast<size_t>(nidx) * B + n.bin]) {continue;}
    if (std::hypot(n.s.x - goal_x, n.s.y - goal_y) <= p.tolerance) {goal_node = ni; break;}
    if (++out.iterations > p.max_iterations) {break;}
    if ((out.iterations & 1023) == 0 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > p.max_time)
    {
      out.timed_out = true;
      return out;
    }
    for (int dir = 1; dir >= (p.allow_reverse ? -1 : 1); dir -= 2) {
      for (int c : curv) {
        const double k = c * kappa;
        const SearchPose mid = move(n.s, k, 0.5 * dir * len);
        const SearchPose end = move(n.s, k, dir * len);
        const int eb = (n.bin + c * dir + B) % B;
        int mi, ei;
        if (!g.cell(mid.x, mid.y, mi) || !g.cell(end.x, end.y, ei)) {continue;}
        const double fm = cellFactor(g, p, mi), fe = cellFactor(g, p, ei);
        if (!std::isfinite(fm) || !std::isfinite(fe) || !std::isfinite(h[ei])) {continue;}
        // Body heading must be safe where the rover is: the start bin or the end bin at the
        // midpoint of an arc, the end bin at its end.
        if (!(headingOk(g, mi, n.bin) || headingOk(g, mi, eb)) || !headingOk(g, ei, eb)) {
          continue;
        }
        double m = len * 0.5 * (fm + fe);
        if (c != 0) {m *= p.non_straight_penalty;}
        if (dir < 0) {m *= p.reverse_penalty;}
        const double ng = n.g + m;
        double & b = best[static_cast<size_t>(ei) * B + eb];
        if (ng >= b) {continue;}
        b = ng;
        nodes.push_back({{end.x, end.y, eb * step}, eb, ng, ni});
        open.push({ng + p.heuristic_weight * h[ei], static_cast<int>(nodes.size()) - 1});
      }
    }
  }
  if (goal_node < 0) {return out;}

  std::vector<SearchPose> rev;
  for (int i = goal_node; i >= 0; i = nodes[i].parent) {
    rev.push_back(nodes[i].s);
  }
  std::reverse(rev.begin(), rev.end());
  for (size_t i = 0; i < rev.size(); ++i) {
    SearchPose s = rev[i];
    s.yaw = std::remainder(s.yaw, 2.0 * M_PI);
    out.path.push_back(s);
  }
  // End exactly on the goal, keeping the last heading, when it is not blocked.
  if (std::isfinite(cellFactor(g, p, goal_idx))) {
    out.path.push_back({goal_x, goal_y, out.path.back().yaw});
  }
  out.found = true;
  return out;
}

}  // namespace core
}  // namespace rover_traversability
