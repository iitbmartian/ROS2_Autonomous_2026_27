// Copyright 2026 Mars Rover Team, IIT Bombay.

#include "rover_traversability/core/pose_evaluation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace rover_traversability
{
namespace core
{

bool TerrainGrid::cell(double px, double py, int & i, int & j) const
{
  i = static_cast<int>(std::lround(row(px)));
  j = static_cast<int>(std::lround(col(py)));
  return i >= 0 && j >= 0 && i < rows() && j < cols();
}

double stabilityMargin(double a, double b, double hx, double hy, double cx, double cy, double h)
{
  using V = Eigen::Vector3d;
  const V z_up(0.0, 0.0, 1.0);
  // Body axes on the support plane z = a x + b y: x along the heading, z the plane normal.
  const V n = V(-a, -b, 1.0).normalized();
  const V ex = V(1.0, 0.0, a).normalized();
  const V ey = n.cross(ex);
  const V cg = cx * ex + cy * ey + h * n;
  // Wheels are fixed in the body: hx and hy apart along the plane, not horizontally. Placed
  // horizontally the support polygon would widen by 1 / cos(tilt) and overstate the margin.
  auto contact = [&](double sx, double sy) {return V(sx * hx * ex + sy * hy * ey);};
  // Counter-clockwise seen from above: FL, RL, RR, FR. The interior is left of each edge.
  const std::array<V, 4> c = {contact(1, 1), contact(-1, 1), contact(-1, -1), contact(1, -1)};
  double margin = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 4; ++k) {
    const V & P = c[k];
    const V u = (c[(k + 1) % 4] - P).normalized();
    const V l = cg - P;
    const V l_perp = l - l.dot(u) * u;
    // In the plane perpendicular to the edge: the vertical, and the horizontal inward direction.
    const V z_p = (z_up - z_up.dot(u) * u).normalized();
    const V inward = -(u.cross(z_up)).normalized();
    margin = std::min(margin, std::atan2(l_perp.dot(inward), l_perp.dot(z_p)));
  }
  return margin;
}

double flatStabilityMargin(const PhysicalParams & p)
{
  return stabilityMargin(
    0.0, 0.0, p.wheelbase / 2.0, p.track / 2.0, p.cg_offset_x, p.cg_offset_y, p.cg_height);
}

namespace
{

float at(const Eigen::MatrixXf & m, int i, int j, float fallback)
{
  if (m.size() == 0) {return fallback;}
  const float v = m(i, j);
  return std::isfinite(v) ? v : fallback;
}

// The four cells around a point and its bilinear weights, false outside the grid. A contact
// read from the nearest cell alone is off by up to half a cell (2.5 cm), which on a 10 deg
// slope understates the tilt by about 1 deg: lenient, so contacts interpolate.
struct Bilinear
{
  int i0, j0;
  double fi, fj;
};

bool bilinear(const TerrainGrid & g, double x, double y, Bilinear & b)
{
  const double r = g.row(x), c = g.col(y);
  if (r < 0.0 || c < 0.0 || r > g.rows() - 1 || c > g.cols() - 1) {return false;}
  b.i0 = std::min(static_cast<int>(std::floor(r)), g.rows() - 2);
  b.j0 = std::min(static_cast<int>(std::floor(c)), g.cols() - 2);
  b.fi = r - b.i0;
  b.fj = c - b.j0;
  return true;
}

// Interpolated value, NaN if any of the four cells is unknown.
double interp(const Eigen::MatrixXf & m, const Bilinear & b)
{
  const double v00 = m(b.i0, b.j0), v01 = m(b.i0, b.j0 + 1);
  const double v10 = m(b.i0 + 1, b.j0), v11 = m(b.i0 + 1, b.j0 + 1);
  return (1 - b.fi) * ((1 - b.fj) * v00 + b.fj * v01) + b.fi * ((1 - b.fj) * v10 + b.fj * v11);
}

// Largest finite value of the four cells, the strict reading for step and holes.
float max4(const Eigen::MatrixXf & m, const Bilinear & b, float fallback)
{
  if (m.size() == 0) {return fallback;}
  float out = fallback;
  for (int di = 0; di <= 1; ++di) {
    for (int dj = 0; dj <= 1; ++dj) {
      const float v = m(b.i0 + di, b.j0 + dj);
      if (std::isfinite(v)) {out = std::max(out, v);}
    }
  }
  return out;
}

}  // namespace

EvalCache makeEvalCache(const TerrainGrid & g, const PhysicalParams & p, const EvalParams & e)
{
  EvalCache c;
  c.flat_margin = flatStabilityMargin(p);
  c.belly_radius = e.belly_radius >= 0.0 ? e.belly_radius :
    std::hypot(p.wheelbase / 2.0 - p.wheel_radius, p.track / 2.0 - p.wheel_width / 2.0);
  const int br = static_cast<int>(std::ceil(c.belly_radius / g.resolution));
  for (int di = -br; di <= br; ++di) {
    for (int dj = -br; dj <= br; ++dj) {
      // Rows run along -x and columns along -y.
      const double dx = -di * g.resolution, dy = -dj * g.resolution;
      if (dx * dx + dy * dy <= c.belly_radius * c.belly_radius) {c.belly.push_back({di, dj, dx, dy});}
    }
  }
  if (g.variance.size() != 0) {
    c.sigma = g.variance.array().max(0.0f).sqrt().matrix();
  }
  return c;
}

PoseResult evaluatePose(
  const TerrainGrid & g, double x, double y, double yaw, const PhysicalParams & p,
  const EvalParams & e, const EvalCache * cache)
{
  EvalCache local;
  if (cache == nullptr) {
    local = makeEvalCache(g, p, e);
    cache = &local;
  }
  PoseResult r;
  const double hx = p.wheelbase / 2.0, hy = p.track / 2.0;
  const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
  const double k = e.risk_kappa;

  // Contacts FL, RL, RR, FR in body (+-hx, +-hy).
  const std::array<std::array<double, 2>, 4> s = {{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}};
  std::array<double, 4> z{}, var{}, step{};
  bool unknown = false;
  for (int w = 0; w < 4; ++w) {
    const double bx = s[w][0] * hx, by = s[w][1] * hy;
    Bilinear bl;
    if (!bilinear(g, x + cyaw * bx - syaw * by, y + syaw * bx + cyaw * by, bl)) {
      unknown = true;
      continue;
    }
    if (max4(g.negative_obstacle, bl, 0.0f) == 1.0f) {r.negative_contact = true;}
    const double zv = interp(g.elevation, bl);
    if (!std::isfinite(zv)) {unknown = true; continue;}
    z[w] = zv;
    var[w] = g.variance.size() ? std::max(0.0, interp(g.variance, bl)) : 0.0;
    if (!std::isfinite(var[w])) {var[w] = max4(g.variance, bl, 0.0f);}
    step[w] = max4(g.step, bl, 0.0f);
  }
  if (r.negative_contact) {
    r.status = PoseStatus::kLethal;
    r.u = std::numeric_limits<double>::infinity();
    return r;
  }
  if (unknown) {
    r.status = PoseStatus::kUnknown;
    return r;
  }

  // Least-squares plane through the four contacts, in the yaw-aligned body frame. With the
  // contacts at (+-hx, +-hy) it has a closed form.
  const double a = (z[0] + z[3] - z[1] - z[2]) / (4.0 * hx);
  const double b = (z[0] + z[1] - z[2] - z[3]) / (4.0 * hy);
  const double c = (z[0] + z[1] + z[2] + z[3]) / 4.0;
  const double sum_var = var[0] + var[1] + var[2] + var[3];
  const double sa = std::sqrt(sum_var) / (4.0 * hx);
  const double sb = std::sqrt(sum_var) / (4.0 * hy);
  r.pitch = std::atan(a);
  r.roll = std::atan(b);

  // Stability margin at the worst corner of the kappa-sigma box around (a, b).
  r.margin = std::numeric_limits<double>::infinity();
  for (const double da : {-1.0, 1.0}) {
    for (const double db : {-1.0, 1.0}) {
      r.margin = std::min(r.margin, stabilityMargin(
          a + da * k * sa, b + db * k * sb, hx, hy, p.cg_offset_x, p.cg_offset_y, p.cg_height));
    }
  }
  const double m_flat = cache->flat_margin;
  r.u_tip = std::max(0.0, (m_flat - r.margin) / (m_flat - p.stability_margin_min));

  // Articulation: rocker angles from front and rear contact heights per side.
  const double d = (z[0] - z[1]) - (z[3] - z[2]);
  const double d_eff = std::abs(d) + k * std::sqrt(sum_var);
  r.u_art = std::atan(d_eff / (2.0 * hx)) / 2.0 / p.rocker_max;

  // Wheel step: the step layer at each contact, widened by the height uncertainty there.
  double step_max = 0.0;
  for (int w = 0; w < 4; ++w) {
    step_max = std::max(step_max, step[w] + k * std::sqrt(2.0 * var[w]));
  }
  r.u_step = step_max / p.step_max;

  // Belly clearance: highest terrain (plus kappa sigma) above the support plane in the belly
  // disc around the centre cell. Unknown cells there are left to the unknown-space rules.
  int ci, cj;
  g.cell(x, y, ci, cj);
  // The centre cell sits up to half a cell from (x, y). Plane offsets are taken from (x, y).
  const double ox = g.x(ci) - x, oy = g.y(cj) - y;
  const bool with_sigma = cache->sigma.size() != 0 && k != 0.0;
  double h_max = 0.0;
  for (const auto & o : e.check_clearance ? cache->belly : std::vector<EvalCache::Offset>{}) {
    const int ii = ci + o.di, jj = cj + o.dj;
    if (ii < 0 || jj < 0 || ii >= g.rows() || jj >= g.cols()) {continue;}
    const float zv = g.elevation(ii, jj);
    if (!std::isfinite(zv)) {continue;}
    const double wx = o.dx + ox, wy = o.dy + oy;
    const double bx = cyaw * wx + syaw * wy, by = -syaw * wx + cyaw * wy;
    double h = zv - (a * bx + b * by + c);
    if (with_sigma) {
      const float s = cache->sigma(ii, jj);
      if (std::isfinite(s)) {h += k * s;}
    }
    h_max = std::max(h_max, h);
  }
  r.clearance_height = h_max;
  r.u_clear = h_max / p.ground_clearance;

  // Roughness under the support polygon. rough_radius 0 means the layer is already the max
  // over that disc (precomputed), so only the centre cell is read.
  double rough = 0.0;
  if (e.check_roughness && g.roughness.size() != 0) {
    const double rr = e.rough_radius >= 0.0 ? e.rough_radius : std::hypot(hx, hy);
    if (rr <= 0.0) {
      rough = at(g.roughness, ci, cj, 0.0f);
    } else {
      const int rc = static_cast<int>(std::ceil(rr / g.resolution));
      for (int di = -rc; di <= rc; ++di) {
        for (int dj = -rc; dj <= rc; ++dj) {
          const int ii = ci + di, jj = cj + dj;
          if (ii < 0 || jj < 0 || ii >= g.rows() || jj >= g.cols()) {continue;}
          if ((di * di + dj * dj) * g.resolution * g.resolution > rr * rr) {continue;}
          rough = std::max(rough, static_cast<double>(at(g.roughness, ii, jj, 0.0f)));
        }
      }
    }
  }
  r.u_rough = rough / p.roughness_max;

  r.u = std::max({r.u_tip, r.u_clear, r.u_art, r.u_step, r.u_rough});
  r.status = r.u > 1.0 ? PoseStatus::kLethal : PoseStatus::kOk;
  return r;
}

}  // namespace core
}  // namespace rover_traversability
