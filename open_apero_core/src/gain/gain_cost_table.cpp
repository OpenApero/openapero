#include "open_apero_core/gain/gain_cost_table.h"

#include <algorithm>

namespace open_apero {

void GainCostTable::insert(const Path& path) {
  const double c = path.cost;
  const double g = path.gain;

  // The implicit origin (0,0) already covers non-positive cost or gain.
  if (c <= 0.0 || g <= 0.0) {
    return;
  }

  // Dominated? A cheaper-or-equal knot that is at least as good makes this candidate redundant.
  // points_ is sorted by cost, so the relevant competitors are those with cost <= c.
  for (const Point& p : points_) {
    if (p.cost > c + kEpsilon) {
      break;
    }
    if (p.gain >= g - kEpsilon) {
      return;
    }
  }

  // Drop any existing knot this candidate dominates (cost >= c and gain <= g).
  points_.erase(
      std::remove_if(points_.begin(), points_.end(), [&](const Point& p) { return p.cost >= c - kEpsilon && p.gain <= g + kEpsilon; }),
      points_.end());

  // Insert in cost order, keeping the Pareto frontier sorted.
  auto it = std::lower_bound(points_.begin(), points_.end(), c, [](const Point& p, double cost) { return p.cost < cost; });
  points_.insert(it, Point{c, g});

  if (concave_) {
    restoreConcavity();
  }
}

void GainCostTable::restoreConcavity() {
  // Andrew's monotone chain for the upper concave hull of {(0,0)} ∪ points_.
  // Sweeps left-to-right (points_ sorted by cost); pops any knot whose cross product >= 0
  // (left turn / collinear), keeping only right turns so slopes strictly decrease.
  const Point origin{0.0, 0.0};
  auto cross = [](const Point& o, const Point& a, const Point& b) {
    return (a.cost - o.cost) * (b.gain - o.gain) - (a.gain - o.gain) * (b.cost - o.cost);
  };

  std::vector<Point> hull;
  hull.reserve(points_.size());
  for (const Point& p : points_) {
    while (!hull.empty()) {
      const Point& a = hull.size() >= 2 ? hull[hull.size() - 2] : origin;
      if (cross(a, hull.back(), p) >= 0.0) {
        hull.pop_back();
      } else {
        break;
      }
    }
    hull.push_back(p);
  }
  points_ = std::move(hull);
}

double GainCostTable::valueAt(double remaining_cost) const {
  if (points_.empty()) {
    return 0.0;
  }
  const double r = remaining_cost > 0.0 ? remaining_cost : 0.0;

  // Within the first segment, from the implicit origin (0,0) to the first knot.
  const Point& front = points_.front();
  if (r <= front.cost) {
    return front.cost > 0.0 ? front.gain * (r / front.cost) : front.gain;
  }

  // Within the observed range: interpolate between the bracketing knots.
  for (size_t i = 1; i < points_.size(); ++i) {
    if (r <= points_[i].cost) {
      const Point& a = points_[i - 1];
      const Point& b = points_[i];
      const double t = (r - a.cost) / (b.cost - a.cost);
      return a.gain + t * (b.gain - a.gain);
    }
  }

  // Beyond the last knot: extrapolate along the last segment's slope (origin->knot for a single
  // knot), floored at 0 to stay monotone.
  const Point& last = points_.back();
  const double prev_cost = points_.size() >= 2 ? points_[points_.size() - 2].cost : 0.0;
  const double prev_gain = points_.size() >= 2 ? points_[points_.size() - 2].gain : 0.0;
  double slope = (last.cost - prev_cost) > 0.0 ? (last.gain - prev_gain) / (last.cost - prev_cost) : 0.0;
  if (slope < 0.0) {
    slope = 0.0;
  }
  return last.gain + slope * (r - last.cost);
}

}  // namespace open_apero
