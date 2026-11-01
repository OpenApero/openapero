#pragma once

/**
 * @file gain_cost_table.h
 * @brief Episodic optimistic gain-vs-cost frontier used by the kTableGain selection objective.
 */

#include <memory>
#include <vector>

#include "open_apero_core/graph/path.h"

namespace open_apero {

/**
 * @brief Optimistic gain-vs-cost frontier (monotone non-decreasing; optionally concave).
 *
 * Each observed path contributes a (cost, gain) knot; dominated knots are dropped, so gain
 * strictly increases with cost. With concavity enabled the knots are reduced to the upper
 * concave hull of {(0,0)} and the knots. Episodic: reset() clears it.
 */
class GainCostTable {
 public:
  /// @brief Construct a frontier with optional concave-envelope reduction (off unless requested;
  /// PathPlanner sets it from its concave_envelope key, default true).
  explicit GainCostTable(bool concave = false) : concave_(concave) {}

  /// @brief One frontier knot: a cost and its best gain. No path is stored; paths are only valid
  /// for the planning iteration that produced them.
  struct Point {
    double cost;
    double gain;
  };

  /// @brief Enable/disable the optional concave-envelope reduction.
  void setConcave(bool on) { concave_ = on; }

  /// @brief Clear all knots; call at episode reset.
  void reset() { points_.clear(); }

  /// @brief Whether the frontier currently holds any knot.
  bool empty() const { return points_.empty(); }

  /// @brief The retained frontier knots, ordered by increasing cost (origin (0,0) is implicit).
  const std::vector<Point>& points() const { return points_; }

  /**
   * @brief Insert a candidate path's (cost, gain) into the frontier.
   *
   * Skips the candidate if it is dominated (a cheaper-or-equal knot is at least as good),
   * removes any existing knots it dominates, and — when concavity is enabled — restores the
   * upper concave hull. Non-positive cost or gain is ignored (covered by the implicit origin).
   */
  void insert(const Path& path);

  /**
   * @brief Optimistic gain reachable for a given remaining cost.
   * @param remaining_cost Remaining cost budget (clamped to >= 0).
   * @return Linear interpolation within the observed range (anchored at the origin); beyond the
   *         last knot, linear extrapolation along the last segment's slope (floored at 0), not a
   *         flat clamp to the best observed gain. Returns 0 when the frontier is empty.
   */
  double valueAt(double remaining_cost) const;

 private:
  /// @brief Reduce points_ to the upper concave hull of {(0,0)} ∪ points_.
  void restoreConcavity();

  std::vector<Point> points_;  ///< Sorted by increasing cost; origin (0,0) handled implicitly.
  bool concave_ = false;
};

}  // namespace open_apero
