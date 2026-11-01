#pragma once

/**
 * @file path.h
 * @brief Candidate paths of a search: shared steps, comparison, and what a path has visited.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <queue>
#include <vector>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/node.h"

namespace open_apero {

/// @brief Ordered sequence of node pointers forming a path.
using PathNodes = std::vector<Node*>;

/// @brief A node of a path and the step before it; paths share the steps they have in common.
struct PathStep {
  Node* node;
  const PathStep* parent;
};

/// @brief Owns every step a search builds; a step stays valid as long as its owner does.
class PathSteps {
 public:
  /** @brief The step @p node appended after @p parent; nullptr starts a path. */
  const PathStep* extend(const PathStep* parent, Node* node) {
    steps_.push_back({node, parent});
    return &steps_.back();
  }

  /** @brief Drop the step extend() returned last, which no path holds. */
  void discard() { steps_.pop_back(); }

  /** @brief The last step of a path through @p nodes in order. */
  const PathStep* chain(const PathNodes& nodes) {
    const PathStep* last = nullptr;
    for (Node* node : nodes) {
      last = extend(last, node);
    }
    return last;
  }

 private:
  // Paths and steps point at steps, so growing must never move one: std::deque's push_back doesn't, std::vector's would.
  std::deque<PathStep> steps_;
};

/**
 * @brief A candidate path: its ratio, gain and cost, and its last step.
 *
 * Comparable with operator> for the search heaps. Copying a path copies no nodes.
 */
struct Path {
  double ratio;
  double gain;
  double cost;
  const PathStep* last;  ///< nullptr for a path holding no nodes.

  Path(double r, double g, double c, const PathStep* l = nullptr) : ratio(r), gain(g), cost(c), last(l) {}

  bool isValid() const { return last != nullptr; }

  /** @brief The nodes from the first to the last. */
  PathNodes toNodes() const {
    PathNodes nodes;
    for (const PathStep* step = last; step != nullptr; step = step->parent) {
      nodes.push_back(step->node);
    }
    std::reverse(nodes.begin(), nodes.end());
    return nodes;
  }

  /**
   * @brief Compare paths by ratio, then gain, then cost (lower cost is better).
   * @param other The path to compare against.
   * @return True if this path is strictly greater (better) than the other.
   */
  bool operator>(const Path& other) const {
    if (!std::isnan(ratio) && !std::isnan(other.ratio)) {
      if (ratio > other.ratio + kEpsilon) return true;
      if (ratio < other.ratio - kEpsilon) return false;
    }

    if (!std::isnan(gain) && !std::isnan(other.gain)) {
      if (gain > other.gain + kEpsilon) return true;
      if (gain < other.gain - kEpsilon) return false;
    }

    if (!std::isnan(cost) && !std::isnan(other.cost)) {
      return cost < other.cost;
    }

    return false;  // If all values are NaN, consider them equal
  }
};

/// @brief Min-heap of paths ordered by the preference comparator: the WORST path is on
/// top, so a full beam can test/evict its weakest candidate in O(log size) via top()/pop().
using PathMinHeap = std::priority_queue<Path, std::vector<Path>, std::greater<Path>>;

/**
 * @brief Holds one path at a time, as entries on its nodes and anchors: the most recent path that visited them.
 *
 * load() replaces the previous path and gives the new one a fresh id. The entries live on the objects, so one search
 * runs at a time.
 */
class PathLedger {
 public:
  PathLedger() : path_id_(++last_path_id_) {}

  /**
   * @brief Load the path: its nodes, their anchors (none of a gain-suppressed node) and the edges it traversed from
   * its final node. Replaces the previously loaded path.
   */
  void load(const Path& path) {
    path_id_ = ++last_path_id_;
    const Node* const final_node = path.last->node;
    for (const PathStep* step = path.last; step != nullptr; step = step->parent) {
      recordNode(step->node);
      recordTraversedEdgeFromFinalNode(step, final_node);
    }
  }

  /** @brief Whether the loaded path already traversed the edge from its final node to @p node. */
  bool revisitsEdge(const Node* node) const { return node->most_recent_path_traversed_edge_from_final_node_to_this == path_id_; }

  /**
   * @brief What @p node is worth to the loaded path.
   *
   * 0 for a visited or gain-suppressed node. Uncorrelated: the node's own gain. Correlated: the gains of
   * its anchors the path has not visited.
   */
  double marginalGain(const Node* node, bool is_gain_correlated) const {
    if (node->most_recent_path_visited_this == path_id_ || node->is_gain_suppressed) {
      return 0.0;
    }
    if (!is_gain_correlated) {
      return node->getSingleNodeGain();
    }
    double marginal_gain = 0.0;
    for (const auto& anchor : node->gain_anchors) {
      if (anchor->most_recent_path_collected_this != path_id_) {
        marginal_gain += anchor->gain;
      }
    }
    return marginal_gain;
  }

  /**
   * @brief The gain of a path through @p nodes, loading them in order: what each node adds to the nodes before it; the
   * first adds nothing.
   */
  double pathGain(const PathNodes& nodes, bool is_gain_correlated) {
    path_id_ = ++last_path_id_;
    double path_gain = 0.0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (i > 0) {
        path_gain = path_gain + marginalGain(nodes[i], is_gain_correlated);
      }
      recordNode(nodes[i]);
    }
    return path_gain;
  }

 private:
  void recordNode(Node* node) {
    node->most_recent_path_visited_this = path_id_;
    if (node->is_gain_suppressed) {
      return;
    }
    for (const auto& anchor : node->gain_anchors) {
      anchor->most_recent_path_collected_this = path_id_;
    }
  }

  /** @brief If @p step leaves @p final_node, record the edge from there to the step's node. */
  void recordTraversedEdgeFromFinalNode(const PathStep* step, const Node* final_node) {
    if (step->parent != nullptr && step->parent->node == final_node) {
      step->node->most_recent_path_traversed_edge_from_final_node_to_this = path_id_;
    }
  }

  inline static std::size_t last_path_id_ = 0;  ///< Shared by every PathLedger, so no earlier path's id reads as current.
  std::size_t path_id_;
};

}  // namespace open_apero
