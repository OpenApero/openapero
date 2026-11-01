#include "open_apero_core/plugin/path_planner/path_planner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/gain/gain_suppression.h"

namespace open_apero {

void PathPlanner::setupFromParamMap(Configurable::ParamMap* param_map) {
  setParam<bool>(param_map, "is_gain_correlated", &p_is_gain_correlated_, false);
  setParam<double>(param_map, "gain_suppression_radius", &p_gain_suppression_radius_, 0.0);

  // Without a default an omitted key is reported as missing rather than unparseable.
  std::string objective_name;
  if (objective_required_) {
    setParam<std::string>(param_map, "objective", &objective_name);
  } else {
    setParam<std::string>(param_map, "objective", &objective_name, toString(p_objective_));
  }
  p_objective_ = objectiveFromString(objective_name);

  setParam<bool>(param_map, "concave_envelope", &p_concave_envelope_, true);
  gain_cost_table_.setConcave(p_concave_envelope_);
}

bool PathPlanner::checkParamsValid(std::string* error_message) {
  if (!std::isfinite(p_gain_suppression_radius_) || p_gain_suppression_radius_ < 0.0) {
    *error_message = "gain_suppression_radius expected finite and >= 0.0";
    return false;
  }
  return Plugin::checkParamsValid(error_message);
}

namespace {

// Whether the objective's score reads Node::is_at_frontier; only these may break ties on it.
bool objectiveReadsFrontier(Objective objective) {
  return objective == Objective::kExpectedGain || objective == Objective::kTableGain;
}

}  // namespace

std::vector<Node*> PathPlanner::concludeSearch(const Path& best, Node* start_node, Node* required_terminal_node,
                                               const std::vector<Node*>& fallback_candidates, std::mt19937* rng, double* score) const {
  if (best.isValid()) {
    return best.toNodes();
  }

  // Nothing was selected. What that means depends entirely on what was asked.
  if (required_terminal_node != nullptr) {
    return {};
  }

  if (fallback_candidates.empty()) {
    throw std::runtime_error("PathPlanner: no path was selected and no fallback step exists.");
  }

  // Seeded runs depend on this exact distribution.
  std::uniform_int_distribution<> distribution(0, static_cast<int>(fallback_candidates.size()) - 1);
  Node* const step = fallback_candidates[static_cast<std::size_t>(distribution(*rng))];
  *score = step->getSingleNodeGain();
  return {start_node, step};
}

bool PathPlanner::updateBest(Path candidate, Path& best, double& best_score, Node* required_terminal_node) {
  if (!candidate.isValid()) {
    return false;
  }

  // Rejected before scoring so kTableGain never inserts a disallowed candidate into its table.
  if (required_terminal_node != nullptr && candidate.last->node != required_terminal_node) {
    return false;
  }

  const double budget = agent_.getCurrentCostBudget();
  const bool at_frontier = candidate.last->node->is_at_frontier;

  double score;
  switch (p_objective_) {
    case Objective::kPathGain:
      score = candidate.gain;
      break;

    case Objective::kPathRatio:
      score = candidate.ratio;
      break;

    case Objective::kExpectedGain:
      score = at_frontier ? candidate.ratio * budget : candidate.gain;
      break;

    case Objective::kTableGain:
      gain_cost_table_.insert(candidate);
      score = at_frontier ? candidate.gain + gain_cost_table_.valueAt(std::max(0.0, budget - candidate.cost)) : candidate.gain;
      break;

    default:
      throw std::runtime_error("Invalid objective type.");
  }

  // Exact comparison: a tolerance would let the tie-break install a strictly lower-ranked candidate.
  if (score > best_score) {
    best_score = score;
    best = candidate;
    return true;
  }
  if (!best.isValid() || score < best_score) {
    return false;
  }

  // Exact tie: prefer a frontier end where the objective reads it, else Path::operator> (ratio, gain,
  // lower cost), so enumeration order never decides and a zero-gain graph heads for the frontier, not a
  // min-cost cycle.
  const bool frontier_decides = objectiveReadsFrontier(p_objective_) && at_frontier != best.last->node->is_at_frontier;
  const bool wins_tie = frontier_decides ? at_frontier : candidate > best;
  if (wins_tie) {
    best_score = score;
    best = candidate;
  }
  return wins_tie;
}

void PathPlanner::suppressGains(const std::vector<Node*>& nodes, const Node* start_node) const {
  suppressNonMaximumClusters(nodes, start_node, p_gain_suppression_radius_);
}

void PathPlanner::suppressGains(const GraphType& graph, const Node* start_node) const {
  std::vector<Node*> nodes;
  nodes.reserve(boost::num_vertices(graph));
  for (auto vertices = boost::vertices(graph); vertices.first != vertices.second; ++vertices.first) {
    nodes.push_back(graph[*vertices.first].get());
  }
  suppressGains(nodes, start_node);
}

}  // namespace open_apero
