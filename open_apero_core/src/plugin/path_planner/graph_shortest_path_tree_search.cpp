// Positive edge costs allow Dijkstra in place of the paper's Bellman-Ford.

#include "open_apero_core/plugin/path_planner/graph_shortest_path_tree_search.h"

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "open_apero_core/graph/path.h"

namespace open_apero {
namespace path_planner {

namespace {

std::vector<NodeId> getTopCandidates(GraphType* graph, double top_node_percentage, bool consider_frontier) {
  double max_gain = std::numeric_limits<double>::lowest();
  double min_gain = std::numeric_limits<double>::max();

  auto vertices = boost::vertices(*graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    auto gain = (*graph)[*it]->getSingleNodeGain();
    max_gain = std::max(max_gain, gain);
    min_gain = std::min(min_gain, gain);
  }

  auto gain_range = max_gain - min_gain;
  auto gain_threshold = max_gain - top_node_percentage * gain_range - 1e-4;

  std::vector<NodeId> goal_candidates;
  for (auto it = vertices.first; it != vertices.second; ++it) {
    auto node = (*graph)[*it].get();
    if (node->getSingleNodeGain() > gain_threshold) {
      goal_candidates.push_back(*it);
    } else if (consider_frontier && node->is_at_frontier) {
      goal_candidates.push_back(*it);
    }
  }

  return goal_candidates;
}

}  // namespace

PluginRegistry::Registration<GraphShortestPathTreeSearch> GraphShortestPathTreeSearch::registration("GraphShortestPathTreeSearch");

GraphShortestPathTreeSearch::GraphShortestPathTreeSearch(AgentInterface& agent) : PathPlanner(agent) {}

void GraphShortestPathTreeSearch::setupFromParamMap(Configurable::ParamMap* param_map) {
  PathPlanner::setupFromParamMap(param_map);

  setParam<double>(param_map, "top_node_percentage", &p_top_node_percentage_, 1.0);

  rng_.seed(agent_.getRandomSeed(), (*param_map)["param_namespace"]);
}

std::pair<std::vector<Node*>, double> GraphShortestPathTreeSearch::planPath(Node* start_node, GraphType* graph,
                                                                            Node* required_terminal_node) {
  if (graph == nullptr) {
    // Without a graph the only path is the start node itself, which reaches no destination.
    if (required_terminal_node != nullptr) {
      return std::pair<std::vector<Node*>, double>({}, kDoubleMin);
    }
    return std::pair<std::vector<Node*>, double>({start_node}, 0.0);
  }

  suppressGains(*graph, start_node);

  std::unordered_map<NodeId, double> distances;
  std::unordered_map<NodeId, NodeId> predecessors;

  NodeId start_node_id = getNodeId(start_node, graph);
  if (start_node_id == kInvalidNodeId) {
    throw std::runtime_error("Start node not found in graph");
  }
  findShortestPaths(start_node_id, distances, predecessors, graph);

  auto edges = boost::edges(*graph);
  for (auto e_it = edges.first; e_it != edges.second; ++e_it) {
    (*graph)[*e_it]->is_considered = false;
  }

  if (boost::num_vertices(*graph) == 1) {
    if (required_terminal_node != nullptr) {
      return std::pair<std::vector<Node*>, double>({}, kDoubleMin);
    }
    return std::pair<std::vector<Node*>, double>({start_node}, 0.0);
  }

  // A destination replaces the usual high-gain candidates rather than filtering them.
  std::vector<NodeId> top_candidate_goals;
  if (required_terminal_node != nullptr) {
    const NodeId terminal_id = getNodeId(required_terminal_node, graph);
    if (terminal_id == kInvalidNodeId) {
      return std::pair<std::vector<Node*>, double>({}, kDoubleMin);
    }
    top_candidate_goals = {terminal_id};
  } else {
    top_candidate_goals = getTopCandidates(graph, p_top_node_percentage_, true);
  }

  PathSteps steps;
  PathLedger ledger;
  Path best(kDoubleMin, kDoubleMin, kDoubleMax);
  double best_score = kDoubleMin;

  for (NodeId goal_id : top_candidate_goals) {
    if (goal_id == start_node_id) {
      continue;
    }

    double path_cost = distances[goal_id];
    if (path_cost > agent_.getCurrentCostBudget()) {
      continue;
    }

    PathNodes path_nodes;
    for (NodeId v = goal_id; v != start_node_id; v = predecessors[v]) {
      if (predecessors[v] == v) {
        path_nodes.clear();  // No path found
        break;
      }

      EdgeId edge = boost::edge(predecessors[v], v, *graph).first;
      (*graph)[edge]->is_considered = true;

      path_nodes.push_back((*graph)[v].get());
    }

    if (!path_nodes.empty()) {
      path_nodes.push_back(start_node);
      std::reverse(path_nodes.begin(), path_nodes.end());
    } else {
      continue;
    }

    if (path_nodes.front() != start_node) {
      throw std::runtime_error("GraphShortestPathTreeSearch: Path does not start with start node");
    }

    const double path_gain = ledger.pathGain(path_nodes, p_is_gain_correlated_);
    const double path_ratio = path_gain / path_cost;

    updateBest(Path(path_ratio, path_gain, path_cost, steps.chain(path_nodes)), best, best_score, required_terminal_node);
  }

  // Falls back to a neighbor when every scored candidate is invalid, unless a destination was asked for.
  const std::vector<Node*> best_path_nodes =
      concludeSearch(best, start_node, required_terminal_node, getAdjacentNodes(start_node_id, graph), &rng_.engine, &best_score);

  return {best_path_nodes, best_score};
}

}  // namespace path_planner
}  // namespace open_apero
