// What depth-wise and node-wise beam search share: the parameters, the receding-horizon reuse of the
// previous path (check_previous_path) and the frame of planPath(); each search supplies its beam rounds.

#include "open_apero_core/plugin/path_planner/graph_beam_search.h"

#include <string>
#include <utility>
#include <vector>

namespace open_apero {
namespace path_planner {

GraphBeamSearch::GraphBeamSearch(AgentInterface& agent, std::string raw_seed_label)
    : PathPlanner(agent), raw_seed_label_(std::move(raw_seed_label)) {}

void GraphBeamSearch::setupFromParamMap(Configurable::ParamMap* param_map) {
  PathPlanner::setupFromParamMap(param_map);

  setParam<int>(param_map, "beam_depth", &p_beam_depth_, 10);
  setParam<int>(param_map, "beam_width", &p_beam_width_, 1);

  setParam<bool>(param_map, "check_previous_path", &p_check_previous_path_, true);

  rng_.seed(agent_.getRandomSeed(), (*param_map)["param_namespace"]);
}

bool GraphBeamSearch::checkParamsValid(std::string* error_message) {
  if (p_beam_width_ < 1) {
    *error_message = "beam_width expected >= 1";
    return false;
  }
  if (p_beam_depth_ < 1) {
    *error_message = "beam_depth expected >= 1";
    return false;
  }
  return PathPlanner::checkParamsValid(error_message);
}

void GraphBeamSearch::setupRaw(Objective obj, int beam_depth, int beam_width, bool check_previous_path, bool is_gain_correlated,
                               double gain_suppression_radius) {
  p_objective_ = obj;
  p_beam_depth_ = beam_depth;
  p_beam_width_ = beam_width;
  p_check_previous_path_ = check_previous_path;
  p_is_gain_correlated_ = is_gain_correlated;
  p_gain_suppression_radius_ = gain_suppression_radius;
  rng_.seed(agent_.getRandomSeed(), raw_seed_label_);
}

std::pair<std::vector<Node*>, double> GraphBeamSearch::planPath(Node* start_node, GraphType* graph, Node* required_terminal_node) {
  if (graph == nullptr) {
    // Without a graph the only path is the start node itself, which reaches no destination.
    if (required_terminal_node != nullptr) {
      return std::pair<std::vector<Node*>, double>({}, kDoubleMin);
    }
    return std::pair<std::vector<Node*>, double>({start_node}, 0.0);
  }

  suppressGains(*graph, start_node);

  NodeId start_node_id = getNodeId(start_node, graph);
  PathSteps steps;
  PathLedger ledger;
  const Path start_path(0, 0, 0, steps.extend(nullptr, start_node));

  Path best(kDoubleMin, kDoubleMin, kDoubleMax);  // invalid (no nodes) until a candidate wins
  double best_score = kDoubleMin;

  auto edges = boost::edges(*graph);
  for (auto e_it = edges.first; e_it != edges.second; ++e_it) {
    (*graph)[*e_it]->is_considered = false;
  }

  // Reuse and extend the surviving tail of the previous path.
  if (p_check_previous_path_ && !previous_path_.empty()) {
    Path path = start_path;

    // Skip the previous first step once the robot has reached it.
    int start_idx = (start_node == previous_path_.at(1)) ? 2 : 1;
    for (int i = start_idx; i < static_cast<int>(previous_path_.size()); ++i) {
      Node* new_node = previous_path_[i];
      Edge* new_edge = getEdge(path.last->node, new_node, graph);

      if (!new_edge) {
        break;
      }

      double new_cost = path.cost + new_edge->cost;
      if (new_cost > agent_.getCurrentCostBudget()) {
        break;
      }

      ledger.load(path);
      double new_gain = path.gain + ledger.marginalGain(new_node, p_is_gain_correlated_);
      double new_ratio = new_gain / new_cost;
      path = Path(new_ratio, new_gain, new_cost, steps.extend(path.last, new_node));

      updateBest(path, best, best_score, required_terminal_node);
    }

    auto node_id = getNodeId(path.last->node, graph);

    ledger.load(path);
    auto neighbors = boost::adjacent_vertices(node_id, *graph);

    for (auto neighbor_it = neighbors.first; neighbor_it != neighbors.second; ++neighbor_it) {
      auto new_node_id = *neighbor_it;
      auto new_node = (*graph)[new_node_id].get();

      if (ledger.revisitsEdge(new_node)) {
        continue;
      }

      EdgeId edge_id = boost::edge(node_id, new_node_id, *graph).first;
      auto edge = (*graph)[edge_id];
      double new_cost = path.cost + edge->cost;

      if (new_cost > agent_.getCurrentCostBudget()) {
        continue;
      }

      double new_gain = path.gain + ledger.marginalGain(new_node, p_is_gain_correlated_);
      double new_ratio = new_gain / new_cost;

      if (!updateBest(Path(new_ratio, new_gain, new_cost, steps.extend(path.last, new_node)), best, best_score, required_terminal_node)) {
        steps.discard();
      }
    }
  }

  searchBeams(start_path, graph, required_terminal_node, steps, ledger, best, best_score);

  // Keeps moving when every scored candidate was pruned, unless a destination was asked for.
  std::vector<Node*> best_path_nodes =
      concludeSearch(best, start_node, required_terminal_node, getAdjacentNodes(start_node_id, graph), &rng_.engine, &best_score);

  // previous_path_ seeds the next exploration sweep, and a path chosen to reach one destination
  // is not a horizon to extend.
  if (required_terminal_node == nullptr) {
    previous_path_ = best_path_nodes;
  }
  return {best_path_nodes, best_score};
}

}  // namespace path_planner
}  // namespace open_apero
