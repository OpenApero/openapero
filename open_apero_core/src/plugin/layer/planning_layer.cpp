#include "open_apero_core/plugin/layer/planning_layer.h"

#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeinfo>
#include <utility>
#include <vector>

#include <boost/core/demangle.hpp>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/logging/scoped_timer.h"
#include "open_apero_core/plugin/generator/graph_generator.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"

namespace open_apero {
namespace layer {

PluginRegistry::Registration<PlanningLayer> PlanningLayer::registration("PlanningLayer");

PlanningLayer::PlanningLayer(AgentInterface& agent) : Layer(agent) {
  metrics_ = std::make_unique<PlanningLayerMetrics>();
}

void PlanningLayer::setupFromParamMap(Configurable::ParamMap* param_map) {
  int replan_num_steps;
  setParam<int>(param_map, "replan_num_steps", &replan_num_steps, std::numeric_limits<int>::max());
  p_replan_num_steps_ = (replan_num_steps < 1) ? std::numeric_limits<int>::max() : replan_num_steps;

  setParam<int>(param_map, "max_expansions", &p_max_expansions_, 50);

  Layer::setupFromParamMap(param_map);

  if (generator_->as<GraphGenerator>() == nullptr) {
    throw std::runtime_error("PlanningLayer needs a GraphGenerator, got " + boost::core::demangle(typeid(*generator_).name()));
  }
}

void PlanningLayer::resetEpisode() {
  Layer::resetEpisode();
  graph_ = nullptr;
  current_node_ = nullptr;
}

void PlanningLayer::onEpisodeStart(const RobotState& init_state) {
  Node* start_node = nullptr;
  std::tie(graph_, start_node) = generator_->as<GraphGenerator>()->generateInitNode(init_state);
  current_node_ = start_node;
}

void PlanningLayer::onStateReached(const RobotState& /*state*/, const Edge* executed_edge, Node* active_node) {
  if (current_node_ != nullptr) {
    if (executed_edge) {
      getMetrics().real_accumulated_gain = evaluator_->getRealAccumulatedGain();
    }

    Node* previous = current_node_;

    auto* edge_generator = generator_->as<GraphGenerator>();
    const bool active_node_in_structure = active_node != nullptr && edge_generator->containsNode(active_node);

    // Every start-node update below is charged to StartNodeUpdate: it runs outside the update() UpdateTime scope.
    if (active_node_in_structure) {
      current_node_ = active_node;
      evaluator_->onNodeReached(current_node_, executed_edge);
      if (previous != current_node_) {
        ScopedTimer t(getMetrics().start_node_update_s, agent_.isLoggingPerformance());
        edge_generator->setNewStartNode(previous, current_node_);
      }
    } else if (executed_edge != nullptr) {
      {
        ScopedTimer t(getMetrics().start_node_update_s, agent_.isLoggingPerformance());
        current_node_ = edge_generator->insertIntermediateNodeAtCurrentState(previous, *executed_edge);
      }
      if (current_node_ == nullptr) {
        throw std::logic_error("PlanningLayer: generator returned nullptr from insertIntermediateNodeAtCurrentState");
      }
      evaluator_->onNodeReached(current_node_, executed_edge);
      // Insertion also moves the start node and cleans up stale nodes.
    } else {
      // First tick: no edge yet, the robot is on current_node_.
      evaluator_->onNodeReached(current_node_, executed_edge);
    }
  }

  getMetrics().num_nodes_after_new_start = getNodeCount();
}

bool PlanningLayer::splitActiveEdge(Node* active_node, Edge* edge, int waypoint_idx, Node** intermediate_out, Edge** finished_edge_out) {
  std::tie(*intermediate_out, *finished_edge_out) =
      generator_->as<GraphGenerator>()->insertIntermediateNode(current_node_, active_node, edge, waypoint_idx);
  return true;
}

bool PlanningLayer::currentNodeHasOutgoingEdges() const {
  if (graph_ == nullptr) {
    return true;
  }
  const NodeId current_node_id = getNodeId(current_node_, graph_);
  if (current_node_id == kInvalidNodeId) {
    throw std::logic_error("PlanningLayer: current node is not a vertex of the planning graph");
  }
  return boost::out_degree(current_node_id, *graph_) > 0;
}

int PlanningLayer::getNodeCount() const {
  if (current_node_ == nullptr) {
    return 0;
  }
  return static_cast<int>(generator_->as<GraphGenerator>()->getAllNodes(current_node_).size());
}

void PlanningLayer::warmUp() {
  generator_->generateNewNode(current_node_);
  evaluator_->evaluate({current_node_});

  Edge fake_edge;
  fake_edge.trajectory.push_back(current_node_->robot_state);
  generator_->as<GraphGenerator>()->computeCost(&fake_edge);

  update();
  // The search's own fallback throws for a start node without out-edges, which planPath() declines instead.
  if (currentNodeHasOutgoingEdges()) {
    path_planner_->planPath(current_node_, graph_);
  }
  // The warm-up search remembered a path into this graph, which the first episode replaces.
  path_planner_->resetEpisode();
}

std::optional<double> PlanningLayer::realAccumulatedGain() const {
  if (!evaluator_) {
    return std::nullopt;
  }
  return evaluator_->getRealAccumulatedGain();
}

std::string PlanningLayer::cycleSummary() const {
  const PlanningLayerMetrics& m = getMetrics();
  const int net_change = m.num_nodes_after_expansion - m.num_nodes_after_new_start;
  std::stringstream ss;
  ss << "[" << m.num_nodes_after_new_start << "]"                                                //
     << " ---update and expansion: " << std::showpos << net_change << std::noshowpos << "---> "  //
     << "[" << m.num_nodes_after_expansion << "] nodes at selection";
  return ss.str();
}

void PlanningLayer::getVisualization(Node* selected_node, VisualizationMarkers* markers) {
  if (current_node_ == nullptr) {
    return;
  }
  if (selected_node) {
    // The next node is usually the unevaluated split at the replanning horizon: show the first
    // evaluated node the path heads to (selected_path_[0] is current_node_).
    Node* shown_node = selected_node;
    for (size_t i = 1; i < selected_path_.size(); ++i) {
      if (selected_path_[i]->info) {
        shown_node = selected_path_[i];
        break;
      }
    }
    evaluator_->getVisualization(markers, shown_node);
  }
  generator_->as<GraphGenerator>()->getVisualization(current_node_, markers);
}

void PlanningLayer::getPathVisualization(Node* selected_node, VisualizationMarkers* markers) {
  // A selected node means the agent took this layer's path this tick, which is the only
  // state in which selected_path_ is known to still be alive.
  if (current_node_ == nullptr || selected_node == nullptr) {
    return;
  }

  evaluator_->getPathVisualization(markers, selected_path_);
}

void PlanningLayer::expand() {
  if (current_node_ != nullptr) {
    PlanningLayerMetrics& m = getMetrics();
    const bool log_perf = agent_.isLoggingPerformance();
    ScopedTimer expand_timer(m.expand_s, log_perf);

    for (int i = 0; i < p_max_expansions_; ++i) {
      std::vector<Node*> new_node_cluster;
      {
        ScopedTimer t(m.generate_node_s, log_perf);
        new_node_cluster = generator_->generateNewNode(current_node_);
      }

      auto* edge_generator = generator_->as<GraphGenerator>();
      bool node_connected = false;
      std::vector<Edge*> new_edges;
      {
        ScopedTimer t(m.generate_edge_s, log_perf);
        if (!new_node_cluster.empty()) {
          node_connected = edge_generator->generateNewEdges(new_node_cluster, new_edges);
        }
      }

      if (node_connected) {
        ScopedTimer t(m.evaluate_node_s, log_perf);
        evaluator_->evaluate(new_node_cluster);
      }

      if (node_connected) {
        for (auto new_edge : new_edges) {
          edge_generator->computeCost(new_edge);
        }
      }

      m.num_expansions += 1;
    }

    if (!currentNodeHasOutgoingEdges()) {
      generator_->as<GraphGenerator>()->connectStartNode(current_node_);
    }
  }

  getMetrics().num_nodes_after_expansion = getNodeCount();
}

void PlanningLayer::update() {
  if (current_node_ == nullptr) return;
  PlanningLayerMetrics& m = getMetrics();
  const bool log_perf = agent_.isLoggingPerformance();
  ScopedTimer update_timer(m.update_s, log_perf);

  {
    ScopedTimer t(m.generator_update_s, log_perf);
    generator_->as<GraphGenerator>()->updateNodeEdge(current_node_);
  }

  {
    ScopedTimer t(m.evaluator_update_s, log_perf);
    for (const auto& cluster : generator_->getAllNodeClusters()) {
      evaluator_->updateEvaluation(cluster, current_node_);
    }
  }
}

bool PlanningLayer::planPath(const RobotState& /*current_state*/, Map* map, RobotModel* robot, std::vector<Node*>* planned_path,
                             std::vector<Edge*>* planned_edges) {
  if (current_node_ == nullptr) {
    return false;
  }

  PlanningLayerMetrics& m = getMetrics();
  ScopedTimer plan_path_timer(m.plan_path_s, agent_.isLoggingPerformance());
  planned_path->clear();
  planned_edges->clear();
  // Cleared up front so every non-success exit leaves nothing for the visualization.
  selected_path_.clear();

  m.num_nodes_at_selection = getNodeCount();

  auto* edge_generator = generator_->as<GraphGenerator>();

  auto all_nodes = edge_generator->getAllNodes(current_node_);
  // current_node_ is where the robot stands, not a destination, and its own flag may be false.
  int traversable_count = 0;
  for (const auto* node : all_nodes) {
    if (node != current_node_ && node->is_traversable) {
      ++traversable_count;
    }
  }
  if (traversable_count == 0) {
    agent_.printInfo("PlanningLayer planPath declined: no traversable node besides the start node (" + std::to_string(all_nodes.size()) +
                     " nodes)");
    return false;
  }

  if (!currentNodeHasOutgoingEdges()) {
    agent_.printInfo("PlanningLayer planPath declined: start node has no out-edges (" + std::to_string(all_nodes.size()) + " nodes)");
    return false;
  }

  std::tie(*planned_path, m.planned_value) = path_planner_->planPath(current_node_, graph_);
  if (planned_path->size() < 2) {
    planned_path->clear();
    agent_.printInfo("PlanningLayer planPath declined: path planner returned nothing");
    return false;
  }

  for (size_t i = 0; i + 1 < planned_path->size(); ++i) {
    Edge* edge = getEdge((*planned_path)[i], (*planned_path)[i + 1], graph_);
    if (!edge) {
      planned_path->clear();
      planned_edges->clear();
      return false;
    }
    planned_edges->push_back(edge);
  }

  // Refine before slicing so the robot model sees the full horizon.
  Edge* first_edge = planned_edges->front();
  if (robot) {
    // The split index is also the number of committed states.
    robot->refineBeforeExecute(first_edge->trajectory, p_replan_num_steps_);

    // Refinement may change a graph-validated trajectory: re-check before mutating the graph.
    if (!robot->isTrajectoryTraversable(first_edge->trajectory, map)) {
      first_edge->is_traversable = false;

      bool logged_invalid_state = false;
      for (size_t i = 0; i < first_edge->trajectory.size(); ++i) {
        const RobotState& state = first_edge->trajectory[i];
        if (!robot->isStateTraversable(state, map)) {
          std::ostringstream warning;
          warning << "[PlanningLayer:" << instanceName() << "] Refined trajectory rejected before execution: waypoint_idx=" << i << "/"
                  << first_edge->trajectory.size() << " sensor_pos=[" << state.getSensorPosition().transpose() << "] sensor_yaw_pitch=["
                  << yawPitchFromQuaternion(state.getSensorOrientation()).transpose() << "]";
          agent_.printWarning(warning.str());
          logged_invalid_state = true;
          break;
        }
      }
      if (!logged_invalid_state) {
        std::ostringstream warning;
        warning << "[PlanningLayer:" << instanceName()
                << "] Refined trajectory rejected before execution by trajectory-level validation; no individual invalid state found"
                << " (trajectory_size=" << first_edge->trajectory.size() << ").";
        agent_.printWarning(warning.str());
      }

      // The graph retains ownership of the refined edge. Marking it
      // untraversable lets normal graph maintenance prune it next cycle.
      planned_path->clear();
      planned_edges->clear();
      return false;
    }
  }

  // Split long edges at the replanning horizon.
  Node* original_next_node = (*planned_path)[1];
  Node* new_intermediate = nullptr;
  Edge* new_first_edge = nullptr;
  std::tie(new_intermediate, new_first_edge) =
      edge_generator->insertIntermediateNode(current_node_, original_next_node, first_edge, p_replan_num_steps_);
  if (new_intermediate != original_next_node) {
    planned_path->insert(planned_path->begin() + 1, new_intermediate);
    (*planned_edges)[0] = new_first_edge;
    Edge* remainder_edge = getEdge(new_intermediate, original_next_node, graph_);
    if (!remainder_edge) {
      throw std::runtime_error("PlanningLayer::planPath: no remainder edge after inserting intermediate node.");
    }
    planned_edges->insert(planned_edges->begin() + 1, remainder_edge);
  }

  // Account for the refined or split trajectory.
  edge_generator->computeCost(planned_edges->front());

  selected_path_ = *planned_path;
  return true;
}

}  // namespace layer
}  // namespace open_apero
