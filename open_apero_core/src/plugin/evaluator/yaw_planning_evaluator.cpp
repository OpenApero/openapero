#include "open_apero_core/plugin/evaluator/yaw_planning_evaluator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"
#include "open_apero_core/plugin/generator/graph_generator.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/plugin_factory.h"

namespace open_apero {
namespace evaluator {

// YawPlanningEvaluator
YawPlanningEvaluator::YawPlanningEvaluator(AgentInterface& agent) : Evaluator(agent) {}

void YawPlanningEvaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  setParam<double>(param_map, "update_gain", &p_update_gain_, 0.0);

  // Create following evaluator
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_evaluator_args", &args, param_ns + "/following_evaluator");
  following_evaluator_ = agent_.getFactory().createPlugin<Evaluator>(args, agent_, verbose_plugins_);
  sensor_evaluator_ = dynamic_cast<SimulatedSensorEvaluator*>(following_evaluator_.get());
  if (sensor_evaluator_ == nullptr) {
    throw std::runtime_error("YawPlanningEvaluator: the following evaluator at '" + args + "' must be a SimulatedSensorEvaluator");
  }

  Evaluator::setupFromParamMap(param_map);

  rng_.seed(agent_.getRandomSeed(), param_ns);
}

bool YawPlanningEvaluator::evaluateUnobserved(const NodeClusterPtrs& cluster) {
  if (cluster.size() > 1) {
    throw std::runtime_error(
        "YawPlanningEvaluator::evaluate called with more than one node; it supports only tree-based generators, which "
        "generate one node at a time.");
  }
  auto node = cluster.front();

  // Init
  node->info = std::make_unique<YawPlanningInfo>();
  YawPlanningInfo* info = static_cast<YawPlanningInfo*>(node->info.get());
  info->active_orientation = 0;

  // Sample all directions
  NodeClusterPtrs orientations_to_evaluate;
  for (int i = 0; i < static_cast<int>(node->candidate_orientations.size()); ++i) {
    auto orientation = node->shallowCopy();
    // The generator refuses an orientation that would make the node a pose twin of its
    // parent. The copy stays in info->orientations -- indices must line up with
    // candidate_orientations -- but an orientation that was never applied gets no gain.
    const bool applied = setNodeOrientation(orientation.get(), node->candidate_orientations.at(i));
    orientation->is_traversable = node->is_candidate_traversable.at(i);

    info->orientations.push_back(std::move(orientation));
    info->visited_orientation.push_back(false);

    if (applied && node->is_candidate_traversable.at(i)) {
      orientations_to_evaluate.push_back(info->orientations.back().get());
    }
  }

  following_evaluator_->evaluate(orientations_to_evaluate);

  // Apply best segment to the original one, store rest in info
  return setBestYaw(node);
}

bool YawPlanningEvaluator::setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation) {
  return getLayer()->getGenerator().as<GraphGenerator>()->setNodeOrientation(node, orientation);
}

bool YawPlanningEvaluator::setBestYaw(Node* node) {
  // Select the best yaw out of all current orientations.
  YawPlanningInfo* info = static_cast<YawPlanningInfo*>(node->info.get());

  if (info->orientations.size() == 0) {
    agent_.printWarning("[YawPlanningEvaluator::setBestYaw] No orientations to choose from!");
    return false;
  }

  // The generator refuses an orientation that would make the node a pose twin of its
  // parent or of a coincident child; drop a refused candidate and re-pick until one
  // applies or none remain.
  std::vector<bool> refused(info->orientations.size(), false);

  while (true) {
    double max_gain = std::numeric_limits<double>::lowest();
    std::vector<int> best_indices;
    std::vector<int> feasible_indices;
    for (int i = 0; i < static_cast<int>(info->orientations.size()); ++i) {
      if (!node->is_candidate_traversable[i]) {
        // unsafe
        continue;
      }

      if (refused[i]) {
        // the generator cannot apply this orientation
        continue;
      }

      // i-th orientation is feasible
      feasible_indices.push_back(i);

      if (info->visited_orientation[i]) {
        // the orientation has been selected
        continue;
      }

      double gain = info->orientations[i]->getSingleNodeGain();
      if (std::isnan(gain)) {
        continue;
      }

      if (gain > max_gain + 1e-4) {
        max_gain = gain;
        best_indices = {i};
      } else if (gain > max_gain - 1e-4) {
        best_indices.push_back(i);
      }
    }

    if (feasible_indices.empty()) {
      // no feasible yaw; the node keeps its current orientation
      return false;
    }

    int best_index;
    if (!best_indices.empty()) {
      std::shuffle(best_indices.begin(), best_indices.end(), rng_.engine);
      best_index = best_indices.front();
    } else {
      // get frontier node from feasible indices
      std::vector<int> frontier_indices;
      for (auto id : feasible_indices) {
        if (info->orientations[id]->is_at_frontier) {
          frontier_indices.push_back(id);
        }
      }

      std::vector<int> random_indices;
      if (!frontier_indices.empty()) {
        random_indices = frontier_indices;
      } else {
        random_indices = feasible_indices;
      }

      std::shuffle(random_indices.begin(), random_indices.end(), rng_.engine);
      best_index = random_indices.front();
    }

    // The winner's full orientation, not just its yaw: a pitched candidate grid would
    // otherwise be flattened back to level here.
    const Eigen::Quaterniond best_orientation = info->orientations[best_index]->robot_state.getSensorOrientation();
    if (!setNodeOrientation(node, best_orientation)) {
      refused[best_index] = true;
      continue;
    }
    info->active_orientation = best_index;

    node->gain_anchors = info->orientations[best_index]->gain_anchors;
    node->is_at_frontier = info->orientations[best_index]->is_at_frontier;

    return true;
  }
}

void YawPlanningEvaluator::onNodeReached(Node* reached_node, const Edge* traversed_edge) {
  YawPlanningInfo* info = reached_node->info ? reached_node->info->as<YawPlanningInfo>() : nullptr;
  if (info) {
    info->visited_orientation[info->active_orientation] = true;
    following_evaluator_->onNodeReached(info->orientations[info->active_orientation].get(), traversed_edge);
  }
}

bool YawPlanningEvaluator::reevaluateUnobserved(const NodeClusterPtrs& cluster, const Node* current_node) {
  if (cluster.size() > 1) {
    throw std::runtime_error(
        "YawPlanningEvaluator::updateEvaluation called with more than one node; it supports only tree-based generators, "
        "which generate one node at a time.");
  }
  auto node = cluster.front();

  // Visits are tracked per orientation (visited_orientation): the node itself is never observed.
  YawPlanningInfo* info = node->info ? node->info->as<YawPlanningInfo>() : nullptr;
  if (!info) {
    return true;
  }

  // Sync orientation sub-nodes with any trajectory changes in the tree.
  auto tree_node = node->as<TreeNode>();
  if (!tree_node) {
    throw std::runtime_error("YawPlanningEvaluator can only be used with tree-based generators that produce TreeNodes!");
  }
  for (size_t i = 0; i < info->orientations.size(); ++i) {
    auto tree_node_i = info->orientations[i]->as<TreeNode>();
    if (tree_node_i->parent != tree_node->parent ||
        !tree_node_i->edge.trajectory.front().isSameSensorPosition(tree_node->edge.trajectory.front())) {
      tree_node_i->parent = tree_node->parent;
      tree_node_i->edge.trajectory = tree_node->edge.trajectory;

      // A refusal (pose twin of the new parent) is fine to ignore here: the copy keeps
      // the parent's trajectory, and setBestYaw can never apply it to the real node.
      setNodeOrientation(info->orientations[i].get(), tree_node_i->robot_state.getSensorOrientation());
    }
  }

  NodeClusterPtrs orientations_to_update;
  for (int i = 0; i < static_cast<int>(info->orientations.size()); ++i) {
    auto& orientation_i = info->orientations[i];
    if (info->visited_orientation[i]) {
      continue;
    }

    if (!node->is_candidate_traversable[i]) {
      continue;
    }

    if (orientation_i->getSingleNodeGain() < p_update_gain_ && !orientation_i->is_at_frontier) {
      continue;
    }
    if (!sensor_evaluator_->sensorViewOverlapsUpdatedMap(info->orientations[i]->robot_state)) {
      continue;
    }

    orientations_to_update.push_back(info->orientations[i].get());
  }

  following_evaluator_->evaluate(orientations_to_update);

  // Skip yaw update for the current node — its orientation is fixed by the executed
  // trajectory endpoint; calling setBestYaw() would reorient child edges and break
  // the trajectory-continuity check in the next addVisitedEdge() call.
  if (node == current_node) {
    return true;
  }

  return setBestYaw(node);
}

void YawPlanningEvaluator::getVisualization(VisualizationMarkers* markers, const Node* node) {
  const YawPlanningInfo* info = node->info ? node->info->as<YawPlanningInfo>() : nullptr;
  if (info == nullptr) {
    return;
  }

  // Let the followup evaluator draw the visualization of the selected
  // orientation
  following_evaluator_->getVisualization(markers, info->orientations[info->active_orientation].get());
}

}  // namespace evaluator
}  // namespace open_apero
