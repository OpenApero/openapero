#include "open_apero_core/plugin/generator/rrt.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace generator {

PluginRegistry::Registration<RRT> RRT::registration("RRT");

RRT::RRT(AgentInterface& agent) : TreeGenerator(agent) {}

void RRT::setupFromParamMap(Configurable::ParamMap* param_map) {
  TreeGenerator::setupFromParamMap(param_map);

  setParam<int>(param_map, "sample_max_tries", &p_sample_max_tries_, 1000);
}

bool RRT::checkParamsValid(std::string* error_message) {
  if (p_sample_max_tries_ < 0) {
    *error_message = "sample_max_tries expected >= 0 (0 = unlimited)";
    return false;
  }
  return TreeGenerator::checkParamsValid(error_message);
}

void RRT::setNewStartNode(Node* previous_start, Node* new_start) {
  if (previous_start == new_start) {
    throw std::runtime_error("RRT::setNewStartNode called with same previous and new start node");
  }

  if (previous_start != root_node_.get()) {
    throw std::runtime_error("RRT::setNewStartNode called with non-current node");
  }

  // find the index of the new_start in the children of the previous_start
  bool is_node_found = false;

  for (int i = 0; i < static_cast<int>(root_node_->children.size()); ++i) {
    if (root_node_->children[i].get() == new_start) {
      root_node_ = std::move(root_node_->children[i]);
      root_node_->parent = nullptr;

      is_node_found = true;
      break;
    }
  }

  if (!is_node_found) {
    throw std::runtime_error("RRT::setNewStartNode failed to find new_start in children of previous_start");
  }

  // Prune intermediate children of the new root node
  auto& children = root_node_->children;
  children.erase(std::remove_if(children.begin(), children.end(), [](const auto& c) { return c->is_intermediate; }), children.end());

  // If the root has changed, rebuild the kdtree
  rebuildKdTree();
}

NodeClusterPtrs RRT::generateNewNode(Node* start_node) {
  // sample candidate points
  int counter = 0;
  while (p_sample_max_tries_ == 0 || counter < p_sample_max_tries_) {
    if (p_sample_max_tries_ > 0) {
      counter++;
    }

    Node* parent_node;
    Eigen::Vector3d goal_pos;
    // Spacing requires an existing anchor, so seed an empty tree at the robot pose.
    // That seed is not a free choice of position, so it is exempt from the stationing
    // rule -- and being deterministic, retrying it cannot change the outcome.
    const bool is_bootstrap_seed = kd_tree_data_.size() == 0;
    if (is_bootstrap_seed) {
      // Empty tree: seed the first child at the robot's position, parented to start_node.
      parent_node = start_node;
      goal_pos = start_node->robot_state.getSensorPosition();
    } else {
      // sample a pos
      if (!robot_->samplePosition(&goal_pos, agent_.getInitialSensorPosition().z())) {
        return {};
      }

      // find closest point in kdtree
      NodeClusterPtrs nearest_node_cluster;
      double out_dist_sqr = 0.0;
      if (!findNearestNode(goal_pos, nearest_node_cluster, out_dist_sqr)) {
        return {};
      }

      if (nearest_node_cluster.size() != 1) {
        throw std::runtime_error("RRT::generateNewNode: findNearestNode returns more than one node!");
      }

      parent_node = nearest_node_cluster.front();
      const Eigen::Vector3d& v_nearest_pos = parent_node->robot_state.getSensorPosition();

      // Check if the goal pos is too close or too far using out_dist_sqr
      if (out_dist_sqr < p_min_edge_length_ * p_min_edge_length_) {
        continue;
      }

      // Steer function: if too far, crop the segment
      if (p_max_edge_length_ > 0.0 && out_dist_sqr > p_max_edge_length_ * p_max_edge_length_) {
        goal_pos = v_nearest_pos + (goal_pos - v_nearest_pos).normalized() * p_min_edge_length_;
      }
    }

    // sample candidate orientations and check traversability for each
    std::vector<Eigen::Quaterniond> candidate_orientations;
    robot_->sampleOrientations(goal_pos, &candidate_orientations);

    std::vector<RobotState> candidate_states;
    std::vector<bool> is_candidate_traversable;
    robot_->checkOrientations(goal_pos, candidate_orientations, &candidate_states, &is_candidate_traversable,
                              is_bootstrap_seed ? StationingCheck::kExemptBootstrap : StationingCheck::kEnforce);

    const bool any_traversable = std::any_of(is_candidate_traversable.begin(), is_candidate_traversable.end(), [](bool t) { return t; });
    if (!any_traversable) {
      // The seed position is fixed, so every remaining try would test it again.
      if (is_bootstrap_seed) {
        return {};
      }
      continue;
    }

    std::vector<int> traversable_indices;
    traversable_indices.reserve(is_candidate_traversable.size());
    for (std::size_t i = 0; i < is_candidate_traversable.size(); ++i) {
      if (is_candidate_traversable.at(i)) {
        traversable_indices.push_back(static_cast<int>(i));
      }
    }

    // A child at the parent's position (the seed) must differ in orientation, or no edge exists;
    // the orientation grid is deterministic, so bail rather than retry.
    if (isSamePosition(parent_node->robot_state.getSensorPosition(), goal_pos)) {
      traversable_indices.erase(
          std::remove_if(traversable_indices.begin(), traversable_indices.end(),
                         [&](int idx) { return candidate_states.at(idx).isSameSensorOrientation(parent_node->robot_state); }),
          traversable_indices.end());
      if (traversable_indices.empty()) {
        return {};
      }
    }

    const int selected_index = traversable_indices.front();

    auto new_node = parent_node->as<TreeNode>()->spawnChild();
    new_node->robot_state = candidate_states.at(selected_index);
    new_node->is_traversable = is_candidate_traversable.at(selected_index);
    new_node->candidate_orientations = candidate_orientations;
    new_node->is_candidate_traversable = is_candidate_traversable;

    return {new_node};
  }

  return {};
}

}  // namespace generator
}  // namespace open_apero
