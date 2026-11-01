#include "open_apero_core/plugin/generator/rrt_star.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace generator {

PluginRegistry::Registration<RRTStar> RRTStar::registration("RRTStar");

RRTStar::RRTStar(AgentInterface& agent) : RRT(agent) {}

bool RRTStar::generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) {
  if (node_cluster.size() > 1) {
    throw std::runtime_error("RRTStar::generateNewEdges called with more than one new node");
  }
  auto node = node_cluster.front();

  if (!TreeGenerator::generateNewEdges(node_cluster, new_edges)) {
    return false;
  }

  std::vector<NodeClusterPtrs> candidate_parent_clusters;
  if (!findNearbyNodes(node->robot_state.getSensorPosition(), candidate_parent_clusters, p_max_edge_length_)) {
    return false;
  }

  std::vector<Node*> candidate_parents;
  candidate_parents.reserve(candidate_parent_clusters.size());
  for (const auto& node_cluster : candidate_parent_clusters) {
    if (node_cluster.size() > 1) {
      throw std::runtime_error("RRTStar::generateNewEdges: node cluster size > 1");
    }

    candidate_parents.push_back(node_cluster.front());
  }

  rewireToBestParent(node, candidate_parents);

  for (const auto candidate : candidate_parents) {
    TreeNode* candidate_tree_node = candidate->as<TreeNode>();
    if (candidate_tree_node->parent) {
      rewireToBestParent(candidate, {node});
    }
  }

  new_edges.clear();
  new_edges.push_back(&(node->as<TreeNode>()->edge));
  return true;
}

void RRTStar::setNewStartNode(Node* previous_start, Node* new_start) {
  if (previous_start == new_start) {
    throw std::runtime_error("RRTStar::setNewStartNode called with same previous and new start node");
  }

  if (previous_start != root_node_.get()) {
    throw std::runtime_error("RRTStar::setNewStartNode called with non-current node");
  }

  // Preserve the old root below the new root until intermediate pruning.
  TreeNode* new_tree_start = new_start->as<TreeNode>();
  auto reinserted_root = new_tree_start->spawnChild();
  static_cast<Node&>(*reinserted_root) = *root_node_;  // copies all Node fields; parent set by spawnChild()

  reinserted_root->info = std::move(root_node_->info);  // moved: the old root is discarded below
  reinserted_root->edge.trajectory = reverseTrajectory(new_tree_start->edge.trajectory);
  computeCost(&(reinserted_root->edge));

  std::vector<TreeNode*> children_to_move;
  root_node_->getChildren(&children_to_move);
  children_to_move.erase(std::remove(children_to_move.begin(), children_to_move.end(), new_start), children_to_move.end());
  for (const auto child : children_to_move) {
    moveNodeToNewParent(child, reinserted_root);
  }

  if (root_node_->children.size() != 1) {
    throw std::runtime_error(
        "RRTStar::setNewStartNode: current node has more than one child. They should already have been moved to the reinserted root.");
  }

  root_node_ = std::move(root_node_->children[0]);
  root_node_->parent = nullptr;
  rebuildKdTree();

  rewireTree(root_node_.get());

  // Intermediates should be childless after rewiring.
  removeIntermediateNodes(root_node_.get());

  rebuildKdTree();
}

void RRTStar::rewireTree(TreeNode* root) {
  std::vector<TreeNode*> nodes;

  root->getTree(&nodes);
  std::vector<std::pair<double, TreeNode*>> distance_pairs;

  for (int i = 1; i < static_cast<int>(nodes.size()); ++i) {
    distance_pairs.push_back(
        std::make_pair((nodes[i]->robot_state.getSensorPosition() - root->robot_state.getSensorPosition()).norm(), nodes[i]));
  }
  std::sort(distance_pairs.begin(), distance_pairs.end());

  for (int i = 0; i < static_cast<int>(distance_pairs.size()); ++i) {
    auto current = distance_pairs[i].second;

    std::vector<NodeClusterPtrs> nearby_node_clusters;
    if (!findNearbyNodes(current->robot_state.getSensorPosition(), nearby_node_clusters, p_max_edge_length_)) {
      continue;
    }

    std::vector<Node*> nearby_nodes;
    nearby_nodes.reserve(nearby_node_clusters.size());
    for (const auto& node_cluster : nearby_node_clusters) {
      if (node_cluster.size() > 1) {
        throw std::runtime_error("RRTStar::rewireTree: node cluster size > 1");
      }
      auto node = node_cluster.front();

      if (node->is_intermediate && node != root) {
        continue;
      }

      nearby_nodes.push_back(node);
    }

    // Rewiring preserves root connectivity: a sole child is an ancestor of all
    // other nodes and therefore cannot choose one of them as its new parent.
    if (rewireToBestParent(current, nearby_nodes, true)) {
      for (auto nearby_node : nearby_nodes) {
        if (nearby_node->as<TreeNode>()->parent == current) {
          rewireToBestParent(nearby_node, {current});
        }
      }
    }
  }
}

void RRTStar::removeIntermediateNodes(TreeNode* node) {
  auto& children = node->children;
  children.erase(std::remove_if(children.begin(), children.end(), [](const auto& c) { return c->is_intermediate; }), children.end());
  for (auto& child : children) {
    removeIntermediateNodes(child.get());
  }
}

bool RRTStar::rewireToBestParent(Node* node, const std::vector<Node*>& candidates, bool force_rewire) {
  auto tree_node = node->as<TreeNode>();
  if (!tree_node) {
    throw std::runtime_error("RRTStar::rewireToBestParent: node is not a TreeNode");
  }

  if (!tree_node->parent) {
    throw std::runtime_error("RRTStar::rewireToBestParent: node has no parent (root)");
  }

  // Connecting to a descendant would form a cycle.
  std::vector<TreeNode*> safe_candidates;
  for (const auto candidate : candidates) {
    bool is_node_predecessor = false;

    TreeNode* current = candidate->as<TreeNode>();
    while (current) {
      if (current == node) {
        is_node_predecessor = true;
        break;
      }
      current = current->parent;
    }

    if (!is_node_predecessor) {
      safe_candidates.push_back(candidate->as<TreeNode>());
    }
  }

  auto initial_parent = tree_node->parent;

  std::unique_ptr<Node> best_segment{tree_node->shallowCopy()};
  auto best_segment_ptr = best_segment->as<TreeNode>();

  double least_cost_to_node = 0.0;
  for (auto current = best_segment_ptr; current; current = current->parent) {
    least_cost_to_node += current->edge.cost;
  }

  for (auto candidate : safe_candidates) {
    // Coincident positions require rotation; exact pose twins are invalid.
    const bool is_rotation = isSamePosition(candidate->robot_state.getBasePosition(), tree_node->robot_state.getBasePosition());
    if (is_rotation && candidate->robot_state.isSameSensorOrientation(tree_node->robot_state)) {
      agent_.printError("RRTStar::rewireToBestParent: skipping pose-identical candidate parent; the tree holds no pose twins.");
      continue;
    }

    tree_node->edge.trajectory.clear();
    tree_node->parent = candidate;

    double cost_to_candidate = 0.0;
    for (auto current = candidate; current; current = current->parent) {
      cost_to_candidate += current->edge.cost;
    }

    Edge new_edge;
    const bool built = is_rotation ? generateRotationEdgeBetweenNodes(candidate, tree_node, &new_edge)
                                   : generateEdgeBetweenNodes(candidate, tree_node, &new_edge);
    if (built) {
      tree_node->edge = new_edge;
      tree_node->robot_state = tree_node->edge.trajectory.back();

      bool rewire_to_candidate = false;
      if (force_rewire) {
        rewire_to_candidate = true;
      } else {
        if (cost_to_candidate + tree_node->edge.cost < least_cost_to_node) {
          rewire_to_candidate = true;
        }
      }

      if (rewire_to_candidate) {
        least_cost_to_node = cost_to_candidate + tree_node->edge.cost;

        best_segment = tree_node->shallowCopy();
        best_segment_ptr = best_segment->as<TreeNode>();

        force_rewire = false;
      }
    }
  }

  if (best_segment_ptr->parent == nullptr) {
    return false;
  } else {
    tree_node->parent = best_segment_ptr->parent;
    tree_node->edge = best_segment_ptr->edge;

    if (tree_node->parent == initial_parent) {
      return !force_rewire;
    } else {
      for (int i = 0; i < static_cast<int>(initial_parent->children.size()); ++i) {
        if (initial_parent->children[i].get() == node) {
          tree_node->parent->children.push_back(std::move(initial_parent->children[i]));
          initial_parent->children.erase(initial_parent->children.begin() + i);
          return true;
        }
      }
      // Construction guarantees a valid parent when rewiring is requested.
      return false;
    }
  }
}

}  // namespace generator

}  // namespace open_apero
