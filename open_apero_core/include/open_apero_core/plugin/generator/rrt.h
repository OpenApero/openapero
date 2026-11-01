#pragma once

/**
 * @file rrt.h
 * @brief Rapidly-exploring Random Tree (RRT) node/edge generator.
 */

#include <memory>
#include <string>
#include <vector>

#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/generator/tree_generator.h"

namespace open_apero {
namespace generator {

/**
 * @brief RRT generator (RRAT in the paper) that expands a tree with random linear segments.
 */
class RRT : public TreeGenerator {
 public:
  /** @brief Constructs the RRT generator and binds it to the agent. */
  explicit RRT(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  /** @brief Loads RRT-specific parameters (sample_max_tries) from the config map. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Validates sample_max_tries >= 0, then the GraphGenerator edge lengths. */
  bool checkParamsValid(std::string* error_message) override;

  /**
   * @brief Updates the tree root when the robot moves to a new start node.
   * @param previous_start The old root node being replaced.
   * @param new_start      The new root node after the robot has moved.
   */
  void setNewStartNode(Node* previous_start, Node* new_start) override;

  /**
   * @brief Samples a random state and generates a new node cluster near the tree.
   * @param start_node The current root; seeds the first child of an empty tree (the nearest node comes from the KD-tree).
   * @return Cluster of newly generated candidate nodes (empty on failure).
   */
  NodeClusterPtrs generateNewNode(Node* start_node) override;

 protected:
  static PluginRegistry::Registration<RRT> registration;

  // parameters
  int p_sample_max_tries_ = 1000;  // maximum sampling attempts per generateNewNode() call; 0 = unlimited
};

}  // namespace generator
}  // namespace open_apero
