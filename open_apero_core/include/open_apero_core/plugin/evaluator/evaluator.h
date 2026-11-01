#pragma once

/**
 * @file evaluator.h
 * @brief Base class for computing information gain at planning nodes.
 *
 * An Evaluator attaches gain anchors to each node, representing how much new information
 * visiting that viewpoint would acquire. Implementations include the naive (unknown voxel
 * counting), surface frontier and voxel weight evaluators, all built on the abstract
 * SimulatedSensorEvaluator.
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

// Layer includes this plugin interface, so including layer.h here would create
// an include cycle.
class Layer;

/**
 * @brief Abstract base class for evaluating information gain at planning nodes.
 *
 * Implementations must define evaluateUnobserved() and getRealAccumulatedGain(). Callers use
 * evaluate() and updateEvaluation(), which pass only the unobserved nodes (Node::is_observed) on to
 * evaluateUnobserved() and reevaluateUnobserved(), so an observed node's gain stays spent under every
 * evaluator.
 */
class Evaluator : public Plugin {
 public:
  /** @brief Constructs an Evaluator and binds it to the agent. */
  explicit Evaluator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~Evaluator() = default;

  /** @brief Sets the owning layer; called by the owning layer after setup. getLayer() is nullptr during setupFromParamMap. */
  void setLayer(Layer* layer) { layer_ = layer; }

  /** @brief Owning layer, or nullptr if not yet wired. Used to reach sibling plugins. */
  Layer* getLayer() const { return layer_; }

  /**
   * @brief Resets evaluator state for a new episode. Default: no-op.
   *
   * Per-node state is cleared with the graph; subclasses clear accumulated gain tracking.
   */
  void resetEpisode() override {}

  /**
   * @brief Evaluates the information gain for a cluster of candidate viewpoint nodes.
   *
   * All nodes in the cluster share the same spatial position but may differ in
   * orientation. Observed nodes are skipped; the rest go to evaluateUnobserved().
   * @param cluster The candidate viewpoint nodes to evaluate.
   * @return true on success, and when no unobserved node is left.
   */
  bool evaluate(const NodeClusterPtrs& cluster);

  /**
   * @brief Re-evaluates a cluster's gain after the map has been updated.
   *
   * Called during each planning cycle for every cluster in the graph. Observed nodes are skipped;
   * the rest go to reevaluateUnobserved().
   * @param cluster      The nodes to update.
   * @param current_node If non-null and present in the cluster, gain is re-evaluated
   *                     but the node's orientation is not modified.
   * @return Whether re-evaluation succeeded, and true when no unobserved node is left. Never used
   *         to prune nodes; the generator's traversability pruning does that.
   */
  bool updateEvaluation(const NodeClusterPtrs& cluster, const Node* current_node = nullptr);

  /**
   * @brief Visualizes the information gain at this node for simulation display.
   * @param markers Output visualization markers.
   * @param node The node to visualize.
   */
  virtual void getVisualization(VisualizationMarkers* markers, const Node* node);

  /**
   * @brief Visualizes what the selected path as a whole is being credited with.
   *
   * Differs from the per-node view where gain is shared between nodes. The default draws nothing.
   *
   * @param markers Output visualization markers.
   * @param path The selected path, in the order it is traversed.
   */
  virtual void getPathVisualization(VisualizationMarkers* markers, const std::vector<Node*>& path);

  /**
   * @brief Returns the total accumulated information gain across all reached nodes.
   * @return Accumulated gain value.
   */
  virtual double getRealAccumulatedGain() const = 0;

  /**
   * @brief Called once per planning cycle after a node has been reached.
   *
   * The default spends the node (Node::markObserved()). Subclasses that track visits more finely
   * (e.g. per orientation) override it without calling the base.
   *
   * @param reached_node   The node the robot just navigated to.
   * @param traversed_edge Edge used to reach the node, or nullptr.
   */
  virtual void onNodeReached(Node* reached_node, const Edge* traversed_edge);

  /** @brief Loads base Evaluator parameters (bounding volume) from the config map. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

 protected:
  /**
   * @brief Evaluates a cluster for evaluate(); fills each node's gain anchors.
   *
   * Implementations are encouraged to share expensive computation (e.g., raycasting) across
   * cluster members.
   * @param cluster Nodes of one cluster, none of them observed; evaluate() never passes an empty one.
   * @return true on success.
   */
  virtual bool evaluateUnobserved(const NodeClusterPtrs& cluster) = 0;

  /**
   * @brief Re-evaluates a cluster for updateEvaluation(). Default: never re-evaluates.
   * @param cluster      Nodes of one cluster, none of them observed; updateEvaluation() never passes
   *                     an empty one.
   * @param current_node As for updateEvaluation().
   * @return Whether re-evaluation succeeded.
   */
  virtual bool reevaluateUnobserved(const NodeClusterPtrs& cluster, const Node* current_node);

  // bounding volume of interesting target
  std::unique_ptr<BoundingVolume> bounding_volume_;

  Layer* layer_ = nullptr;
};

}  // namespace open_apero
