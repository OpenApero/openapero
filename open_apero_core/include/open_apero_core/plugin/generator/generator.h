#pragma once

/**
 * @file generator.h
 * @brief KD-tree-backed base for sampled node generation.
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/kdtree.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

// Layer includes this plugin interface, so including layer.h here would create
// an include cycle.
class Layer;

/**
 * @brief Abstract base class for sample-based node generators.
 *
 * Provides the KD-tree, the sampling interface, and collision-checking access to the robot
 * model; subclasses implement a sampling strategy.
 */
class Generator : public Plugin {
 public:
  /** @brief Constructs a Generator and binds it to the agent. */
  explicit Generator(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~Generator() = default;

  /** @brief Sets the owning layer; called by the owning layer after setup. getLayer() is nullptr during setupFromParamMap. */
  void setLayer(Layer* layer) { layer_ = layer; }

  /** @brief Owning layer, or nullptr if not yet wired. Used to reach sibling plugins. */
  Layer* getLayer() const { return layer_; }

  /** @brief Binds the agent's robot model; the base reads no keys. */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /** @brief Resets sampling-side state (KD-tree) for a new episode. */
  void resetEpisode() override;

  /**
   * @brief Returns all non-intermediate node clusters in the KD-tree, grouped by position.
   *
   * Each entry is a cluster of nodes sharing the same 3D location. The returned
   * reference is invalidated if the KD-tree is rebuilt (e.g. after rebuildKdTree()).
   * @return Const reference to the vector of node clusters.
   */
  const std::vector<NodeClusterPtrs>& getAllNodeClusters() const { return kd_tree_data_.nodes; }

  /**
   * @brief Whether @p node is still a member of this generator's structure.
   *
   * Pointer-only check: must not dereference @p node, which may already be freed. The default
   * answers over the KD-tree index; a generator inheriting it must rebuild the index in the same
   * operation that releases a node. Generators owning nodes outside the index override it.
   */
  virtual bool containsNode(const Node* node) const;

  /** @brief Rebuilds the KD-tree for the current node set. */
  virtual bool rebuildKdTree() = 0;

  /**
   * @brief Samples a new candidate position and adds it as a node cluster to the index.
   * @param start_node Optional sampling context, such as a graph's start node.
   * @return Pointers to the newly created node cluster.
   */
  virtual NodeClusterPtrs generateNewNode(Node* start_node) = 0;

  /** @brief Returns the agent map used by this generator. */
  Map* getMap() const { return &agent_.getMap(); }

  /**
   * @brief Generates gain-colored markers for all sampled (non-intermediate) nodes in the KD-tree.
   *
   * @param markers Output marker container.
   */
  virtual void getVisualization(VisualizationMarkers* markers);

 protected:
  /**
   * @brief Finds all nearby node clusters within max_length of a point.
   * @param point Query point.
   * @param nearby_node_clusters Output node clusters in KD-tree traversal order.
   * @param max_length Maximum search radius.
   * @return true if any nearby nodes were found.
   */
  bool findNearbyNodes(const Eigen::Vector3d& point, std::vector<NodeClusterPtrs>& nearby_node_clusters, double max_length);

  /**
   * @brief Finds the nearest node cluster to a point in the KD-tree.
   * @param point Query point.
   * @param nearest_node_cluster Output nearest node cluster.
   * @param dist_squared Output squared distance to the nearest cluster.
   * @return true if a nearest node was found.
   */
  bool findNearestNode(const Eigen::Vector3d& point, NodeClusterPtrs& nearest_node_cluster, double& dist_squared);

  /**
   * @brief Adds a freshly built node cluster to the KD-tree.
   *
   * All nodes in @p cluster must share a 3D position (they map to a single
   * KD-tree point with multiple stored Node pointers).
   *
   * @param cluster The cluster to register.
   */
  void addToKdTree(const NodeClusterPtrs& cluster);

  /**
   * @brief Finds nearby clusters in an explicitly supplied spatial index.
   *
   * @p kdtree must be null or built over @p data. The output is always cleared first and is in
   * KD-tree traversal order (unsorted). A radius <= 1e-2 is unbounded, as in findNearbyNodes().
   */
  static bool findNearbyNodesInIndex(const KdTree* kdtree, const KdTreeData& data, const Eigen::Vector3d& point,
                                     std::vector<NodeClusterPtrs>* nearby_node_clusters, double max_length);

  /**
   * @brief Finds the nearest cluster in an explicitly supplied spatial index.
   *
   * @p kdtree must be null or have been constructed over @p data. The cluster
   * output is cleared before every query; both outputs are populated on success.
   */
  static bool findNearestNodeInIndex(const KdTree* kdtree, const KdTreeData& data, const Eigen::Vector3d& point,
                                     NodeClusterPtrs* nearest_node_cluster, double* dist_squared);

  /**
   * @brief Adds a cluster to an explicitly supplied spatial index.
   *
   * The tree must be initialized over @p data. The cluster must be non-empty,
   * non-intermediate, co-located, and not already present in the index.
   */
  static void addClusterToIndex(KdTree* kdtree, KdTreeData* data, const NodeClusterPtrs& cluster);

  /**
   * @brief Recomputes is_traversable for every node of one co-located cluster.
   *
   * Goes through RobotModel::checkOrientations, so refresh applies exactly the creation rules
   * (stationing included); the nodes' states are unchanged.
   *
   * @param cluster Co-located nodes sharing one sensor position. Must be non-empty.
   * @param map     Unused by the in-tree generators: checkOrientations queries the robot's own map.
   */
  void refreshClusterTraversability(const NodeClusterPtrs& cluster, Map* map) const;

  /**
   * @brief Whether the map region updated since the last cycle can change the robot's verdict at any of @p states.
   *
   * True when the updated region meets the body bounds (RobotModel::getBodyBounds) of a state, grown by @p reach per
   * axis, or when the robot reports no bounds. False without a map update.
   * @param states The states to check, e.g. an edge's trajectory.
   * @param reach  How far beyond the bodies a change still matters [m], e.g. an edge length for a new connection.
   */
  bool isAffectedByMapUpdate(const RobotTrajectory& states, const Eigen::Vector3d& reach = Eigen::Vector3d::Zero()) const;

  /// isAffectedByMapUpdate() over the nodes' robot states.
  bool isAffectedByMapUpdate(const NodeClusterPtrs& cluster, const Eigen::Vector3d& reach = Eigen::Vector3d::Zero()) const;

  /// Robot model used for sampling-time collision checks.
  RobotModel* robot_ = nullptr;

  /// Spatial index for sampled node clusters.
  std::unique_ptr<KdTree> kdtree_;
  KdTreeData kd_tree_data_;

  Layer* layer_ = nullptr;
};

}  // namespace open_apero
