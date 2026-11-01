#pragma once

/**
 * @file rrag.h
 * @brief RRAG: node and edge generator that builds and maintains a graph-based
 *        exploration structure.
 */

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/plugin/generator/graph_generator.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/history_data.h"

namespace open_apero {
namespace generator {

/**
 * @brief RRAG (rapidly-exploring random annulus graph): samples new nodes in the annulus
 *        [min_edge_length, max_edge_length] around existing clusters and maintains the graph.
 *
 * The paper's tree-shaped variants, RRAT and RRAT*, are registered as "RRT" and "RRTStar".
 */
class RRAG : public GraphGenerator {
 public:
  /**
   * @brief Construct a RRAG.
   * @param agent Reference to the parent agent interface.
   */
  explicit RRAG(AgentInterface& agent);  // NOLINT(google-runtime-references) non-const ref to agent is intentional

  virtual ~RRAG() = default;

  /**
   * @brief Initialize parameters from the parameter map.
   * @param param_map Pointer to the plugin parameter map.
   */
  void setupFromParamMap(Configurable::ParamMap* param_map) override;

  /**
   * @brief Validates sample_max_tries >= 0, then the GraphGenerator edge lengths.
   * @param error_message Output string describing any validation errors.
   * @return True if all parameters are valid.
   */
  bool checkParamsValid(std::string* error_message) override;

  /** @brief Resets graph state for a new episode. */
  void resetEpisode() override {
    GraphGenerator::resetEpisode();
    graph_.reset();
  }

  /**
   * @brief Retrieve every node of the graph.
   * @param start_node Unused.
   * @return Vector of pointers to all nodes.
   */
  std::vector<Node*> getAllNodes(Node* start_node) const override;

  /**
   * @brief Retrieve every edge of the graph.
   * @param start_node Unused.
   * @return Vector of pointers to all edges.
   */
  std::vector<Edge*> getAllEdges(Node* start_node) const override;

  /**
   * @brief Create the initial graph node from the robot's starting state.
   * @param init_state The initial robot state.
   * @return A pair of the graph and the initial node.
   */
  std::pair<GraphType*, Node*> generateInitNode(const RobotState& init_state) override;

  /**
   * @brief Update the start node of the graph after the robot moves.
   * @param previous_start The old start node.
   * @param new_start The new start node.
   */
  void setNewStartNode(Node* previous_start, Node* new_start) override;

  /**
   * @brief Sample a new point and add it as a node cluster to the graph.
   * @param start_node The current start node.
   * @return Pointers to the newly created node cluster.
   */
  NodeClusterPtrs generateNewNode(Node* start_node) override;

  /**
   * @brief Generate edges connecting a new node cluster to the existing graph.
   * @param node_cluster The node cluster to connect.
   * @param new_edges Output vector of newly created edges.
   * @return True if edges were successfully generated.
   */
  bool generateNewEdges(NodeClusterPtrs& node_cluster, std::vector<Edge*>& new_edges) override;

  /**
   * @brief Insert an intermediate node along an existing edge.
   * @param start_node The source node of the edge.
   * @param goal_node The target node of the edge.
   * @param edge The edge to split.
   * @param node_index Index of the intermediate point along the trajectory.
   * @return A pair of the new intermediate node and the new edge.
   */
  std::pair<Node*, Edge*> insertIntermediateNode(Node* start_node, Node* goal_node, Edge* edge, int node_index) override;

  /**
   * @brief Insert a new intermediate node at the endpoint of @p executed_edge into the boost graph.
   *
   * Never returns nullptr: the new vertex stays as a detached intermediate start node when no
   * out-edge can be planned from it. Removes the previous intermediate start node either way.
   *
   * @param previous The current start vertex in this graph.
   * @param executed_edge Edge whose endpoint gives the new node's robot state.
   */
  Node* insertIntermediateNodeAtCurrentState(Node* previous, const Edge& executed_edge) override;

  bool containsNode(const Node* node) const override;

  void connectStartNode(Node* start_node) override;

  /**
   * @brief Re-checks the nodes and edges a map update can affect, restores missing edges, and prunes.
   * @param start_node The current start node.
   */
  void updateNodeEdge(Node* start_node) override;

  /**
   * @brief Change the orientation of a node and update connected edges.
   * @param node The node to reorient.
   * @param orientation The new orientation quaternion.
   * @return True if the orientation was successfully set.
   */
  bool setNodeOrientation(Node* node, const Eigen::Quaterniond& orientation) override;

  /** @brief Graph edges are bidirectional. */
  bool isEdgeBidirectional() const override { return true; }

 protected:
  /**
   * @brief Finds the nearest eligible anchor for expanding toward @p point.
   *
   * The anchor sets minimum spacing, maximum-distance pullback and the new cluster's first
   * connection. Returning false aborts node generation, or removes the not yet indexed cluster
   * during edge generation. The default searches the primary spatial index.
   */
  virtual bool findNearestExpansionAnchor(const Eigen::Vector3d& point, NodeClusterPtrs& nearest_node_cluster, double& dist_squared);

  /**
   * @brief Checks a sampled position after spacing and any pullback.
   *
   * Called before orientation and feasibility checks, except for the seed sample of an empty
   * index; returning false redraws a fresh sample.
   * @return true if the sample may be placed at @p position.
   */
  virtual bool isSamplePositionAdmissible(const Eigen::Vector3d& position) const;

  /**
   * @brief Builds the orientation cluster at @p position and adds its nodes to the graph.
   *
   * The robot then gives the members the gain anchors they share (RobotModel::attachSharedGainAnchors).
   * Precondition: @p position has passed spacing.
   * @param position   Sensor position of the cluster.
   * @param stationing Whether the orientation check enforces the stationing rule.
   * @return The new cluster, or an empty cluster, with nothing added to the graph, when no
   *         orientation at @p position is traversable.
   */
  NodeClusterPtrs buildClusterAt(const Eigen::Vector3d& position, StationingCheck stationing);

  /**
   * @brief Registers a newly connected cluster in RRAG's spatial indices.
   *
   * The default updates the primary index. Overrides that maintain auxiliary
   * indices must call RRAG::indexNodeCluster exactly once before augmenting it.
   */
  virtual void indexNodeCluster(const NodeClusterPtrs& cluster);

  bool rebuildKdTree() override;

  std::vector<Edge*> generateInterClusterEdges(NodeClusterPtrs& node_cluster);

  std::vector<Edge*> generateIntraClusterEdges(NodeClusterPtrs& node_cluster);

  /// Virtual so a subclass sampling several node populations can forbid edges
  /// between some of them: every inter-cluster edge this generator creates -- on insertion and
  /// on the per-cycle generateMissingEdges pass -- is routed through here.
  virtual std::vector<Edge*> generateEdgesFromClusterToCluster(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster);

  /**
   * @brief Connects two clusters along one planned corridor's own departure and arrival directions.
   *
   * Plans a single seed edge for the pair the chord picks, then re-selects the pairs against
   * the directions its trajectory actually leaves and arrives along. Every other selected pair
   * re-aims the seed's end rotations, so the corridor is planned once however many pairs it serves.
   *
   * @param from_cluster Source cluster.
   * @param to_cluster   Target cluster, at a different position.
   * @param from_states  Robot states of @p from_cluster, in cluster order.
   * @param to_states    Robot states of @p to_cluster, in cluster order.
   * @return The edges inserted into the graph.
   */
  std::vector<Edge*> generatePathHeadingEdges(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster,
                                              const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states);

  /**
   * @brief Connects two clusters that share a position by rotation alone.
   *
   * The source of such a pair is always a single intermediate node left behind by
   * a start-node update, so every target orientation is reachable from it by one rotation.
   *
   * @param from_cluster Source cluster, a single intermediate node.
   * @param to_cluster   Target cluster, at the same position.
   * @return The edges inserted into the graph.
   */
  std::vector<Edge*> generateRotationEdges(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster);

  /**
   * @brief Connects two clusters along the straight line between their positions.
   *
   * The robot picks the single pair of endpoint orientations best aligned with that chord.
   *
   * @param from_cluster Source cluster.
   * @param to_cluster   Target cluster, at a different position.
   * @param from_states  Robot states of @p from_cluster, in cluster order.
   * @param to_states    Robot states of @p to_cluster, in cluster order.
   * @return The edges inserted into the graph.
   */
  std::vector<Edge*> generateStraightLineEdges(NodeClusterPtrs& from_cluster, NodeClusterPtrs& to_cluster,
                                               const std::vector<RobotState>& from_states, const std::vector<RobotState>& to_states);

  /// Adds an already-built edge to the graph and propagates connect_with_start_node.
  Edge* insertEdge(Node* from_node, Node* to_node, NodeId from_node_id, NodeId to_node_id, std::shared_ptr<Edge> edge);

  // updateNodeEdge() must call these in declaration order: each update refreshes
  // traversability flags consumed by the following removal step. An update re-checks a
  // cluster, an edge or a missing connection only where isAffectedByMapUpdate() admits it.
  void updateNodeTraversability(Node* start_node);

  /// The start node's cluster is kept whatever its flags say: the robot stands on it.
  void removeUntraversableNodeClusters(Node* start_node);

  /// The start node's own edges are revalidated whatever the map update.
  void updateEdgeTraversability(Node* start_node);

  void removeUntraversableEdges();

  void generateMissingEdges(Node* start_node);

  /// Prunes nothing while the start state is untraversable; reachability is undefined
  /// from a start node the robot cannot stand on.
  void removeDisconnectedNodeClusters(Node* start_node);

  /// Whether the robot could stand at @p start_node's state, per the robot model rather
  /// than Node::is_traversable: an intermediate start node is created traversable and never refreshed.
  bool isStartStateTraversable(const Node* start_node) const;

  static PluginRegistry::Registration<RRAG> registration;

  /// Owned Boost graph.
  std::unique_ptr<GraphType> graph_;

  /// Maximum sampling attempts per generateNewNode() call; 0 = unlimited.
  int p_sample_max_tries_ = 1000;

  /** @brief How a translation edge's endpoint orientations are chosen. */
  enum class TranslationAdjacency {
    kStraightLine,     ///< The single pair best aligned with the chord between the clusters (default).
    kPathHeading,      ///< The single pair best aligned with the planned trajectory's end directions.
    kPathHeadingCell,  ///< Every pair bracketing those directions (the enclosing cell per side); the robot
                       ///< model decides the fan-out, and the in-tree robots return one pair.
  };

  /// Which directions the endpoint orientations of a translation edge are scored against.
  TranslationAdjacency p_translation_adjacency_ = TranslationAdjacency::kStraightLine;
};

}  // namespace generator
}  // namespace open_apero
