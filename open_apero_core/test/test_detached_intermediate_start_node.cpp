/**
 * @file test_detached_intermediate_start_node.cpp
 * @brief A start-node update always yields a start node, even where no out-edge can be planned.
 *
 * A graph keeps a detached intermediate start node (zero out-edges) and suspends reachability pruning
 * while the robot cannot stand; node and edge maintenance keep running. A tree falls back to a
 * fresh single-node root.
 */

#include <cstddef>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/generator/tree_generator.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/robot_state.h"
#include "rrag_test_stubs.h"

namespace open_apero {
namespace {

/// What another layer hands the agent: an edge whose endpoint is the only thing the start-node
/// update reads from it.
Edge executedEdge(const RobotModel& robot, const Eigen::Vector3d& from, const RobotState& to) {
  Edge edge;
  edge.trajectory.push_back(makeState(robot, from));
  edge.trajectory.push_back(to);
  edge.is_backtrack = true;
  edge.is_traversable = true;
  setUniformTrajectoryTimes(edge.trajectory, robot.getTimeStep());
  return edge;
}

Edge executedEdge(const RobotModel& robot, const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
  return executedEdge(robot, from, makeState(robot, to));
}

/// The single edge joining any node of @p from to any node of @p to, or nullptr.
Edge* edgeBetweenClusters(const NodeClusterPtrs& from, const NodeClusterPtrs& to, const GraphType* graph) {
  for (Node* from_node : from) {
    for (Node* to_node : to) {
      if (Edge* edge = getEdge(from_node, to_node, graph)) {
        return edge;
      }
    }
  }
  return nullptr;
}

/// The graph fixture the start-node tests share: a start node at the origin and, at the caller's
/// choice, indexed clusters within max_edge_length of it.
class DetachedIntermediateStartNodeTest : public ::testing::Test {
 protected:
  // The generator binds the agent's robot in setupFromParamMap, so it is built after provide().
  DetachedIntermediateStartNodeTest() {
    agent_.provide(&map_, &robot_);
    generator_ = std::make_unique<RefreshProbe>(agent_, Configurable::ParamMap{{"max_edge_length", "6.0"}});
    std::tie(std::ignore, start_node_) = generator_->generateInitNode(makeState(robot_, Eigen::Vector3d::Zero()));
  }

  StubAgent agent_;
  StubMap map_{agent_};
  StationingRobot robot_{agent_};
  std::unique_ptr<RefreshProbe> generator_;
  Node* start_node_ = nullptr;
};

TEST_F(DetachedIntermediateStartNodeTest, InsertionReturnsADetachedStartNodeWhenNoOutEdgeCanBePlanned) {
  NodeClusterPtrs cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  const Eigen::Vector3d hole(1.0, 0.0, 0.0);
  robot_.blocked_positions.push_back(hole);

  Node* new_start_node = generator_->insertIntermediateNodeAtCurrentState(start_node_, executedEdge(robot_, Eigen::Vector3d::Zero(), hole));

  ASSERT_NE(new_start_node, nullptr);
  EXPECT_TRUE(new_start_node->is_intermediate);
  EXPECT_EQ(generator_->outDegree(new_start_node), 0u);
  EXPECT_FALSE(generator_->isIndexed(new_start_node));
  for (Node* node : cluster) {
    EXPECT_TRUE(generator_->containsNode(node)) << "the clusters the start node cannot reach must survive";
    EXPECT_TRUE(generator_->isIndexed(node));
  }
  EXPECT_EQ(generator_->numVertices(), 1u + cluster.size());
}

TEST_F(DetachedIntermediateStartNodeTest, InsertionSupersedesThePreviousIntermediateStartNodeWhenDetached) {
  generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  const Eigen::Vector3d hole(1.0, 0.0, 0.0);
  robot_.blocked_positions.push_back(hole);

  generator_->insertIntermediateNodeAtCurrentState(start_node_, executedEdge(robot_, Eigen::Vector3d::Zero(), hole));

  EXPECT_FALSE(generator_->containsNode(start_node_));
}

TEST_F(DetachedIntermediateStartNodeTest, InsertionStillConnectsATraversableReachedPose) {
  NodeClusterPtrs cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));

  Node* new_start_node = generator_->insertIntermediateNodeAtCurrentState(
      start_node_, executedEdge(robot_, Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0)));

  ASSERT_NE(new_start_node, nullptr);
  EXPECT_EQ(generator_->outDegree(new_start_node), 1u);
  EXPECT_NE(edgeBetweenClusters({new_start_node}, cluster, generator_->graph()), nullptr);
  EXPECT_FALSE(generator_->containsNode(start_node_));
}

TEST_F(DetachedIntermediateStartNodeTest, InsertionSupersedesAPreviousDetachedStartNodeOnTheNextReach) {
  NodeClusterPtrs cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  const Eigen::Vector3d hole(1.0, 0.0, 0.0);
  robot_.blocked_positions.push_back(hole);

  Node* detached_start_node =
      generator_->insertIntermediateNodeAtCurrentState(start_node_, executedEdge(robot_, Eigen::Vector3d::Zero(), hole));
  ASSERT_EQ(generator_->outDegree(detached_start_node), 0u) << "fixture must start detached";

  robot_.blocked_positions.clear();
  Node* new_start_node =
      generator_->insertIntermediateNodeAtCurrentState(detached_start_node, executedEdge(robot_, hole, Eigen::Vector3d(1.5, 0.0, 0.0)));

  ASSERT_NE(new_start_node, nullptr);
  EXPECT_FALSE(generator_->containsNode(detached_start_node));
  EXPECT_EQ(generator_->outDegree(new_start_node), 1u);
  EXPECT_EQ(generator_->numVertices(), 1u + cluster.size());
}

TEST_F(DetachedIntermediateStartNodeTest, InsertionAdoptsAnExistingNodeAtTheReachedPoseAndDropsThePreviousStartNode) {
  NodeClusterPtrs cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));

  Node* new_start_node = generator_->insertIntermediateNodeAtCurrentState(
      start_node_, executedEdge(robot_, Eigen::Vector3d::Zero(), cluster.front()->robot_state));

  EXPECT_EQ(new_start_node, cluster.front());
  EXPECT_FALSE(generator_->containsNode(start_node_));
  EXPECT_EQ(generator_->numVertices(), cluster.size());
}

// The guard has to read the robot model: an intermediate start node is created traversable and never
// refreshed, so its flag cannot tell.
TEST_F(DetachedIntermediateStartNodeTest, DisconnectionPruningIsSuspendedForAnUntraversableStartNode) {
  NodeClusterPtrs near_cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  NodeClusterPtrs far_cluster = generator_->placeCluster(Eigen::Vector3d(-2.0, 0.0, 0.0));
  robot_.blocked_positions.push_back(start_node_->robot_state.getSensorPosition());

  generator_->removeDisconnectedNodeClusters(start_node_);

  for (Node* node : near_cluster) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
  for (Node* node : far_cluster) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
  EXPECT_TRUE(start_node_->is_traversable) << "the start node flag says traversable; only the robot model disagrees";
  EXPECT_TRUE(agent_.warnings.empty());
}

TEST_F(DetachedIntermediateStartNodeTest, DisconnectionPruningStillWipesUnreachedClustersFromATraversableDetachedStartNode) {
  NodeClusterPtrs cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));

  generator_->removeDisconnectedNodeClusters(start_node_);

  for (Node* node : cluster) {
    EXPECT_FALSE(generator_->containsNode(node));
  }
  EXPECT_EQ(generator_->numVertices(), 1u);
  ASSERT_EQ(agent_.warnings.size(), 1u);
  EXPECT_NE(agent_.warnings.front().find("no out-edges"), std::string::npos);
}

TEST_F(DetachedIntermediateStartNodeTest, DisconnectionPruningStillRemovesOnlyTheUnreachedClustersFromAConnectedStartNode) {
  NodeClusterPtrs reached_cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  NodeClusterPtrs unreached_cluster = generator_->placeCluster(Eigen::Vector3d(-2.0, 0.0, 0.0));
  NodeClusterPtrs start_node_cluster = {start_node_};
  ASSERT_FALSE(generator_->generateEdgesFromClusterToCluster(start_node_cluster, reached_cluster).empty());

  generator_->removeDisconnectedNodeClusters(start_node_);

  for (Node* node : reached_cluster) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
  for (Node* node : unreached_cluster) {
    EXPECT_FALSE(generator_->containsNode(node));
  }
  EXPECT_TRUE(agent_.warnings.empty()) << "a connected start node prunes without complaint";
}

TEST_F(DetachedIntermediateStartNodeTest, MaintenanceWithADetachedUntraversableStartNodeStillRemovesClustersThatBecameUntraversable) {
  const Eigen::Vector3d carved(2.0, 0.0, 0.0);
  NodeClusterPtrs cluster = generator_->placeCluster(carved);
  NodeClusterPtrs survivor = generator_->placeCluster(Eigen::Vector3d(-2.0, 0.0, 0.0));
  robot_.blocked_positions = {start_node_->robot_state.getSensorPosition(), carved};

  generator_->updateNodeEdge(start_node_);

  for (Node* node : cluster) {
    EXPECT_FALSE(generator_->containsNode(node)) << "a carved cluster still goes, start node or no start node";
  }
  for (Node* node : survivor) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
}

TEST_F(DetachedIntermediateStartNodeTest, MaintenanceWithADetachedUntraversableStartNodeStillRemovesEdgesThatBecameUntraversable) {
  NodeClusterPtrs from_cluster = generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  NodeClusterPtrs to_cluster = generator_->placeCluster(Eigen::Vector3d(5.0, 0.0, 0.0));
  std::vector<Edge*> inter_cluster_edges = generator_->generateEdgesFromClusterToCluster(from_cluster, to_cluster);
  ASSERT_EQ(inter_cluster_edges.size(), 1u);
  const RobotTrajectory& trajectory = inter_cluster_edges.front()->trajectory;
  const Eigen::Vector3d carved = trajectory[trajectory.size() / 2].getSensorPosition();
  robot_.blocked_positions = {start_node_->robot_state.getSensorPosition(), carved};

  generator_->updateNodeEdge(start_node_);

  EXPECT_EQ(edgeBetweenClusters(from_cluster, to_cluster, generator_->graph()), nullptr);
  for (Node* node : from_cluster) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
  for (Node* node : to_cluster) {
    EXPECT_TRUE(generator_->containsNode(node));
  }
}

// The bootstrap case: the first cluster is born at the start node position, so the only way to
// attach it is by rotation, and the hook is what runs after the cluster exists.
TEST_F(DetachedIntermediateStartNodeTest, ConnectStartNodeAttachesACoincidentClusterByRotationEdges) {
  NodeClusterPtrs cluster = generator_->placeCluster(start_node_->robot_state.getSensorPosition());
  ASSERT_EQ(generator_->outDegree(start_node_), 0u);

  generator_->connectStartNode(start_node_);

  EXPECT_EQ(generator_->outDegree(start_node_), cluster.size() - 1u) << "every candidate but the pose-identical one gets a rotation edge";
  EXPECT_NE(edgeBetweenClusters({start_node_}, cluster, generator_->graph()), nullptr);
  EXPECT_TRUE(StubAgent::contains(agent_.infos, "pose-identical")) << "the skipped twin is reported";
  EXPECT_TRUE(agent_.errors.empty()) << "and it is not an error: the hook reaches this legitimately";
}

TEST_F(DetachedIntermediateStartNodeTest, ConnectStartNodeLeavesAnUntraversableStartNodeDetached) {
  generator_->placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  robot_.blocked_positions.push_back(start_node_->robot_state.getSensorPosition());

  generator_->connectStartNode(start_node_);

  EXPECT_EQ(generator_->outDegree(start_node_), 0u);
}

/// The tree's rerooting path calls computeCost, which needs an owning layer; bypass it.
/// The remaining pure virtuals are sampling hooks rerooting never touches.
class TreeRootProbe : public TreeGenerator {
 public:
  explicit TreeRootProbe(AgentInterface& agent) : TreeGenerator(agent) {
    Configurable::ParamMap params;
    setupFromParamMap(&params);
  }

  bool computeCost(Edge* edge) override {
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }

  NodeClusterPtrs generateNewNode(Node* /*start_node*/) override { return {}; }

  void setNewStartNode(Node* /*previous_start*/, Node* /*new_start*/) override {}

  TreeNode* root() { return root_node_.get(); }
};

class DetachedTreeRootTest : public ::testing::Test {
 protected:
  DetachedTreeRootTest() {
    agent_.provide(&map_, &robot_);
    generator_ = std::make_unique<TreeRootProbe>(agent_);
    std::tie(std::ignore, previous_) = generator_->generateInitNode(makeState(robot_, Eigen::Vector3d::Zero()));
  }

  StubAgent agent_;
  StubMap map_{agent_};
  StationingRobot robot_{agent_};
  std::unique_ptr<TreeRootProbe> generator_;
  Node* previous_ = nullptr;
};

TEST_F(DetachedTreeRootTest, TreeRerootReturnsAFreshSingleNodeRootWhenTheEdgeCannotBePlanned) {
  const Eigen::Vector3d reached(2.0, 0.0, 0.0);
  // The edge back to the old root has to cross the pose the old root stands at.
  robot_.blocked_positions.push_back(Eigen::Vector3d::Zero());

  Node* new_root = generator_->insertIntermediateNodeAtCurrentState(previous_, executedEdge(robot_, Eigen::Vector3d::Zero(), reached));

  ASSERT_NE(new_root, nullptr);
  EXPECT_EQ(new_root, generator_->root());
  EXPECT_NE(new_root, previous_);
  EXPECT_TRUE(new_root->is_intermediate);
  EXPECT_TRUE(generator_->root()->children.empty()) << "the unreachable subtree is dropped, not reparented";
  EXPECT_TRUE(new_root->robot_state.getSensorPosition().isApprox(reached));
}

TEST_F(DetachedTreeRootTest, TreeRerootStillReparentsThePreviousRootWhenTheEdgeCanBePlanned) {
  Node* new_root = generator_->insertIntermediateNodeAtCurrentState(
      previous_, executedEdge(robot_, Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, 0.0, 0.0)));

  ASSERT_NE(new_root, nullptr);
  EXPECT_EQ(new_root, generator_->root());
  ASSERT_EQ(generator_->root()->children.size(), 1u);
  EXPECT_EQ(generator_->root()->children.front().get(), previous_);
  EXPECT_EQ(previous_->as<TreeNode>()->parent, generator_->root());
}

}  // namespace
}  // namespace open_apero
