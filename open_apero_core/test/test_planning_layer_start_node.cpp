/**
 * @file test_planning_layer_start_node.cpp
 * @brief The planning layer moves its start node without re-seeding, and says why it declines.
 *
 * An arrival at a vertex of the graph moves the start node onto it once; a tick without an executed
 * edge keeps it. A nullptr from insertIntermediateNodeAtCurrentState is a contract violation; a start
 * node without out-edges is connected after expansion (GraphGenerator::connectStartNode), and the
 * planPath gates (no traversable node besides the start node, no out-edge, no path) log why they
 * decline. The tests drive the agent's order: onStateReached, update, expand, planPath.
 */

#include <algorithm>
#include <climits>
#include <cstddef>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/layer/planning_layer.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/state/robot_state.h"
#include "rrag_test_stubs.h"

namespace open_apero {
namespace {

class TestEvaluator : public Evaluator {
 public:
  explicit TestEvaluator(AgentInterface& agent) : Evaluator(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
  double getRealAccumulatedGain() const override { return 0.0; }

 protected:
  bool evaluateUnobserved(const NodeClusterPtrs& /*cluster*/) override { return true; }
};

/// Offers the first step the graph actually holds, which is what makes a decline attributable
/// to the layer's gates rather than to the planner's own search.
class FirstNeighborPathPlanner : public PathPlanner {
 public:
  explicit FirstNeighborPathPlanner(AgentInterface& agent) : PathPlanner(agent) {}

  std::pair<std::vector<Node*>, double> planPath(Node* start_node, GraphType* graph, Node* /*required_terminal_node*/) override {
    searched_graph = graph;
    if (decline || graph == nullptr) {
      return {{}, 0.0};
    }
    const NodeId start_id = getNodeId(start_node, graph);
    for (const auto& edge_id : boost::make_iterator_range(boost::out_edges(start_id, *graph))) {
      return {{start_node, (*graph)[boost::target(edge_id, *graph)].get()}, 1.0};
    }
    return {{}, 0.0};
  }

  bool decline = false;
  /// The graph of the last search; nullptr before the first.
  GraphType* searched_graph = nullptr;
};

/// Records every start-node update, including those the generator runs inside an intermediate insertion.
class CountingRefreshProbe : public RefreshProbe {
 public:
  using RefreshProbe::RefreshProbe;

  void setNewStartNode(Node* previous_start, Node* new_start) override {
    start_node_updates.emplace_back(previous_start, new_start);
    RefreshProbe::setNewStartNode(previous_start, new_start);
  }

  std::vector<std::pair<Node*, Node*>> start_node_updates;
};

/// The layer with its three plugins wired by hand: no expansion (clusters are placed
/// directly) and no replanning split, so a cycle is exactly the start-node update plus maintenance.
class StartNodeLayerProbe : public layer::PlanningLayer {
 public:
  explicit StartNodeLayerProbe(AgentInterface& agent) : layer::PlanningLayer(agent) {
    auto generator = std::make_unique<CountingRefreshProbe>(agent, Configurable::ParamMap{{"max_edge_length", "6.0"}});
    generator_probe_ = generator.get();
    generator->setLayer(this);
    generator_ = std::move(generator);

    auto evaluator = std::make_unique<TestEvaluator>(agent);
    evaluator->setLayer(this);
    evaluator_ = std::move(evaluator);

    auto planner = std::make_unique<FirstNeighborPathPlanner>(agent);
    path_planner_probe_ = planner.get();
    planner->setLayer(this);
    path_planner_ = std::move(planner);

    p_max_expansions_ = 0;
    p_replan_num_steps_ = INT_MAX;
    setInstanceName("start_node_test");
  }

  CountingRefreshProbe& generatorProbe() { return *generator_probe_; }
  FirstNeighborPathPlanner& pathPlannerProbe() { return *path_planner_probe_; }

 private:
  CountingRefreshProbe* generator_probe_ = nullptr;
  FirstNeighborPathPlanner* path_planner_probe_ = nullptr;
};

class PlanningLayerStartNodeTest : public ::testing::Test {
 protected:
  // The layer's plugins bind the agent's robot in setupFromParamMap, so it is built after provide().
  PlanningLayerStartNodeTest() {
    agent_.provide(&map_, &robot_);
    layer_ = std::make_unique<StartNodeLayerProbe>(agent_);
    layer_->onEpisodeStart(makeState(robot_, Eigen::Vector3d::Zero()));
  }

  /// The recovery edge and foreign intermediate node BacktrackingLayer hands the agent.
  void reach(const Eigen::Vector3d& position) {
    executed_edge_ = Edge();
    executed_edge_.trajectory.push_back(makeState(robot_, layer_->getCurrentNode()->robot_state.getSensorPosition()));
    executed_edge_.trajectory.push_back(makeState(robot_, position));
    executed_edge_.is_backtrack = true;
    executed_edge_.is_traversable = true;
    setUniformTrajectoryTimes(executed_edge_.trajectory, robot_.getTimeStep());

    active_node_ = std::make_shared<Node>();
    active_node_->robot_state = executed_edge_.trajectory.back();
    active_node_->is_intermediate = true;
    active_node_->is_traversable = true;

    layer_->onStateReached(active_node_->robot_state, &executed_edge_, active_node_.get());
  }

  bool planPath(std::vector<Node*>* planned_path, std::vector<Edge*>* planned_edges) {
    return layer_->planPath(layer_->getCurrentNode()->robot_state, &map_, &robot_, planned_path, planned_edges);
  }

  StubAgent agent_;
  StubMap map_{agent_};
  StationingRobot robot_{agent_};
  std::unique_ptr<StartNodeLayerProbe> layer_;
  Edge executed_edge_;
  std::shared_ptr<Node> active_node_;
};

TEST_F(PlanningLayerStartNodeTest, AnArrivalAtTheStepTargetMovesTheStartNodeOntoItOnce) {
  Node* start_node = layer_->getCurrentNode();
  layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  layer_->update();
  layer_->expand();
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;
  ASSERT_TRUE(planPath(&planned_path, &planned_edges));
  ASSERT_EQ(planned_path.size(), 2u);
  Node* target = planned_path[1];
  const Edge executed = *planned_edges.front();
  layer_->generatorProbe().start_node_updates.clear();

  layer_->onStateReached(target->robot_state, &executed, target);

  ASSERT_EQ(layer_->generatorProbe().start_node_updates.size(), 1u);
  EXPECT_EQ(layer_->generatorProbe().start_node_updates.front(), std::make_pair(start_node, target));
  EXPECT_EQ(layer_->getCurrentNode(), target);
  EXPECT_TRUE(target->is_observed) << "the evaluator spends the reached node";
}

// The episode's first tick: no edge has run, so the start node stays wherever the reported state is.
TEST_F(PlanningLayerStartNodeTest, ATickWithoutAnExecutedEdgeKeepsTheStartNode) {
  Node* start_node = layer_->getCurrentNode();

  layer_->onStateReached(makeState(robot_, Eigen::Vector3d(0.5, 0.0, 0.0)), nullptr, nullptr);

  EXPECT_EQ(layer_->getCurrentNode(), start_node);
  EXPECT_TRUE(layer_->generatorProbe().start_node_updates.empty());
}

TEST_F(PlanningLayerStartNodeTest, ABacktrackEndingInAHoleKeepsTheGraphAndTheNextTraversableReachPlansImmediately) {
  NodeClusterPtrs cluster = layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  const int node_count_before = layer_->getNodeCount();
  const Eigen::Vector3d hole(1.0, 0.0, 0.0);
  robot_.blocked_positions.push_back(hole);

  reach(hole);
  layer_->update();
  layer_->expand();
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_FALSE(planPath(&planned_path, &planned_edges));
  EXPECT_EQ(layer_->getNodeCount(), node_count_before) << "the graph outlives the hole";
  EXPECT_TRUE(StubAgent::contains(agent_.infos, "declined: start node has no out-edges"));

  robot_.blocked_positions.clear();
  const Eigen::Vector3d escaped(1.5, 0.0, 0.0);
  reach(escaped);
  layer_->update();
  layer_->expand();
  agent_.infos.clear();

  ASSERT_TRUE(planPath(&planned_path, &planned_edges));
  ASSERT_EQ(planned_path.size(), 2u);
  EXPECT_EQ(planned_path.front(), layer_->getCurrentNode());
  EXPECT_NE(std::find(cluster.begin(), cluster.end(), planned_path.back()), cluster.end());
  ASSERT_EQ(planned_edges.size(), 1u);
  EXPECT_TRUE(planned_edges.front()->trajectory.front().getSensorPosition().isApprox(escaped));
  EXPECT_FALSE(StubAgent::contains(agent_.infos, "declined"));
}

TEST_F(PlanningLayerStartNodeTest, AReachedPoseWithoutAnyClusterInRangeStillYieldsACurrentNode) {
  Node* previous_start_node = layer_->getCurrentNode();

  reach(Eigen::Vector3d(1.0, 0.0, 0.0));

  Node* current = layer_->getCurrentNode();
  ASSERT_NE(current, nullptr);
  EXPECT_NE(current, previous_start_node);
  EXPECT_NE(current, active_node_.get()) << "the foreign node is not imported, only its state";
  EXPECT_TRUE(current->is_intermediate);
  EXPECT_TRUE(layer_->generatorProbe().containsNode(current));
  EXPECT_EQ(layer_->getNodeCount(), 1);
}

// Inserting the reached state moves the start node as setNewStartNode does, so it is charged to the same timer.
TEST_F(PlanningLayerStartNodeTest, InsertingAStateOffTheGraphIsChargedToStartNodeUpdate) {
  agent_.setLoggingPerformance(true);

  reach(Eigen::Vector3d(1.0, 0.0, 0.0));

  EXPECT_GT(layer_->getMetrics().start_node_update_s, 0.0);
}

TEST_F(PlanningLayerStartNodeTest, PlanPathLogsThatTheStartNodeHasNoOutEdges) {
  layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  const std::string node_count = std::to_string(layer_->getNodeCount());
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_FALSE(planPath(&planned_path, &planned_edges));

  EXPECT_TRUE(StubAgent::contains(agent_.infos, "PlanningLayer planPath declined: start node has no out-edges (" + node_count + " nodes)"));
}

TEST_F(PlanningLayerStartNodeTest, PlanPathLogsWhenNoNodeBesidesTheStartNodeIsTraversable) {
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_FALSE(planPath(&planned_path, &planned_edges));

  EXPECT_TRUE(StubAgent::contains(agent_.infos, "PlanningLayer planPath declined: no traversable node besides the start node (1 nodes)"));
}

// The start node's flag is a verdict about standing there, not a gate on leaving: a demoted start node still
// plans to its one traversable neighbour. One, so that a count including the start node would decline.
TEST_F(PlanningLayerStartNodeTest, PlanPathProceedsFromADemotedStartNodeToASingleTraversableNode) {
  NodeClusterPtrs cluster = layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  layer_->expand();
  ASSERT_GT(layer_->generatorProbe().outDegree(layer_->getCurrentNode()), 0u) << "connectStartNode must have connected the start node";
  layer_->getCurrentNode()->is_traversable = false;
  for (std::size_t i = 1; i < cluster.size(); ++i) {
    cluster[i]->is_traversable = false;
  }
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_TRUE(planPath(&planned_path, &planned_edges));
}

TEST_F(PlanningLayerStartNodeTest, PlanPathLogsWhenThePathPlannerReturnsNothing) {
  layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  layer_->expand();
  ASSERT_GT(layer_->generatorProbe().outDegree(layer_->getCurrentNode()), 0u) << "connectStartNode must have connected the start node";
  layer_->pathPlannerProbe().decline = true;
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_FALSE(planPath(&planned_path, &planned_edges));

  EXPECT_TRUE(StubAgent::contains(agent_.infos, "PlanningLayer planPath declined: path planner returned nothing"));
}

// connectStartNode makes a cluster born this cycle plannable in the same cycle, although update(),
// which also connects the start node, runs before expand().
TEST_F(PlanningLayerStartNodeTest, ExpansionConnectsAStartNodeThatHasNoOutEdges) {
  layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  ASSERT_EQ(layer_->generatorProbe().outDegree(layer_->getCurrentNode()), 0u);

  layer_->expand();

  EXPECT_GT(layer_->generatorProbe().outDegree(layer_->getCurrentNode()), 0u);
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;
  EXPECT_TRUE(planPath(&planned_path, &planned_edges));
}

// The counts follow the agent's order: the start-node update in onStateReached, then update and expansion.
TEST_F(PlanningLayerStartNodeTest, CycleSummaryReportsTheCountsInCycleOrder) {
  layer_->onStateReached(layer_->getCurrentNode()->robot_state, nullptr, nullptr);
  layer_->update();
  layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  layer_->expand();

  EXPECT_EQ(layer_->cycleSummary(), "[1] ---update and expansion: +4---> [5] nodes at selection");
}

TEST_F(PlanningLayerStartNodeTest, WarmUpSearchesTheLayersGraph) {
  layer_->generatorProbe().placeCluster(Eigen::Vector3d(2.0, 0.0, 0.0));
  layer_->expand();
  ASSERT_GT(layer_->generatorProbe().outDegree(layer_->getCurrentNode()), 0u);

  layer_->warmUp();

  EXPECT_EQ(layer_->pathPlannerProbe().searched_graph, layer_->generatorProbe().graph());
}

TEST_F(PlanningLayerStartNodeTest, WarmUpSkipsTheSearchFromAStartNodeWithoutOutEdges) {
  layer_->warmUp();

  EXPECT_EQ(layer_->pathPlannerProbe().searched_graph, nullptr);
}

}  // namespace
}  // namespace open_apero
