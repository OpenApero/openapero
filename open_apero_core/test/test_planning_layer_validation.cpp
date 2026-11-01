/**
 * @file test_planning_layer_validation.cpp
 * @brief The planning layer validates a refined trajectory before it splits, costs or returns it,
 *        and visualizes the selected path's first evaluated node.
 */

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/rrag.h"
#include "open_apero_core/plugin/layer/planning_layer.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

RobotState makePlanningState(double x) {
  RobotState state;
  const Eigen::Vector3d position(x, 0.0, 0.0);
  state.setSensorPosition(position);
  state.setBasePosition(position);
  return state;
}

class PlanningTestMap : public Map {
 public:
  explicit PlanningTestMap(AgentInterface& agent) : Map(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
};

/// ConfigAgent with seed 3.
class PlanningTestAgent : public ConfigAgent {
 public:
  PlanningTestAgent() { setRandomSeed(3); }
};

class RefiningRobotProbe : public RobotModel {
 public:
  explicit RefiningRobotProbe(AgentInterface& agent) : RobotModel(agent) {
    Configurable::ParamMap params{{"safe_radius", "0.0"}};
    setupFromParamMap(&params);
  }

  bool planTrajectory(const RobotState& /*start*/, const RobotState& /*goal*/, Map* /*map*/, RobotTrajectory* /*trajectory*/) override {
    return false;
  }

  RobotTrajectory steer(const RobotState& /*start*/, const RobotState& /*goal*/, Map* /*map*/) const override { return {}; }

  Traversability placeRobotAtSensorPose(RobotState* robot_state, const Transform3d& sensor_transform, Map* /*map*/) const override {
    robot_state->setSensorTransform(sensor_transform);
    robot_state->setBaseTransform(sensor_transform);
    return Traversability::kUnset;
  }

  void refineBeforeExecute(RobotTrajectory& trajectory, int /*num_committed_states*/) override {
    ++refine_calls;
    if (sabotage_middle && trajectory.size() > 2u) {
      trajectory[1].setSensorPosition(Eigen::Vector3d(blocked_x, 0.0, 0.0));
      trajectory[1].setBasePosition(Eigen::Vector3d(blocked_x, 0.0, 0.0));
    }
  }

  bool isTrajectoryTraversable(const RobotTrajectory& trajectory, Map* map) const override {
    if (force_trajectory_failure) {
      return false;
    }
    return RobotModel::isTrajectoryTraversable(trajectory, map);
  }

  Traversability getStateTraversabilityRaw(const RobotState& state, Map* /*map*/) const override {
    return std::abs(state.getSensorPosition().x() - blocked_x) < 1e-9 ? Traversability::kUntraversable : Traversability::kTraversable;
  }

  bool isTraversabilityOrientationRelated() const override { return false; }

  bool sabotage_middle = false;
  bool force_trajectory_failure = false;
  double blocked_x = 99.0;
  int refine_calls = 0;
};

class FixedPathPlanner : public PathPlanner {
 public:
  explicit FixedPathPlanner(AgentInterface& agent) : PathPlanner(agent) {}

  std::pair<std::vector<Node*>, double> planPath(Node* /*start_node*/, GraphType* /*graph*/, Node* /*required_terminal_node*/) override {
    return {{start_, goal_}, value_};
  }

  void setPath(Node* start, Node* goal, double value = 42.0) {
    start_ = start;
    goal_ = goal;
    value_ = value;
  }

 private:
  Node* start_ = nullptr;
  Node* goal_ = nullptr;
  double value_ = 0.0;
};

class GeneratorProbe : public generator::RRAG {
 public:
  explicit GeneratorProbe(AgentInterface& agent) : generator::RRAG(agent) {
    Configurable::ParamMap params{{"param_namespace", "/planning_validation_generator"}};
    setupFromParamMap(&params);
  }

  std::pair<Node*, Edge*> addCandidate(Node* start, const RobotTrajectory& trajectory) {
    auto goal = std::make_shared<GraphNode>();
    goal->robot_state = trajectory.back();
    goal->is_traversable = true;
    const NodeId goal_id = boost::add_vertex(goal, *graph_);

    auto edge = std::make_shared<Edge>();
    edge->trajectory = trajectory;
    edge->is_traversable = true;
    const NodeId start_id = getNodeId(start, graph_.get());
    boost::add_edge(start_id, goal_id, edge, *graph_);
    return {goal.get(), edge.get()};
  }

  std::pair<Node*, Edge*> insertIntermediateNode(Node* start_node, Node* goal_node, Edge* edge, int node_index) override {
    ++split_calls;
    return generator::RRAG::insertIntermediateNode(start_node, goal_node, edge, node_index);
  }

  bool computeCost(Edge* edge) override {
    ++cost_calls;
    edge->cost = static_cast<double>(edge->trajectory.size());
    return true;
  }

  std::size_t edgeCount() const { return boost::num_edges(*graph_); }

  Edge* edgeBetween(Node* start, Node* goal) const { return getEdge(start, goal, graph_.get()); }

  int split_calls = 0;
  int cost_calls = 0;
};

/// Records the nodes it is asked to visualize.
class VisualizationRecordingEvaluator : public Evaluator {
 public:
  explicit VisualizationRecordingEvaluator(AgentInterface& agent) : Evaluator(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
  double getRealAccumulatedGain() const override { return 0.0; }
  void getVisualization(VisualizationMarkers* /*markers*/, const Node* node) override { visualized.push_back(node); }

  std::vector<const Node*> visualized;

 protected:
  bool evaluateUnobserved(const NodeClusterPtrs& /*cluster*/) override { return true; }
};

class PlanningLayerProbe : public layer::PlanningLayer {
 public:
  explicit PlanningLayerProbe(AgentInterface& agent, int replan_num_steps) : layer::PlanningLayer(agent) {
    auto generator = std::make_unique<GeneratorProbe>(agent);
    generator_probe_ = generator.get();
    generator->setLayer(this);
    generator_ = std::move(generator);

    auto evaluator = std::make_unique<VisualizationRecordingEvaluator>(agent);
    evaluator_probe_ = evaluator.get();
    evaluator->setLayer(this);
    evaluator_ = std::move(evaluator);

    auto planner = std::make_unique<FixedPathPlanner>(agent);
    path_planner_probe_ = planner.get();
    planner->setLayer(this);
    path_planner_ = std::move(planner);

    p_replan_num_steps_ = replan_num_steps;
    setInstanceName("validation_test");
  }

  std::pair<Node*, Edge*> initialize(const RobotTrajectory& trajectory) {
    onEpisodeStart(trajectory.front());
    auto candidate = generator_probe_->addCandidate(current_node_, trajectory);
    path_planner_probe_->setPath(current_node_, candidate.first);
    return candidate;
  }

  GeneratorProbe& generatorProbe() { return *generator_probe_; }
  VisualizationRecordingEvaluator& evaluatorProbe() { return *evaluator_probe_; }

 private:
  GeneratorProbe* generator_probe_ = nullptr;
  VisualizationRecordingEvaluator* evaluator_probe_ = nullptr;
  FixedPathPlanner* path_planner_probe_ = nullptr;
};

class PlanningLayerValidationTest : public ::testing::Test {
 protected:
  PlanningLayerValidationTest() : map_(agent_), robot_(agent_) { agent_.provide(&map_, &robot_); }

  static RobotTrajectory trajectory(std::size_t size) {
    RobotTrajectory result;
    for (std::size_t i = 0; i < size; ++i) {
      result.push_back(makePlanningState(static_cast<double>(i)));
    }
    return result;
  }

  PlanningTestAgent agent_;
  PlanningTestMap map_;
  RefiningRobotProbe robot_;
};

TEST_F(PlanningLayerValidationTest, InvalidRefinementDeclinesBeforeGraphMutationOrCosting) {
  PlanningLayerProbe layer(agent_, 2);
  RobotTrajectory candidate_trajectory = trajectory(5);
  robot_.blocked_x = 99.0;
  robot_.sabotage_middle = true;
  const auto candidate = layer.initialize(candidate_trajectory);
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_FALSE(layer.planPath(candidate_trajectory.front(), &map_, &robot_, &planned_path, &planned_edges));

  EXPECT_TRUE(planned_path.empty());
  EXPECT_TRUE(planned_edges.empty());
  EXPECT_FALSE(candidate.second->is_traversable);
  EXPECT_DOUBLE_EQ(candidate.second->trajectory[1].getSensorPosition().x(), 99.0);
  EXPECT_EQ(layer.generatorProbe().edgeBetween(layer.getCurrentNode(), candidate.first), candidate.second);
  EXPECT_EQ(layer.generatorProbe().edgeCount(), 1u);
  EXPECT_EQ(layer.generatorProbe().split_calls, 0);
  EXPECT_EQ(layer.generatorProbe().cost_calls, 0);
  EXPECT_DOUBLE_EQ(layer.getMetrics().planned_value, 42.0);
  ASSERT_EQ(agent_.warnings.size(), 1u);
  EXPECT_NE(agent_.warnings.front().find("PlanningLayer:validation_test"), std::string::npos);
  EXPECT_NE(agent_.warnings.front().find("waypoint_idx=1/5"), std::string::npos);
  EXPECT_NE(agent_.warnings.front().find("sensor_pos=[99"), std::string::npos);
}

TEST_F(PlanningLayerValidationTest, ValidLongTrajectoryIsSplitAfterValidationAndCosted) {
  PlanningLayerProbe layer(agent_, 2);
  RobotTrajectory candidate_trajectory = trajectory(5);
  layer.initialize(candidate_trajectory);
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  ASSERT_TRUE(layer.planPath(candidate_trajectory.front(), &map_, &robot_, &planned_path, &planned_edges));

  EXPECT_EQ(robot_.refine_calls, 1);
  EXPECT_EQ(layer.generatorProbe().split_calls, 1);
  EXPECT_EQ(layer.generatorProbe().cost_calls, 3);
  ASSERT_EQ(planned_path.size(), 3u);
  ASSERT_EQ(planned_edges.size(), 2u);
  EXPECT_EQ(planned_edges.front()->trajectory.size(), 3u);
}

TEST_F(PlanningLayerValidationTest, ValidShortTrajectoryRemainsUnsplitAndIsCosted) {
  PlanningLayerProbe layer(agent_, 10);
  RobotTrajectory candidate_trajectory = trajectory(3);
  const auto candidate = layer.initialize(candidate_trajectory);
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  ASSERT_TRUE(layer.planPath(candidate_trajectory.front(), &map_, &robot_, &planned_path, &planned_edges));

  EXPECT_EQ(layer.generatorProbe().split_calls, 1);
  EXPECT_EQ(layer.generatorProbe().cost_calls, 1);
  ASSERT_EQ(planned_path.size(), 2u);
  ASSERT_EQ(planned_edges.size(), 1u);
  EXPECT_EQ(planned_edges.front(), candidate.second);
}

TEST_F(PlanningLayerValidationTest, NullRobotSkipsRefinementAndValidation) {
  PlanningLayerProbe layer(agent_, 10);
  RobotTrajectory candidate_trajectory = trajectory(3);
  layer.initialize(candidate_trajectory);
  robot_.force_trajectory_failure = true;
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_TRUE(layer.planPath(candidate_trajectory.front(), &map_, nullptr, &planned_path, &planned_edges));
  EXPECT_EQ(robot_.refine_calls, 0);
  EXPECT_TRUE(agent_.warnings.empty());
}

TEST_F(PlanningLayerValidationTest, TrajectoryLevelFailureUsesGenericDiagnostic) {
  PlanningLayerProbe layer(agent_, 10);
  RobotTrajectory candidate_trajectory = trajectory(3);
  const auto candidate = layer.initialize(candidate_trajectory);
  robot_.force_trajectory_failure = true;
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  EXPECT_FALSE(layer.planPath(candidate_trajectory.front(), &map_, &robot_, &planned_path, &planned_edges));

  EXPECT_FALSE(candidate.second->is_traversable);
  EXPECT_EQ(layer.generatorProbe().split_calls, 0);
  EXPECT_EQ(layer.generatorProbe().cost_calls, 0);
  ASSERT_EQ(agent_.warnings.size(), 1u);
  EXPECT_NE(agent_.warnings.front().find("trajectory-level validation"), std::string::npos);
  EXPECT_NE(agent_.warnings.front().find("no individual invalid state found"), std::string::npos);
}

TEST_F(PlanningLayerValidationTest, VisualizationShowsTheEvaluatedNodePastTheSplit) {
  PlanningLayerProbe layer(agent_, 2);
  RobotTrajectory candidate_trajectory = trajectory(5);
  const auto candidate = layer.initialize(candidate_trajectory);
  candidate.first->info = std::make_unique<NodeInfo>();
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  ASSERT_TRUE(layer.planPath(candidate_trajectory.front(), &map_, &robot_, &planned_path, &planned_edges));
  ASSERT_EQ(planned_path.size(), 3u);
  ASSERT_EQ(planned_path[1]->info, nullptr);

  VisualizationMarkers markers;
  layer.getVisualization(planned_path[1], &markers);

  EXPECT_EQ(layer.evaluatorProbe().visualized, std::vector<const Node*>{candidate.first});
}

TEST_F(PlanningLayerValidationTest, VisualizationShowsTheSelectedNodeWhenNoneIsEvaluated) {
  PlanningLayerProbe layer(agent_, 2);
  RobotTrajectory candidate_trajectory = trajectory(5);
  layer.initialize(candidate_trajectory);
  std::vector<Node*> planned_path;
  std::vector<Edge*> planned_edges;

  ASSERT_TRUE(layer.planPath(candidate_trajectory.front(), &map_, &robot_, &planned_path, &planned_edges));

  VisualizationMarkers markers;
  layer.getVisualization(planned_path[1], &markers);

  EXPECT_EQ(layer.evaluatorProbe().visualized, std::vector<const Node*>{planned_path[1]});
}

}  // namespace
}  // namespace open_apero
