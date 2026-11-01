/**
 * @file test_layer_stack.cpp
 * @brief Tests for Agent's "layers" dictionary and the strict PluginFactory
 *        contract, driven through the real setupFromParamMap /
 *        requestNextTrajectory code paths.
 */

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent.h"
#include "open_apero_core/common/frames.h"
#include "open_apero_core/logging/logging.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/generator.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/plugin_factory_config.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/observed_viewpoints.h"

namespace open_apero {
namespace {

/// @brief Order in which layers were consulted during arbitration, by instance name.
std::vector<std::string>& consultationOrder() {
  static std::vector<std::string> order;
  return order;
}

// The tests drive the production PluginFactoryConfig (the ROS-free in-memory
// factory) so its getParamMapAndType/listChildNamespaces contract is exercised
// directly, not a bespoke test double.

// ---------------------------------------------------------------------------
// Minimal plugins. The agent builds a Map and a RobotModel before the layer
// stack, so both need a registered stand-in.
// ---------------------------------------------------------------------------
class TestMap : public Map {
 public:
  explicit TestMap(AgentInterface& agent) : Map(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override { ++reset_episode_calls_; }
  bool resetMapUpdateStatus() override {
    ++map_update_reset_calls_;
    return true;
  }
  int mapUpdateResetCalls() const { return map_update_reset_calls_; }
  int resetEpisodeCalls() const { return reset_episode_calls_; }

  /// @brief Stands in for the frames the map has integrated. Empty by default, which
  /// is what the base Map reports and what keeps the map-update gate blocking.
  bool getMappedOpticalFrameTransforms(std::vector<Transform3d>* transforms) override {
    if (mapped_frames_.empty()) {
      return false;
    }
    *transforms = mapped_frames_;
    return true;
  }

  /// @brief Stands in for a backend whose readiness needs more than the frame list:
  /// while a frame is pending, an integrated frame does not yet satisfy the gate.
  bool hasMappedFrameNear(const Transform3d& optical_frame, double position_tolerance, double angle_tolerance) override {
    return !frame_pending_ && Map::hasMappedFrameNear(optical_frame, position_tolerance, angle_tolerance);
  }

  void setMappedOpticalFrames(std::vector<Transform3d> frames) { mapped_frames_ = std::move(frames); }

  void setFramePending(bool pending) { frame_pending_ = pending; }

  /// @brief Records the snapshot path instead of writing a file.
  bool saveMap(const std::string& filename) override {
    saved_maps_.push_back(filename);
    return true;
  }
  const std::vector<std::string>& savedMaps() const { return saved_maps_; }

  static PluginRegistry::Registration<TestMap> registration;

 private:
  int map_update_reset_calls_{0};
  int reset_episode_calls_{0};
  std::vector<Transform3d> mapped_frames_;
  bool frame_pending_{false};
  std::vector<std::string> saved_maps_;
};
PluginRegistry::Registration<TestMap> TestMap::registration("TestMap");

class TestRobot : public RobotModel {
 public:
  explicit TestRobot(AgentInterface& agent) : RobotModel(agent) {}
  // Delegate to the base so its params (max_linear_velocity, etc.) get their
  // defaults; otherwise they stay uninitialized and checkParamsValid trips.
  void setupFromParamMap(ParamMap* param_map) override { RobotModel::setupFromParamMap(param_map); }
  void resetEpisode() override {}
  bool planTrajectory(const RobotState& /*start*/, const RobotState& /*goal*/, Map* /*map*/, RobotTrajectory* /*trajectory*/) override {
    return false;
  }
  RobotTrajectory steer(const RobotState& /*start*/, const RobotState& /*goal*/, Map* /*map*/) const override { return {}; }
  Traversability placeRobotAtSensorPose(RobotState* /*robot_state*/, const Transform3d& /*sensor_transform*/, Map* /*map*/) const override {
    return Traversability::kTraversable;
  }
  Traversability getStateTraversabilityRaw(const RobotState& /*state*/, Map* /*map*/) const override {
    return Traversability::kTraversable;
  }
  bool isTraversabilityOrientationRelated() const override { return false; }
  static PluginRegistry::Registration<TestRobot> registration;
};
PluginRegistry::Registration<TestRobot> TestRobot::registration("TestRobot");

// Minimal generator/evaluator so a layer can run the real Layer::setupFromParamMap,
// which composes generator -> evaluator -> path_planner children.
class TestGenerator : public Generator {
 public:
  explicit TestGenerator(AgentInterface& agent) : Generator(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
  bool rebuildKdTree() override { return true; }
  NodeClusterPtrs generateNewNode(Node* /*start_node*/) override { return {}; }
  static PluginRegistry::Registration<TestGenerator> registration;
};
PluginRegistry::Registration<TestGenerator> TestGenerator::registration("TestGenerator");

class TestEvaluator : public Evaluator {
 public:
  explicit TestEvaluator(AgentInterface& agent) : Evaluator(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
  double getRealAccumulatedGain() const override { return 0.0; }
  static PluginRegistry::Registration<TestEvaluator> registration;

 protected:
  bool evaluateUnobserved(const NodeClusterPtrs& /*cluster*/) override { return true; }
};
PluginRegistry::Registration<TestEvaluator> TestEvaluator::registration("TestEvaluator");

/// @brief Layer stand-in that records arbitration order and never offers a step,
/// so requestNextTrajectory exercises the priority sort without the downstream
/// execution machinery.
class TestLayerBase : public Layer {
 public:
  explicit TestLayerBase(AgentInterface& agent) : Layer(agent) {}

  // Deliberately does NOT call Layer::setupFromParamMap: these fakes compose no
  // roles, so only the priority is read.
  void setupFromParamMap(Configurable::ParamMap* param_map) override {
    setParam<int>(param_map, "priority", &p_priority_, defaultPriority());
  }

  void onStateReached(const RobotState& /*state*/, const Edge* /*executed_edge*/, Node* /*active_node*/) override {}
  void expand() override {}

  bool planPath(const RobotState& /*current_state*/, Map* /*map*/, RobotModel* /*robot*/, std::vector<Node*>* /*planned_path*/,
                std::vector<Edge*>* /*planned_edges*/) override {
    consultationOrder().push_back(instanceName());
    return false;  // never wins: we only observe the order layers are consulted in
  }
};

class TestLayerAlpha : public TestLayerBase {
 public:
  explicit TestLayerAlpha(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "alpha"; }
  static PluginRegistry::Registration<TestLayerAlpha> registration;
};
PluginRegistry::Registration<TestLayerAlpha> TestLayerAlpha::registration("TestLayerAlpha");

class TestLayerBeta : public TestLayerBase {
 public:
  explicit TestLayerBeta(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "beta"; }
  static PluginRegistry::Registration<TestLayerBeta> registration;
};
PluginRegistry::Registration<TestLayerBeta> TestLayerBeta::registration("TestLayerBeta");

/// @brief Lets a test layer sample the agent's active-cursor edge from inside a
/// callback. The fixture installs it; TestAgent (defined below) supplies the
/// accessor, so the layers here need no knowledge of that type.
std::function<const Edge*()>& cursorProbe() {
  static std::function<const Edge*()> probe;
  return probe;
}

/// @brief Layer that owns its edge and frees it from a callback, the way
/// BacktrackingLayer replaces recovery_edge_. Samples the agent's cursor in every
/// callback so the test can assert no stale pointer to the freed edge is ever exposed.
class TestEdgeFreeingLayer : public TestLayerBase {
 public:
  explicit TestEdgeFreeingLayer(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "edge_freeing"; }

  /// @brief Edge the layer owns and will free; the test drives the agent with it.
  static std::unique_ptr<Edge>& ownedEdge() {
    static std::unique_ptr<Edge> edge;
    return edge;
  }
  /// @brief Agent cursor edge sampled once per callback, in call order.
  static std::vector<const Edge*>& sampledCursorEdges() {
    static std::vector<const Edge*> edges;
    return edges;
  }

  void onStateReached(const RobotState& /*state*/, const Edge* /*executed_edge*/, Node* /*active_node*/) override {
    sample();
    ownedEdge().reset();  // the destructive act the agent must already be safe against
  }
  void update() override { sample(); }

  bool planPath(const RobotState& /*current_state*/, Map* /*map*/, RobotModel* /*robot*/, std::vector<Node*>* /*planned_path*/,
                std::vector<Edge*>* /*planned_edges*/) override {
    consultationOrder().push_back(instanceName());
    sample();
    return false;  // never wins, so the agent terminates: expected by the test
  }

 private:
  void sample() {
    if (cursorProbe()) {
      sampledCursorEdges().push_back(cursorProbe()());
    }
  }
};
PluginRegistry::Registration<TestEdgeFreeingLayer> registration_edge_freeing("TestEdgeFreeingLayer");

/// @brief Layer that declares a state-dependent promotion via possiblePriorities, like
/// BacktrackingLayer's promoted_priority, which startup validation must catch.
/// (TestPromotingLayer models an undeclared one, caught only per tick.)
class TestDeclaredPromotionLayer : public TestLayerBase {
 public:
  static constexpr int kPromotedPriority = 1000;
  explicit TestDeclaredPromotionLayer(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "declared_promotion"; }
  std::vector<int> possiblePriorities() const override { return {configuredPriority(), kPromotedPriority}; }
  static PluginRegistry::Registration<TestDeclaredPromotionLayer> registration;
};
PluginRegistry::Registration<TestDeclaredPromotionLayer> TestDeclaredPromotionLayer::registration("TestDeclaredPromotionLayer");

/// @brief Layer that runs the REAL Layer::setupFromParamMap, so its required
/// generator/evaluator/path_planner child slots are composed through the strict
/// factory (unlike TestLayerBase, which composes none).
class TestStrictLayer : public TestLayerBase {
 public:
  explicit TestStrictLayer(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "strict"; }
  void setupFromParamMap(Configurable::ParamMap* param_map) override { Layer::setupFromParamMap(param_map); }
  static PluginRegistry::Registration<TestStrictLayer> registration;
};
PluginRegistry::Registration<TestStrictLayer> TestStrictLayer::registration("TestStrictLayer");

// Minimal path_planner so a TestStrictLayer can compose all of its required child
// slots; TestCostComputer fills the agent's cost_computer slot.
class TestPathPlanner : public PathPlanner {
 public:
  explicit TestPathPlanner(AgentInterface& agent) : PathPlanner(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  std::pair<std::vector<Node*>, double> planPath(Node* /*start_node*/, GraphType* /*graph*/, Node* /*required_terminal_node*/) override {
    return {};
  }
  static PluginRegistry::Registration<TestPathPlanner> registration;
};
PluginRegistry::Registration<TestPathPlanner> TestPathPlanner::registration("TestPathPlanner");

class TestCostComputer : public CostComputer {
 public:
  explicit TestCostComputer(AgentInterface& agent) : CostComputer(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  bool computeCost(Edge* /*edge*/) override { return true; }
  static PluginRegistry::Registration<TestCostComputer> registration;
};
PluginRegistry::Registration<TestCostComputer> TestCostComputer::registration("TestCostComputer");

/// @brief Layer whose runtime priority is decoupled from its configured
/// priority, to exercise the per-tick runtime-priority collision check: it
/// passes the startup distinct-configured-priority check but can report a
/// colliding runtime priority during arbitration.
class TestPromotingLayer : public TestLayerBase {
 public:
  explicit TestPromotingLayer(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "promoting"; }
  void setupFromParamMap(Configurable::ParamMap* param_map) override {
    TestLayerBase::setupFromParamMap(param_map);  // reads configured "priority"
    setParam<int>(param_map, "runtime_priority", &runtime_priority_, p_priority_);
  }
  int getPriority(const RobotState& /*state*/) const override { return runtime_priority_; }

 private:
  int runtime_priority_ = 0;
  static PluginRegistry::Registration<TestPromotingLayer> registration;
};
PluginRegistry::Registration<TestPromotingLayer> TestPromotingLayer::registration("TestPromotingLayer");

/// @brief Layer that always offers one step, so arbitration runs all the way to the
/// dispatch. It owns the node and edge it hands over; nothing else in the fixture does.
class TestWinningLayer : public TestLayerBase {
 public:
  explicit TestWinningLayer(AgentInterface& agent) : TestLayerBase(agent) {}
  std::string layerName() const override { return "winning"; }

  bool planPath(const RobotState& current_state, Map* /*map*/, RobotModel* /*robot*/, std::vector<Node*>* planned_path,
                std::vector<Edge*>* planned_edges) override {
    consultationOrder().push_back(instanceName());
    RobotState start = current_state;
    start.setTimeFromStart(0.0);  // the state reached at the end of an edge keeps that edge's time
    RobotState goal = start;
    goal.setSensorPosition(Eigen::Vector3d(1.0, 0.0, 0.0));
    goal.setTimeFromStart(1.0);
    edge_.trajectory.clear();
    edge_.trajectory.push_back(start);
    edge_.trajectory.push_back(goal);
    planned_path->clear();
    planned_path->push_back(&start_node_);
    planned_path->push_back(&goal_node_);
    planned_edges->clear();
    planned_edges->push_back(&edge_);
    return true;
  }

  std::optional<double> realAccumulatedGain() const override { return 1.25; }

 private:
  Node start_node_;
  Node goal_node_;
  Edge edge_;
  static PluginRegistry::Registration<TestWinningLayer> registration;
};
PluginRegistry::Registration<TestWinningLayer> TestWinningLayer::registration("TestWinningLayer");

// ---------------------------------------------------------------------------
// Agent under test: Agent's only pure virtual is sendMovement.
// ---------------------------------------------------------------------------
class TestAgent : public Agent {
 public:
  TestAgent(PluginFactory* factory, Configurable::ParamMap* param_map) : Agent(factory, param_map) {}

  bool sendMovement(EdgeCursor* /*cursor*/) override { return true; }

  // Expose the protected surface the tests drive.
  void setup(Configurable::ParamMap* param_map) { setupFromParamMap(param_map); }
  void arbitrate() { requestNextTrajectory(); }
  void iterate() { loopIteration(); }
  void seedState() { setCurrentState(RobotState(), StateUpdateKind::kPoseObservation); }
  void beginAwaitingCommandResult() { command_phase_ = CommandPhase::kPending; }
  void reportCommandResult() { setCurrentState(RobotState(), StateUpdateKind::kCommandResult); }
  void reportPoseObservation() { setCurrentState(RobotState(), StateUpdateKind::kPoseObservation); }
  CommandPhase commandPhase() const { return command_phase_; }
  void setCommandPhase(CommandPhase phase) { command_phase_ = phase; }

  // The per-cycle harvest of the map's integrated frames.
  void harvestObservedPoses() { recordObservedPoses(); }
  bool settled(const RobotState& target) { return hasSettled(target); }
  TestMap* testMap() { return map_->as<TestMap>(); }
  const HistoryData& history() const { return *history_data_; }
  void resetForTest() { resetEpisode(); }
  void clearForTest() { clearEpisodeState(); }
  void finalizeForTest() { finalizeEpisodeOutput(EpisodeEnd::kStopped); }  // the platform's stop
  void resumeForTest() { resumeEpisode(); }
  const std::vector<std::unique_ptr<Layer>>& layers() const { return layers_; }
  State state() const { return state_; }
  bool hasActiveDispatch() const { return active_cursor_ || active_node_ || active_layer_; }
  int mapUpdateResetCalls() const { return map_->as<TestMap>()->mapUpdateResetCalls(); }
  int mapResetEpisodeCalls() const { return map_->as<TestMap>()->resetEpisodeCalls(); }

  // Drives a completed terminating edge through the real cycle. Edge and node are
  // caller-owned so a test can inspect them (and their geometry) afterwards.
  void finishTerminatingNode(Edge* edge, Node* terminating_node) {
    terminating_node->is_terminating = true;
    setCurrentState(edge->trajectory.back(), StateUpdateKind::kPoseObservation);
    active_cursor_ = std::make_unique<EdgeCursor>(edge);
    active_cursor_->finish();
    active_node_ = terminating_node;
    command_phase_ = CommandPhase::kResolved;
    loopIteration();
  }

  /// @brief How many times the goal hook fired; must be exactly once per episode.
  int goalReachedCalls() const { return goal_reached_calls_; }
  void onGoalReached() override { ++goal_reached_calls_; }

  // Drives one loopIteration() with a finished edge owned by the CALLER, standing
  // in for a layer-owned graph edge that the layer is about to free.
  void runLoopIterationWithFinishedEdge(Edge* finished_edge, Node* node) {
    // Seed the arrived pose so the settle gate passes for any edge geometry,
    // mirroring the real arrival that precedes this point in the cycle.
    setCurrentState(finished_edge->trajectory.back(), StateUpdateKind::kPoseObservation);
    active_cursor_ = std::make_unique<EdgeCursor>(finished_edge);
    active_cursor_->finish();
    active_node_ = node;
    command_phase_ = CommandPhase::kResolved;
    loopIteration();
  }

  const Edge* activeCursorEdge() const { return active_cursor_ ? active_cursor_->edge : nullptr; }
  const Node* activeNode() const { return active_node_; }

  void publishVisualization(const VisualizationMarkers& /*markers*/) override {}
  // printInfo/printWarning/printError are final in Agent; the fixture installs a
  // no-op logging sink to keep test output quiet instead of overriding them.

 private:
  int goal_reached_calls_ = 0;
};

// ---------------------------------------------------------------------------
// Fixture: a factory pre-populated with a valid map + robot, and an agent
// param map. Tests add layer sub-dictionaries under "/planner/layers/<name>";
// Agent discovers them via the factory's listChildNamespaces.
// ---------------------------------------------------------------------------
class LayerStackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Agent::printX is final and forwards to logging::*; silence it for tests.
    logging::setSink([](logging::Severity, const std::string&) {});
    consultationOrder().clear();
    factory.setParams("/planner/map", {{"type", "TestMap"}});
    factory.setParams("/planner/robot", {{"type", "TestRobot"}});
    factory.setParams("/planner/cost_computer", {{"type", "TestCostComputer"}});
    planner_params["param_namespace"] = "/planner";
    planner_params["verbose"] = "0";
    planner_params["verbose_plugins"] = "0";
  }

  // Builds an agent and runs the real setupFromParamMap.
  std::unique_ptr<TestAgent> build() {
    auto agent = std::make_unique<TestAgent>(&factory, &planner_params);
    agent->setup(&planner_params);
    return agent;
  }

  // Register a layer as a dictionary key under the planner's "layers" namespace.
  void addLayer(const std::string& name, const std::string& type, int priority) {
    factory.setParams("/planner/layers/" + name, {{"type", type}, {"priority", std::to_string(priority)}});
  }

  /// @brief An agent posed at the origin with a command in flight -- the state the
  /// settle gate is normally consulted in.
  std::unique_ptr<TestAgent> buildAgentMidCommand() {
    auto agent = build();
    agent->seedState();
    agent->setCommandPhase(Agent::CommandPhase::kResolved);
    return agent;
  }

  PluginFactoryConfig factory;
  Configurable::ParamMap planner_params;
};

TEST_F(LayerStackTest, DebugDiagnosticsDefaultToOff) {
  addLayer("alpha", "TestLayerAlpha", 0);
  EXPECT_FALSE(build()->isDebugDiagnosticsEnabled());
}

TEST_F(LayerStackTest, DebugDiagnosticsCanBeEnabled) {
  addLayer("alpha", "TestLayerAlpha", 0);
  planner_params["debug_diagnostics"] = "1";
  EXPECT_TRUE(build()->isDebugDiagnosticsEnabled());
}

TEST_F(LayerStackTest, StrictParamsDefaultToOff) {
  addLayer("alpha", "TestLayerAlpha", 0);
  build();
  EXPECT_FALSE(factory.isStrictParams());
}

TEST_F(LayerStackTest, StrictParamsAreForwardedToTheFactory) {
  addLayer("alpha", "TestLayerAlpha", 0);
  planner_params["strict_params"] = "1";
  build();
  EXPECT_TRUE(factory.isStrictParams());
}

TEST_F(LayerStackTest, TheObservedViewpointsDefaultToOff) {
  addLayer("alpha", "TestLayerAlpha", 0);
  auto agent = build();
  EXPECT_FALSE(agent->getHistoryData().getObservedViewpoints().isEnabled());
  std::string error_message;
  EXPECT_TRUE(agent->checkParamsValid(&error_message)) << error_message;
}

TEST_F(LayerStackTest, ObservedViewpointThresholdsConfigureTheHistory) {
  addLayer("alpha", "TestLayerAlpha", 0);
  planner_params["observed_viewpoint_position_threshold"] = "0.2";
  planner_params["observed_viewpoint_angle_threshold"] = "0.3";
  auto agent = build();

  const ObservedViewpoints& viewpoints = agent->getHistoryData().getObservedViewpoints();
  EXPECT_TRUE(viewpoints.isEnabled());
  EXPECT_DOUBLE_EQ(viewpoints.getPositionThreshold(), 0.2);
  EXPECT_DOUBLE_EQ(viewpoints.getAngleThreshold(), 0.3);
  std::string error_message;
  EXPECT_TRUE(agent->checkParamsValid(&error_message)) << error_message;
}

// One threshold at zero would match on position alone or on orientation alone.
TEST_F(LayerStackTest, RejectsAnObservedViewpointPositionThresholdWithoutAnAngleThreshold) {
  addLayer("alpha", "TestLayerAlpha", 0);
  planner_params["observed_viewpoint_position_threshold"] = "0.2";
  std::string error_message;
  EXPECT_FALSE(build()->checkParamsValid(&error_message));
  EXPECT_EQ(error_message, "observed_viewpoint_angle_threshold must be > 0, or 0.0 with observed_viewpoint_position_threshold 0.0");
}

TEST_F(LayerStackTest, RejectsAnObservedViewpointAngleThresholdWithoutAPositionThreshold) {
  addLayer("alpha", "TestLayerAlpha", 0);
  planner_params["observed_viewpoint_angle_threshold"] = "0.2";
  std::string error_message;
  EXPECT_FALSE(build()->checkParamsValid(&error_message));
  EXPECT_EQ(error_message, "observed_viewpoint_position_threshold must be > 0.01, or 0.0 with observed_viewpoint_angle_threshold 0.0");
}

// A radius of at most 0.01 reads as unbounded in a radius query.
TEST_F(LayerStackTest, RejectsAnObservedViewpointPositionThresholdAtTheRadiusQueryFloor) {
  addLayer("alpha", "TestLayerAlpha", 0);
  planner_params["observed_viewpoint_position_threshold"] = "0.005";
  planner_params["observed_viewpoint_angle_threshold"] = "0.2";
  std::string error_message;
  EXPECT_FALSE(build()->checkParamsValid(&error_message));
}

TEST_F(LayerStackTest, PoseObservationsDoNotResolveAnInFlightCommand) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  agent->beginAwaitingCommandResult();

  agent->reportPoseObservation();
  EXPECT_EQ(agent->commandPhase(), Agent::CommandPhase::kPending);
  agent->reportPoseObservation();
  EXPECT_EQ(agent->commandPhase(), Agent::CommandPhase::kPending);

  agent->reportCommandResult();
  EXPECT_EQ(agent->commandPhase(), Agent::CommandPhase::kResultHeld);

  agent->reportPoseObservation();
  EXPECT_EQ(agent->commandPhase(), Agent::CommandPhase::kResolved);
}

// --- what counts as having observed a pose ----------------------------------
//
// The observation record is read as "the scene was seen from here", so it is fed
// from the map's own account of the frames it integrated -- never from the robot
// having arrived somewhere, and never from what a command was expected to produce.

TEST_F(LayerStackTest, AnEpisodeStartsWithNoObservedPoses) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  EXPECT_EQ(agent->history().getObservedViewpoints().size(), 0u);
}

TEST_F(LayerStackTest, HarvestsTheFramesTheMapReportsIntegrating) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const Transform3d first{Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()))};
  const Transform3d second{Eigen::Vector3d(4.0, 0.0, 0.0), Eigen::Quaterniond(Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()))};
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame(first), sensorToOpticalFrame(second)});

  agent->harvestObservedPoses();

  // Stored in the sensor frame the planner reasons in, not the optical frame the
  // map reports: a missing conversion would leave every pose a fixed 120 deg off.
  // The observed-viewpoint thresholds are off by default; the record keeps the poses regardless.
  const ObservedViewpoints& observed = agent->history().getObservedViewpoints();
  ASSERT_FALSE(observed.isEnabled());
  ASSERT_EQ(observed.size(), 2u);
  EXPECT_TRUE(observed.getViewpoint(0).isSamePose(first));
  EXPECT_TRUE(observed.getViewpoint(1).isSamePose(second));
}

TEST_F(LayerStackTest, HarvestingWithNoIntegratedFrameRecordsNothing) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();

  // The map reports no frame until it has integrated one.
  agent->harvestObservedPoses();

  EXPECT_EQ(agent->history().getObservedViewpoints().size(), 0u);
}

// The map integrated the image but its own readiness check still holds the gate back.
// The observation record is the map's account of images consumed, so it takes the
// frame regardless; the gate does not.
TEST_F(LayerStackTest, HarvestsFramesTheGateStillHoldsBack) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const Transform3d pose{Eigen::Vector3d(2.0, -1.0, 0.5), Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()))};
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame(pose)});
  agent->testMap()->setFramePending(true);

  EXPECT_FALSE(agent->testMap()->hasMappedFrameNear(sensorToOpticalFrame(pose), 0.1, 0.1))
      << "the gate must stay shut while the map reports the frame incomplete";

  agent->harvestObservedPoses();

  const ObservedViewpoints& observed = agent->history().getObservedViewpoints();
  ASSERT_EQ(observed.size(), 1u);
  EXPECT_TRUE(observed.getViewpoint(0).isSamePose(pose));
}

// The map keeps reporting a frame until the next dispatch clears its list, so the
// same frame is harvested again on the following cycle.
TEST_F(LayerStackTest, HarvestingTheSameFrameTwiceRecordsOneObservation) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const Transform3d pose{Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity()};
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame(pose)});

  agent->harvestObservedPoses();
  agent->harvestObservedPoses();

  EXPECT_EQ(agent->history().getObservedViewpoints().size(), 1u);
}

// A frame integrated anywhere counts, including where no command was ever
// dispatched: the mapper consumed an image there, which is the whole claim.
TEST_F(LayerStackTest, HarvestsFramesRegardlessOfDispatchState) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  agent->setCommandPhase(Agent::CommandPhase::kNone);
  const Transform3d pose{Eigen::Vector3d(7.0, 0.0, 0.0), Eigen::Quaterniond::Identity()};
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame(pose)});

  agent->harvestObservedPoses();

  ASSERT_EQ(agent->history().getObservedViewpoints().size(), 1u);
  EXPECT_TRUE(agent->history().getObservedViewpoints().getViewpoint(0).isSamePose(pose));
}

TEST_F(LayerStackTest, ResetEpisodeForgetsTheObservedPoses) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  agent->seedState();  // resetEpisode reroots from the current state, so one must exist
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame({Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity()})});
  agent->harvestObservedPoses();
  ASSERT_EQ(agent->history().getObservedViewpoints().size(), 1u);

  agent->resetForTest();

  EXPECT_EQ(agent->history().getObservedViewpoints().size(), 0u);
}

// The clear carries the episode boundary to the map's backend, so it has to run at
// the boundary -- before any pose of the new episode exists to reroot the graph.
TEST_F(LayerStackTest, ClearEpisodeStateRunsWithoutACurrentState) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();  // no seedState(): no pose is held
  ASSERT_EQ(agent->mapResetEpisodeCalls(), 0);

  EXPECT_NO_THROW(agent->clearForTest());

  EXPECT_EQ(agent->mapResetEpisodeCalls(), 1);
  EXPECT_FALSE(agent->hasActiveDispatch());
  EXPECT_EQ(agent->state(), Agent::State::kIdle);
}

// --- the map-update settle gate ---------------------------------------------
//
// With wait_for_map_updated on, arriving at the target is not enough: the map must
// answer that it holds a frame near the target before the agent replans.

TEST_F(LayerStackTest, SettleGateBlocksUntilTheMapReportsAFrame) {
  addLayer("planning", "TestLayerAlpha", 0);
  planner_params["wait_for_map_updated"] = "1";
  auto agent = buildAgentMidCommand();

  EXPECT_FALSE(agent->settled(RobotState()));
}

TEST_F(LayerStackTest, SettleGateOpensOnAFrameAtTheTarget) {
  addLayer("planning", "TestLayerAlpha", 0);
  planner_params["wait_for_map_updated"] = "1";
  auto agent = buildAgentMidCommand();
  const RobotState target;
  const Transform3d target_sensor_transform{target.getSensorPosition(), target.getSensorOrientation()};
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame(target_sensor_transform)});

  EXPECT_TRUE(agent->settled(target));
}

// The frame is at the target, but the map reports itself not done with it.
TEST_F(LayerStackTest, SettleGateStaysShutWhileTheMapHoldsTheFrameBack) {
  addLayer("planning", "TestLayerAlpha", 0);
  planner_params["wait_for_map_updated"] = "1";
  auto agent = buildAgentMidCommand();
  const RobotState target;
  const Transform3d target_sensor_transform{target.getSensorPosition(), target.getSensorOrientation()};
  agent->testMap()->setMappedOpticalFrames({sensorToOpticalFrame(target_sensor_transform)});
  agent->testMap()->setFramePending(true);

  EXPECT_FALSE(agent->settled(target));
}

// A command answered within a second is no stall: only a longer wait warns, then at most once a second.
TEST_F(LayerStackTest, SettleWarningsStartOnceAWaitHasLastedASecond) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  auto agent = build();
  agent->seedState();
  agent->resetForTest();
  agent->arbitrate();  // dispatches the step; its command result is now awaited

  int warnings = 0;
  logging::setSink([&warnings](logging::Severity severity, const std::string& message) {
    warnings += (severity == logging::Severity::kWarning && message.find("[settle]") != std::string::npos) ? 1 : 0;
  });
  agent->iterate();
  const int within_a_second = warnings;
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  agent->iterate();
  agent->iterate();
  const int after_a_second = warnings;
  logging::setSink([](logging::Severity, const std::string&) {});

  EXPECT_EQ(within_a_second, 0);
  EXPECT_EQ(after_a_second, 1);
}

// --- strict factory contract ------------------------------------------------

TEST_F(LayerStackTest, MissingTypeThrowsNamingTheNamespace) {
  factory.setParams("/planner/map", {});  // configured section, but no "type"
  try {
    build();
    FAIL() << "expected a throw when no type is configured";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("/planner/map"), std::string::npos) << msg;
  }
}

TEST_F(LayerStackTest, MissingPathPlannerSlotFailsNamingPathPlanner) {
  // A strict layer composes generator/evaluator/path_planner. With the
  // path_planner slot absent, the strict factory must fail loudly and name the
  // path_planner namespace.
  addLayer("only_layer", "TestStrictLayer", 0);
  factory.setParams("/planner/layers/only_layer/generator", {{"type", "TestGenerator"}});
  factory.setParams("/planner/layers/only_layer/evaluator", {{"type", "TestEvaluator"}});
  // Deliberately no "/planner/layers/only_layer/path_planner" section.
  try {
    build();
    FAIL() << "expected a throw when the path_planner slot is unconfigured";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("/path_planner"), std::string::npos) << msg;
  }
}

TEST_F(LayerStackTest, UnknownLayerTypeThrows) {
  addLayer("only_layer", "NoSuchLayer", 0);
  EXPECT_THROW(build(), std::runtime_error);
}

TEST_F(LayerStackTest, IncompatibleBaseTypeThrows) {
  // TestRobot is registered, but it is not a Layer.
  addLayer("only_layer", "TestRobot", 0);
  EXPECT_THROW(build(), std::runtime_error);
}

TEST_F(LayerStackTest, LayerSectionWithoutTypeThrows) {
  // A layer section must name its type; there is no default.
  factory.setParams("/planner/layers/only_layer", {{"priority", "0"}});
  EXPECT_THROW(build(), std::runtime_error);
}

TEST_F(LayerStackTest, StrictLayerComposesAllChildPlugins) {
  addLayer("planning", "TestStrictLayer", 0);
  factory.setParams("/planner/layers/planning/generator", {{"type", "TestGenerator"}});
  factory.setParams("/planner/layers/planning/evaluator", {{"type", "TestEvaluator"}});
  factory.setParams("/planner/layers/planning/path_planner", {{"type", "TestPathPlanner"}});
  auto agent = build();  // no throw: every required child slot is configured
  ASSERT_EQ(agent->layers().size(), 1u);
  EXPECT_EQ(agent->layers()[0]->instanceName(), "planning");
}

// --- cost computer: one per agent, shared by every layer --------------------

TEST_F(LayerStackTest, EveryLayerSharesTheAgentCostComputer) {
  // Edge costs are not a layer role: all layers price with the agent's one
  // cost computer, which is what keeps their cost-budget debits in one unit.
  addLayer("alpha", "TestLayerAlpha", 0);
  addLayer("beta", "TestLayerBeta", 1);
  auto agent = build();
  ASSERT_EQ(agent->layers().size(), 2u);
  ASSERT_NE(agent->getCostComputer(), nullptr);
  for (const auto& layer : agent->layers()) {
    EXPECT_TRUE(layer->hasCostComputer());
    EXPECT_EQ(layer->getCostComputer(), agent->getCostComputer()) << layer->instanceName();
  }
}

TEST_F(LayerStackTest, MissingAgentCostComputerFailsNamingTheSlot) {
  // Required like map and robot: unconfigured is a startup error, not a layer
  // that silently prices nothing.
  addLayer("alpha", "TestLayerAlpha", 0);
  factory.setParams("/planner/cost_computer", {});
  try {
    build();
    FAIL() << "expected a throw when the agent's cost_computer slot is unconfigured";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("/planner/cost_computer"), std::string::npos) << msg;
  }
}

// --- layers dictionary validation -------------------------------------------

TEST_F(LayerStackTest, DictionaryKeyBecomesInstanceName) {
  addLayer("my_special_layer", "TestLayerAlpha", 0);
  auto agent = build();
  ASSERT_EQ(agent->layers().size(), 1u);
  EXPECT_EQ(agent->layers()[0]->instanceName(), "my_special_layer");
}

TEST_F(LayerStackTest, AllConfiguredLayersAreConstructed) {
  addLayer("planning", "TestLayerAlpha", 0);
  addLayer("termination", "TestLayerBeta", 2000);
  addLayer("backtracking", "TestLayerBeta", -1000);
  auto agent = build();
  ASSERT_EQ(agent->layers().size(), 3u);
  std::set<std::string> names;
  for (const auto& layer : agent->layers()) {
    names.insert(layer->instanceName());
  }
  EXPECT_EQ(names, (std::set<std::string>{"backtracking", "planning", "termination"}));
}

TEST_F(LayerStackTest, MissingLayersDictionaryThrows) {
  // No "/planner/layers/*" configured at all: nothing to arbitrate.
  EXPECT_THROW(build(), std::runtime_error);
}

TEST_F(LayerStackTest, TwoInstancesOfOneLayerTypeStayDistinct) {
  addLayer("first_layer", "TestLayerAlpha", 0);
  addLayer("second_layer", "TestLayerAlpha", 5);
  auto agent = build();
  ASSERT_EQ(agent->layers().size(), 2u);
  std::set<std::string> names;
  for (const auto& layer : agent->layers()) {
    names.insert(layer->instanceName());
    // Same implementation kind, distinct runtime identity.
    EXPECT_EQ(layer->layerName(), agent->layers()[0]->layerName());
  }
  EXPECT_EQ(names, (std::set<std::string>{"first_layer", "second_layer"}));
}

// --- priority distinctness --------------------------------------------------

TEST_F(LayerStackTest, DuplicateConfiguredPriorityThrows) {
  addLayer("planning", "TestLayerAlpha", 3);
  addLayer("other", "TestLayerBeta", 3);  // same configured priority
  EXPECT_THROW(build(), std::runtime_error);
}

// --- active-pointer lifetime ------------------------------------------------

// The cursor does not own its edge and the layer callbacks may destroy it, so the
// agent must drop the non-owning pointers as soon as the completed edge has been
// recorded — not carry them until requestNextTrajectory reassigns them.
TEST_F(LayerStackTest, ActivePointersAreClearedBeforeLayerCallbacksRun) {
  addLayer("edge_freeing", "TestEdgeFreeingLayer", 0);
  auto agent = build();

  TestEdgeFreeingLayer::sampledCursorEdges().clear();
  TestEdgeFreeingLayer::ownedEdge() = std::make_unique<Edge>();
  Edge* edge = TestEdgeFreeingLayer::ownedEdge().get();
  RobotState state;
  edge->trajectory.push_back(state);
  edge->trajectory.push_back(state);
  Node node;
  cursorProbe() = [&agent]() { return agent->activeCursorEdge(); };

  agent->runLoopIterationWithFinishedEdge(edge, &node);
  cursorProbe() = nullptr;

  // Callbacks ran (onStateReached, update, planPath) and none of them saw a
  // cursor: no pointer to the freed edge was ever observable.
  ASSERT_GE(TestEdgeFreeingLayer::sampledCursorEdges().size(), 3u);
  for (const Edge* sampled : TestEdgeFreeingLayer::sampledCursorEdges()) {
    EXPECT_EQ(sampled, nullptr);
  }
  EXPECT_EQ(agent->activeNode(), nullptr);
}

// The edge's contents must still reach history/metrics, from the snapshot taken
// before the pointers are dropped.
TEST_F(LayerStackTest, CompletedEdgeIsStillRecordedAfterTheEarlyClear) {
  addLayer("edge_freeing", "TestEdgeFreeingLayer", 0);
  auto agent = build();

  TestEdgeFreeingLayer::sampledCursorEdges().clear();
  TestEdgeFreeingLayer::ownedEdge() = std::make_unique<Edge>();
  Edge* edge = TestEdgeFreeingLayer::ownedEdge().get();
  RobotState start;
  RobotState end;
  end.setSensorPosition(Eigen::Vector3d(2.0, 0.0, 0.0));
  edge->trajectory.push_back(start);
  edge->trajectory.push_back(end);
  Node node;

  agent->runLoopIterationWithFinishedEdge(edge, &node);

  EXPECT_EQ(agent->getMetrics().num_visited_nodes, 1);
  EXPECT_NEAR(agent->getMetrics().total_path_length, 2.0, 1e-9);
}

// The edge that exhausts the budget was executed like any other, so it must be
// in the totals before the episode ends -- the same guarantee the terminating
// edge gets, on the other termination path.
TEST_F(LayerStackTest, BudgetExhaustingEdgeIsStillFullyAccounted) {
  addLayer("planning", "TestLayerAlpha", 0);
  planner_params["cost_budget"] = "1.0";
  auto agent = build();
  agent->seedState();
  agent->resetForTest();  // installs current_cost_budget_ and zeroes the metrics

  Edge edge;
  edge.cost = 2.0;  // on its own more than the whole budget
  RobotState start;
  RobotState goal;
  goal.setSensorPosition(Eigen::Vector3d(1.0, 0.0, 0.0));
  goal.setTimeFromStart(2.0);
  edge.trajectory.push_back(start);
  edge.trajectory.push_back(goal);
  Node node;

  agent->runLoopIterationWithFinishedEdge(&edge, &node);

  EXPECT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_FALSE(agent->hasActiveDispatch());
  EXPECT_EQ(agent->getMetrics().num_visited_nodes, 1);
  EXPECT_NEAR(agent->getMetrics().total_path_length, 1.0, 1e-9);
  EXPECT_NEAR(agent->getMetrics().total_path_duration, 2.0, 1e-9);
  EXPECT_NEAR(agent->getMetrics().total_cost, 2.0, 1e-9);
}

TEST_F(LayerStackTest, DeclaredPromotionCollidingWithAnotherLayerThrowsAtStartup) {
  // Configured priorities are distinct (0 vs 1000), but the declared promotion
  // value collides with the other layer's configured 1000. Without the
  // possible-priority check this would pass startup and crash mid-episode the
  // first time the promotion fired.
  addLayer("promoting", "TestDeclaredPromotionLayer", 0);
  addLayer("other", "TestLayerAlpha", TestDeclaredPromotionLayer::kPromotedPriority);
  try {
    build();
    FAIL() << "expected the possible-priority overlap to throw at startup";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("promoting"), std::string::npos) << msg;
    EXPECT_NE(msg.find("other"), std::string::npos) << msg;
    EXPECT_NE(msg.find("1000"), std::string::npos) << msg;
  }
}

TEST_F(LayerStackTest, DeclaredPromotionWithoutOverlapStartsUp) {
  // The same declared promotion is fine when nothing else can report 1000.
  addLayer("promoting", "TestDeclaredPromotionLayer", 0);
  addLayer("other", "TestLayerAlpha", 7);
  EXPECT_NO_THROW(build());
}

TEST_F(LayerStackTest, RuntimePriorityCollisionThrowsDuringArbitration) {
  // Configured priorities are distinct (0 vs 5), so startup succeeds; but the
  // promoting layer reports runtime priority 0 at tick time, colliding with the
  // other layer, which must throw during arbitration.
  addLayer("planning", "TestLayerAlpha", 0);
  factory.setParams("/planner/layers/promoting", {{"type", "TestPromotingLayer"}, {"priority", "5"}, {"runtime_priority", "0"}});
  auto agent = build();  // startup OK
  agent->seedState();
  EXPECT_THROW(agent->arbitrate(), std::runtime_error);
}

// --- arbitration ------------------------------------------------------------

TEST_F(LayerStackTest, ArbitrationConsultsHigherPriorityFirst) {
  addLayer("planning", "TestLayerAlpha", 0);
  addLayer("termination", "TestLayerBeta", 2000);
  addLayer("backtracking", "TestLayerBeta", -1000);
  auto agent = build();
  agent->seedState();
  agent->arbitrate();
  EXPECT_EQ(consultationOrder(), (std::vector<std::string>{"termination", "planning", "backtracking"}));
}

TEST_F(LayerStackTest, ArbitrationIgnoresDictionaryOrder) {
  // Dictionary keys are enumerated alphabetically ("aaa" before "zzz"), but
  // arbitration must follow descending priority regardless: "zzz" (higher
  // priority) is consulted before "aaa".
  addLayer("aaa", "TestLayerAlpha", -1);
  addLayer("zzz", "TestLayerBeta", 10);
  auto agent = build();
  agent->seedState();
  agent->arbitrate();
  EXPECT_EQ(consultationOrder(), (std::vector<std::string>{"zzz", "aaa"}));
}

TEST_F(LayerStackTest, TerminalArbitrationResetsMapCycleStateExactlyOnce) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  agent->seedState();

  ASSERT_EQ(agent->mapUpdateResetCalls(), 0);
  agent->arbitrate();  // the test layer declines, so this is the terminal cycle

  EXPECT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(agent->mapUpdateResetCalls(), 1);
}

// --- frame ledger -----------------------------------------------------------

TEST_F(LayerStackTest, DispatchedCommandsAreCountedPerCycle) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  auto agent = build();
  agent->seedState();

  agent->arbitrate();
  EXPECT_EQ(agent->getMetrics().commands_dispatched, 1);

  // Per cycle, not cumulative: resetCycle runs before this cycle's own dispatch.
  agent->arbitrate();
  EXPECT_EQ(agent->getMetrics().commands_dispatched, 1);
}

TEST_F(LayerStackTest, TheFrameLedgerCountsWhatTheMapIntegrated) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  agent->seedState();
  const Transform3d first{Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Quaterniond::Identity()};
  const Transform3d second{Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Quaterniond::Identity()};
  agent->testMap()->setMappedOpticalFrames({first, second});

  // The layer declines, so the cycle ends before resetCycle clears the count.
  agent->arbitrate();

  EXPECT_EQ(agent->getMetrics().frames_mapped, 2);
  EXPECT_EQ(agent->getMetrics().commands_dispatched, 0);
}

TEST_F(LayerStackTest, TheFrameLedgerReportsAMapWithoutAFrameListOnlyUnderDebugDiagnostics) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  // Two cycles: the second reads the first's dispatched command against a map that lists no frames.
  const auto ledger_errors = [this]() {
    auto agent = build();
    agent->seedState();
    int count = 0;
    logging::setSink([&count](logging::Severity severity, const std::string& message) {
      const bool ledger = message.find("[frame ledger]") != std::string::npos;
      count += (ledger && (severity == logging::Severity::kError || severity == logging::Severity::kWarning)) ? 1 : 0;
    });
    agent->arbitrate();
    agent->arbitrate();
    logging::setSink([](logging::Severity, const std::string&) {});
    return count;
  };

  EXPECT_EQ(ledger_errors(), 0);
  planner_params["debug_diagnostics"] = "1";
  EXPECT_EQ(ledger_errors(), 1);
}

TEST_F(LayerStackTest, TerminatingNodeSynchronizesStateStatusAndRestart) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  consultationOrder().clear();

  // The final approach was physically executed, so it must be accounted for
  // exactly once before the episode ends: history, path length, duration, cost.
  Edge edge;
  edge.cost = 3.5;
  RobotState start;
  RobotState goal;
  goal.setSensorPosition(Eigen::Vector3d(1.5, 0.0, 0.0));
  goal.setTimeFromStart(4.0);
  edge.trajectory.push_back(start);
  edge.trajectory.push_back(goal);
  Node terminating_node;

  agent->finishTerminatingNode(&edge, &terminating_node);

  EXPECT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(agent->getExecutionStatus(), Agent::ExecutionStatus::kTerminated);
  EXPECT_FALSE(agent->hasActiveDispatch());

  EXPECT_EQ(agent->goalReachedCalls(), 1);  // the stop hook fires exactly once
  EXPECT_EQ(agent->getMetrics().num_visited_nodes, 1);
  EXPECT_NEAR(agent->getMetrics().total_path_length, 1.5, 1e-9);
  EXPECT_NEAR(agent->getMetrics().total_path_duration, 4.0, 1e-9);
  EXPECT_NEAR(agent->getMetrics().total_cost, 3.5, 1e-9);
  EXPECT_GT(agent->getHistoryData().size(), 0);
  // Terminating means terminating: no layer is consulted for a next step.
  EXPECT_TRUE(consultationOrder().empty());

  agent->seedState();
  agent->resetForTest();
  EXPECT_EQ(agent->state(), Agent::State::kPlanning);
  EXPECT_EQ(agent->getExecutionStatus(), Agent::ExecutionStatus::kAwaitingCommandResult);
  EXPECT_FALSE(agent->hasActiveDispatch());

  // The flag must not leak into the next episode: finishing a NON-terminating
  // edge may not re-run the goal path. (These fake layers never offer a step, so
  // the cycle still ends via "no layer offered a step" — hence the assertion is
  // on the hook, not on state_.)
  Edge next_edge;
  RobotState next_state;
  next_edge.trajectory.push_back(next_state);
  next_edge.trajectory.push_back(next_state);
  Node plain_node;
  agent->runLoopIterationWithFinishedEdge(&next_edge, &plain_node);
  EXPECT_EQ(agent->goalReachedCalls(), 1);
}

// The driver backs off exactly when execution waits on an external input; a
// wrong classification either spins a core (false) or delays planning (true).
TEST(AgentExecutionStatus, AwaitingStatusesAreExternalInputWaits) {
  EXPECT_TRUE(Agent::isAwaitingExternalInput(Agent::ExecutionStatus::kAwaitingCommandResult));
  EXPECT_TRUE(Agent::isAwaitingExternalInput(Agent::ExecutionStatus::kAwaitingPose));
  EXPECT_TRUE(Agent::isAwaitingExternalInput(Agent::ExecutionStatus::kAwaitingMapUpdate));
  EXPECT_FALSE(Agent::isAwaitingExternalInput(Agent::ExecutionStatus::kPlanned));
  EXPECT_FALSE(Agent::isAwaitingExternalInput(Agent::ExecutionStatus::kTerminated));
}

// --- configureRunOutput lifecycle ------------------------------------------
// With logging off (fixture default), the only artifact is the manifest, so
// these exercise the directory contract without needing an open CSV.

namespace {
std::string makeRunDir(const std::string& suffix) {
  std::string dir = ::testing::TempDir() + "/open_apero_run_" + suffix;
  ::mkdir(dir.c_str(), 0775);
  return dir;
}
bool fileExists(const std::string& path) {
  return ::access(path.c_str(), F_OK) == 0;
}
}  // namespace

TEST_F(LayerStackTest, ConfigureRunOutputWritesManifestAndSucceeds) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const std::string dir = makeRunDir("ok");
  ::unlink((dir + "/run_manifest.yaml").c_str());

  EXPECT_TRUE(agent->configureRunOutput({dir}));
  EXPECT_TRUE(fileExists(dir + "/run_manifest.yaml"));
}

TEST_F(LayerStackTest, ConfigureRunOutputEmptyDirFailsAndIsRetryable) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  EXPECT_FALSE(agent->configureRunOutput({""}));
  // A failed attempt must not latch: a valid directory afterwards still works.
  const std::string dir = makeRunDir("retry");
  ::unlink((dir + "/run_manifest.yaml").c_str());
  EXPECT_TRUE(agent->configureRunOutput({dir}));
}

TEST_F(LayerStackTest, ConfigureRunOutputReportsManifestFilesystemCause) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const std::string missing_dir = makeRunDir("manifest_error") + "/missing_" + std::to_string(::getpid());

  std::vector<std::string> errors;
  logging::setSink([&errors](logging::Severity severity, const std::string& message) {
    if (severity == logging::Severity::kError) errors.push_back(message);
  });
  const bool configured = agent->configureRunOutput({missing_dir});
  logging::setSink([](logging::Severity, const std::string&) {});

  EXPECT_FALSE(configured);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("mkstemp failed"), std::string::npos);
  EXPECT_NE(errors.front().find("errno " + std::to_string(ENOENT)), std::string::npos);
}

TEST_F(LayerStackTest, ConfigureRunOutputIsIdempotentForSameDir) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const std::string dir = makeRunDir("idem");
  ::unlink((dir + "/run_manifest.yaml").c_str());
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  EXPECT_TRUE(agent->configureRunOutput({dir}));  // same dir: no-op success
}

TEST_F(LayerStackTest, ConfigureRunOutputRejectsDifferentDirAfterSuccess) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const std::string dir1 = makeRunDir("first");
  const std::string dir2 = makeRunDir("second");
  ::unlink((dir1 + "/run_manifest.yaml").c_str());
  ::unlink((dir2 + "/run_manifest.yaml").c_str());
  ASSERT_TRUE(agent->configureRunOutput({dir1}));
  EXPECT_FALSE(agent->configureRunOutput({dir2}));
}

TEST_F(LayerStackTest, ConfigureRunOutputRecordsTheRandomSeed) {
  addLayer("planning", "TestLayerAlpha", 0);
  planner_params["random_seed"] = "7";
  auto agent = build();
  const std::string dir = makeRunDir("seed");
  ::unlink((dir + "/run_manifest.yaml").c_str());
  ASSERT_TRUE(agent->configureRunOutput({dir}));

  std::ifstream in(dir + "/run_manifest.yaml");
  std::stringstream yaml;
  yaml << in.rdbuf();
  EXPECT_NE(yaml.str().find("\nrandom_seed: 7\n"), std::string::npos) << yaml.str();
}

TEST_F(LayerStackTest, ConfigureRunOutputRecordsTheNondeterministicSeed) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const std::string dir = makeRunDir("seed_default");
  ::unlink((dir + "/run_manifest.yaml").c_str());
  ASSERT_TRUE(agent->configureRunOutput({dir}));

  std::ifstream in(dir + "/run_manifest.yaml");
  std::stringstream yaml;
  yaml << in.rdbuf();
  EXPECT_NE(yaml.str().find("\nrandom_seed: -1\n"), std::string::npos) << yaml.str();
}

TEST_F(LayerStackTest, ConfigureRunOutputRejectsPreexistingTarget) {
  addLayer("planning", "TestLayerAlpha", 0);
  auto agent = build();
  const std::string dir = makeRunDir("preexist");
  const std::string manifest = dir + "/run_manifest.yaml";
  { std::ofstream(manifest) << "stale"; }  // a pre-existing artifact must not be clobbered
  EXPECT_FALSE(agent->configureRunOutput({dir}));
  // The pre-existing file is left untouched.
  std::ifstream in(manifest);
  std::string contents;
  std::getline(in, contents);
  EXPECT_EQ(contents, "stale");
}

// --- episode end: episode summary and map snapshots ------------------------

namespace {

/// @brief A fresh run directory: artifacts of an earlier run of the test binary are removed.
std::string makeCleanRunDir(const std::string& suffix) {
  const std::string dir = makeRunDir(suffix);
  for (const char* name : {"/run_manifest.yaml", "/performance_log.csv", "/map_log.csv", "/episode_summary.yaml"}) {
    ::unlink((dir + name).c_str());
  }
  ::rmdir((dir + "/maps").c_str());
  return dir;
}

std::vector<std::vector<std::string>> readCsv(const std::string& path) {
  std::vector<std::vector<std::string>> rows;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    std::vector<std::string> cells;
    std::stringstream cells_stream(line);
    std::string cell;
    while (std::getline(cells_stream, cell, ',')) {
      cells.push_back(cell);
    }
    rows.push_back(cells);
  }
  return rows;
}

/// @brief Value of column @p name in data row @p row_index (0 = first row after the header).
double csvValue(const std::vector<std::vector<std::string>>& rows, size_t row_index, const std::string& name) {
  const std::vector<std::string>& header = rows.at(0);
  for (size_t column = 0; column < header.size(); ++column) {
    if (header[column] == name) {
      return std::stod(rows.at(row_index + 1).at(column));
    }
  }
  throw std::runtime_error("no column '" + name + "'");
}

using YamlLines = std::vector<std::pair<std::string, std::string>>;

/// @brief The `key: value` lines of a flat YAML file, in order. A key may be double-quoted; an
/// indented line's key is prefixed with its parent's, e.g. "real_accumulated_gain.planning".
YamlLines readYamlLines(const std::string& path) {
  YamlLines lines;
  std::ifstream in(path);
  std::string line;
  std::string parent;
  while (std::getline(in, line)) {
    const size_t start = line.find_first_not_of(' ');
    if (start == std::string::npos) {
      continue;
    }
    const std::string text = line.substr(start);
    const size_t key_end = text[0] == '"' ? text.find('"', 1) + 1 : text.find(':');
    std::string key = text[0] == '"' ? text.substr(1, key_end - 2) : text.substr(0, key_end);
    const std::string value = key_end + 2 <= text.size() ? text.substr(key_end + 2) : "";
    if (start == 0) {
      parent = key;
    } else {
      key = parent + "." + key;
    }
    lines.emplace_back(key, value);
  }
  return lines;
}

std::string yamlValue(const YamlLines& lines, const std::string& key) {
  for (const auto& [line_key, value] : lines) {
    if (line_key == key) {
      return value;
    }
  }
  throw std::runtime_error("no key '" + key + "'");
}

std::string readFileContents(const std::string& path) {
  std::ifstream in(path);
  std::stringstream contents;
  contents << in.rdbuf();
  return contents.str();
}

/// @brief A two-state edge of the given cost from x = @p start_x to 1 m further along x.
Edge makeCostedEdge(double cost, double start_x = 0.0) {
  Edge edge;
  edge.cost = cost;
  RobotState start;
  start.setSensorPosition(Eigen::Vector3d(start_x, 0.0, 0.0));
  RobotState goal;
  goal.setSensorPosition(Eigen::Vector3d(start_x + 1.0, 0.0, 0.0));
  goal.setTimeFromStart(1.0);
  edge.trajectory.push_back(start);
  edge.trajectory.push_back(goal);
  return edge;
}

TEST_F(LayerStackTest, StoppingWritesTheEpisodeSummaryAndNoExtraRow) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["log_performance"] = "1";
  planner_params["cost_budget"] = "5.0";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_stop");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();  // row 0
  Node node;
  Edge first = makeCostedEdge(2.0);
  agent->runLoopIterationWithFinishedEdge(&first, &node);  // row 1
  agent->finalizeForTest();

  EXPECT_EQ(readCsv(dir + "/performance_log.csv").size(), 3u);  // header + one row per cycle
  const YamlLines summary = readYamlLines(dir + "/episode_summary.yaml");
  std::vector<std::string> keys;
  for (const auto& entry : summary) {
    keys.push_back(entry.first);
  }
  EXPECT_EQ(keys, (std::vector<std::string>{"format_version", "end_reason", "cycles", "total_cost", "cost_budget", "total_path_length",
                                            "total_path_duration", "num_visited_nodes", "elapsed_wall_time_s", "real_accumulated_gain",
                                            "real_accumulated_gain.planning"}));
  EXPECT_EQ(yamlValue(summary, "format_version"), "1");
  EXPECT_EQ(yamlValue(summary, "end_reason"), "stopped");
  EXPECT_EQ(yamlValue(summary, "cycles"), "2");
  EXPECT_DOUBLE_EQ(std::stod(yamlValue(summary, "total_cost")), agent->getUsedCostBudget());
  EXPECT_EQ(yamlValue(summary, "cost_budget"), "5");
  EXPECT_EQ(yamlValue(summary, "total_path_length"), "1");
  EXPECT_EQ(yamlValue(summary, "total_path_duration"), "1");
  EXPECT_EQ(yamlValue(summary, "num_visited_nodes"), "1");
  EXPECT_GE(std::stod(yamlValue(summary, "elapsed_wall_time_s")), 0.0);
  EXPECT_EQ(yamlValue(summary, "real_accumulated_gain.planning"), "1.25");
}

TEST_F(LayerStackTest, AResumedEpisodeRewritesTheSummaryAtItsRealEnd) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["log_performance"] = "1";
  planner_params["cost_budget"] = "5.0";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_resume");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();  // row 0
  Node node;
  Edge first = makeCostedEdge(2.0);
  agent->runLoopIterationWithFinishedEdge(&first, &node);  // row 1
  agent->finalizeForTest();
  ASSERT_EQ(yamlValue(readYamlLines(dir + "/episode_summary.yaml"), "end_reason"), "stopped");

  agent->resumeForTest();
  Edge second = makeCostedEdge(4.0, 1.0);
  agent->runLoopIterationWithFinishedEdge(&second, &node);  // exhausts the budget: no row

  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(readCsv(dir + "/performance_log.csv").size(), 3u);
  const YamlLines summary = readYamlLines(dir + "/episode_summary.yaml");
  EXPECT_EQ(yamlValue(summary, "end_reason"), "budget_exhausted");
  EXPECT_EQ(yamlValue(summary, "cycles"), "2");
  EXPECT_EQ(yamlValue(summary, "total_cost"), "6");
  EXPECT_DOUBLE_EQ(agent->getUsedCostBudget(), 6.0);
  EXPECT_EQ(yamlValue(summary, "num_visited_nodes"), "2");
}

TEST_F(LayerStackTest, ASecondFinalizeWithoutResumeRewritesNothing) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["cost_budget"] = "5.0";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_once");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();
  agent->finalizeForTest();
  const std::string stopped = readFileContents(dir + "/episode_summary.yaml");
  ASSERT_NE(stopped.find("\nend_reason: stopped\n"), std::string::npos) << stopped;

  agent->finalizeForTest();
  EXPECT_EQ(readFileContents(dir + "/episode_summary.yaml"), stopped);
  Node node;
  Edge exhausting = makeCostedEdge(6.0);
  agent->runLoopIterationWithFinishedEdge(&exhausting, &node);  // terminates, but no resume re-armed it
  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(readFileContents(dir + "/episode_summary.yaml"), stopped);
}

// RosAgent's shutdown stops a started episode; one that already ended keeps its end reason.
TEST_F(LayerStackTest, AStopAfterTheEpisodeEndedKeepsItsEndReason) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["cost_budget"] = "5.0";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_end_then_stop");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();
  Node node;
  Edge exhausting = makeCostedEdge(6.0);
  agent->runLoopIterationWithFinishedEdge(&exhausting, &node);
  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  agent->finalizeForTest();

  EXPECT_EQ(yamlValue(readYamlLines(dir + "/episode_summary.yaml"), "end_reason"), "budget_exhausted");
}

TEST_F(LayerStackTest, UnreadAgentParametersAreReportedAndStrictModeRejectsThem) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["bogus_key"] = "1";
  auto lenient = build();
  EXPECT_NO_THROW(lenient->reportUnreadParams(planner_params));  // warns only

  planner_params["strict_params"] = "1";
  auto strict = build();
  try {
    strict->reportUnreadParams(planner_params);
    FAIL() << "strict_params must reject an unread key";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_EQ(message.rfind("Agent ignores parameters: bogus_key", 0), 0u) << message;
    EXPECT_EQ(message.find(", "), std::string::npos) << "only the unread key is listed: " << message;
  }
}

}  // namespace

TEST_F(LayerStackTest, BudgetExhaustionWritesTheEpisodeSummary) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["log_performance"] = "1";
  planner_params["cost_budget"] = "5.0";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_budget");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();  // row 0
  Node node;
  Edge first = makeCostedEdge(2.0);
  agent->runLoopIterationWithFinishedEdge(&first, &node);  // row 1
  Edge second = makeCostedEdge(4.0, 1.0);
  agent->runLoopIterationWithFinishedEdge(&second, &node);  // exhausts the budget: no row

  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  const auto rows = readCsv(dir + "/performance_log.csv");
  ASSERT_EQ(rows.size(), 3u);  // header + one row per cycle
  EXPECT_DOUBLE_EQ(csvValue(rows, 1, "TotalCost"), 2.0);
  EXPECT_DOUBLE_EQ(csvValue(rows, 1, "CycleIndex"), 1.0);
  const YamlLines summary = readYamlLines(dir + "/episode_summary.yaml");
  EXPECT_EQ(yamlValue(summary, "end_reason"), "budget_exhausted");
  EXPECT_EQ(yamlValue(summary, "cycles"), "2");
  EXPECT_DOUBLE_EQ(std::stod(yamlValue(summary, "total_cost")), agent->getUsedCostBudget());
  EXPECT_EQ(yamlValue(summary, "num_visited_nodes"), "2");
}

TEST_F(LayerStackTest, ReachingTheGoalWritesTheEpisodeSummary) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["log_performance"] = "1";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_goal");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();  // row 0
  Edge edge = makeCostedEdge(3.5);
  Node terminating_node;
  agent->finishTerminatingNode(&edge, &terminating_node);

  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(readCsv(dir + "/performance_log.csv").size(), 2u);
  const YamlLines summary = readYamlLines(dir + "/episode_summary.yaml");
  EXPECT_EQ(yamlValue(summary, "end_reason"), "goal_reached");
  EXPECT_EQ(yamlValue(summary, "cycles"), "1");
  EXPECT_EQ(yamlValue(summary, "total_cost"), "3.5");
  EXPECT_EQ(yamlValue(summary, "cost_budget"), ".inf");
}

TEST_F(LayerStackTest, NoLayerOfferingAStepWritesTheEpisodeSummary) {
  addLayer("planning", "TestLayerAlpha", 0);
  planner_params["log_performance"] = "1";
  auto agent = build();
  const std::string dir = makeCleanRunDir("summary_declined");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();  // the only layer declines

  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(readCsv(dir + "/performance_log.csv").size(), 1u);  // the header only
  const YamlLines summary = readYamlLines(dir + "/episode_summary.yaml");
  EXPECT_EQ(yamlValue(summary, "end_reason"), "no_step");
  EXPECT_EQ(yamlValue(summary, "cycles"), "0");
  EXPECT_EQ(yamlValue(summary, "real_accumulated_gain"), "{}");  // no layer reports a gain
}

// With an unbounded budget the snapshots have no cost spacing: one is saved, at the end.
TEST_F(LayerStackTest, UnboundedBudgetSavesTheFinalMapAtTermination) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["num_maps_to_save"] = "3";
  auto agent = build();
  ASSERT_FALSE(std::isfinite(agent->getOriginalCostBudget()));
  const std::string dir = makeCleanRunDir("maps_unbounded");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();
  Node node;
  Edge first = makeCostedEdge(2.0);
  agent->runLoopIterationWithFinishedEdge(&first, &node);
  EXPECT_TRUE(agent->testMap()->savedMaps().empty());

  Edge last = makeCostedEdge(1.0, 1.0);
  Node terminating_node;
  agent->finishTerminatingNode(&last, &terminating_node);

  ASSERT_EQ(agent->testMap()->savedMaps().size(), 1u);
  EXPECT_EQ(agent->testMap()->savedMaps().front(), dir + "/maps/00000");
  const auto map_log = readCsv(dir + "/map_log.csv");
  ASSERT_EQ(map_log.size(), 2u);
  EXPECT_EQ(map_log[1].at(0), "00000");
  EXPECT_DOUBLE_EQ(std::stod(map_log[1].at(1)), 2.0);  // TotalPathLength: two 1 m edges
}

TEST_F(LayerStackTest, AStopAndTheRealEndAfterAResumeEachSaveTheFinalMap) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["num_maps_to_save"] = "3";
  auto agent = build();
  const std::string dir = makeCleanRunDir("maps_resume");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();
  agent->finalizeForTest();
  agent->finalizeForTest();
  EXPECT_EQ(agent->testMap()->savedMaps(), (std::vector<std::string>{dir + "/maps/00000"}));

  agent->resumeForTest();
  Edge last = makeCostedEdge(1.0);
  Node terminating_node;
  agent->finishTerminatingNode(&last, &terminating_node);
  EXPECT_EQ(agent->testMap()->savedMaps(), (std::vector<std::string>{dir + "/maps/00000", dir + "/maps/00001"}));
  EXPECT_EQ(yamlValue(readYamlLines(dir + "/episode_summary.yaml"), "end_reason"), "goal_reached");
}

TEST_F(LayerStackTest, BoundedBudgetSpacesMapSnapshotsByUsedBudget) {
  addLayer("planning", "TestWinningLayer", 0);
  planner_params["visualize"] = "0";
  planner_params["num_maps_to_save"] = "2";
  planner_params["cost_budget"] = "10.0";
  auto agent = build();
  const std::string dir = makeCleanRunDir("maps_bounded");
  ASSERT_TRUE(agent->configureRunOutput({dir}));
  agent->seedState();
  agent->resetForTest();

  agent->arbitrate();  // nothing spent yet
  EXPECT_TRUE(agent->testMap()->savedMaps().empty());

  Node node;
  Edge edge_a = makeCostedEdge(1.0);
  agent->runLoopIterationWithFinishedEdge(&edge_a, &node);  // used 1 > 0: snapshot 0
  Edge edge_b = makeCostedEdge(1.0, 1.0);
  agent->runLoopIterationWithFinishedEdge(&edge_b, &node);  // used 2 <= 5
  Edge edge_c = makeCostedEdge(4.0, 2.0);
  agent->runLoopIterationWithFinishedEdge(&edge_c, &node);  // used 6 > 5: snapshot 1
  Edge edge_d = makeCostedEdge(5.0, 3.0);
  agent->runLoopIterationWithFinishedEdge(&edge_d, &node);  // exhausts the budget

  ASSERT_EQ(agent->state(), Agent::State::kTerminated);
  EXPECT_EQ(agent->testMap()->savedMaps(), (std::vector<std::string>{dir + "/maps/00000", dir + "/maps/00001"}));
}

}  // namespace
}  // namespace open_apero
