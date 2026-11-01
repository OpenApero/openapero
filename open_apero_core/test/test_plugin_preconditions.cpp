/**
 * @file test_plugin_preconditions.cpp
 * @brief Setup refuses a map or generator of the wrong type and parameters out of range, naming the plugin
 *        and its namespace; edge splitting refuses indices it cannot split at.
 */

#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/backtracker/backtracker.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/generator/generator.h"
#include "open_apero_core/plugin/generator/rrag.h"
#include "open_apero_core/plugin/generator/rrt.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/sensor_model/sensor_model.h"
#include "open_apero_core/state/robot_state.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// The message a createPlugin() call throws, or "" when it does not throw a std::runtime_error.
std::string evaluatorSetupError(ConfigAgent* agent, const std::string& type) {
  agent->factory().setParams("/eval", {{"type", type}});
  agent->factory().setParams("/eval/sensor_model", {{"type", "IterativeRayCaster"}});
  try {
    agent->getFactory().createPlugin<Evaluator>("/eval", *agent, false);
  } catch (const std::runtime_error& error) {
    return error.what();
  }
  return "";
}

TEST(EvaluatorMapRequirement, SurfaceFrontierEvaluatorRefusesAMapWithoutOccupancy) {
  ConfigAgent agent;
  PlainMap map(agent);
  agent.provide(&map, nullptr);
  EXPECT_NE(
      evaluatorSetupError(&agent, "SurfaceFrontierEvaluator").find("'SurfaceFrontierEvaluator' requires a map of type 'OccupancyMap'!"),
      std::string::npos);
}

TEST(EvaluatorMapRequirement, VoxelWeightEvaluatorRefusesAMapWithoutWeights) {
  ConfigAgent agent;
  FreeOccupancyMap map(agent);
  agent.provide(&map, nullptr);
  EXPECT_NE(evaluatorSetupError(&agent, "VoxelWeightEvaluator").find("'VoxelWeightEvaluator' requires a map of type 'TsdfMap'!"),
            std::string::npos);
}

TEST(EvaluatorMapRequirement, SimulatedSensorEvaluatorsAndSensorModelsRefuseAMapWithoutOccupancy) {
  ConfigAgent agent;
  PlainMap map(agent);
  agent.provide(&map, nullptr);
  EXPECT_NE(evaluatorSetupError(&agent, "NaiveEvaluator")
                .find("NaiveEvaluator at '/eval': 'SimulatedSensorEvaluator' requires a map of type 'OccupancyMap'!"),
            std::string::npos);

  agent.factory().setParams("/sensor", {{"type", "IterativeRayCaster"}});
  std::string message;
  try {
    agent.getFactory().createPlugin<SensorModel>("/sensor", agent, false);
  } catch (const std::runtime_error& error) {
    message = error.what();
  }
  EXPECT_EQ(message, "IterativeRayCaster at '/sensor': 'SensorModel' requires a map of type 'OccupancyMap'!");
}

TEST(EvaluatorMapRequirement, BothAcceptTheMapTypeTheyNeed) {
  ConfigAgent agent;
  FreeOccupancyMap occupancy_map(agent);
  agent.provide(&occupancy_map, nullptr);
  EXPECT_EQ(evaluatorSetupError(&agent, "SurfaceFrontierEvaluator"), "");

  FreeTsdfMap tsdf_map(agent);
  agent.provide(&tsdf_map, nullptr);
  EXPECT_EQ(evaluatorSetupError(&agent, "VoxelWeightEvaluator"), "");
}

/// A Generator that is not a GraphGenerator.
class TestFlatGenerator : public Generator {
 public:
  explicit TestFlatGenerator(AgentInterface& agent) : Generator(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  bool rebuildKdTree() override { return true; }
  NodeClusterPtrs generateNewNode(Node* /*start_node*/) override { return {}; }

  static PluginRegistry::Registration<TestFlatGenerator> registration;
};
PluginRegistry::Registration<TestFlatGenerator> TestFlatGenerator::registration("TestFlatGenerator");

class TestSilentEvaluator : public Evaluator {
 public:
  explicit TestSilentEvaluator(AgentInterface& agent) : Evaluator(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  double getRealAccumulatedGain() const override { return 0.0; }

  static PluginRegistry::Registration<TestSilentEvaluator> registration;

 protected:
  bool evaluateUnobserved(const NodeClusterPtrs& /*cluster*/) override { return true; }
};
PluginRegistry::Registration<TestSilentEvaluator> TestSilentEvaluator::registration("TestSilentEvaluator");

TEST(PlanningLayerGenerator, AGeneratorThatIsNotAGraphGeneratorIsRefusedAtSetup) {
  ConfigAgent agent;
  agent.factory().setParams("/layer", {{"type", "PlanningLayer"}});
  agent.factory().setParams("/layer/generator", {{"type", "TestFlatGenerator"}});
  agent.factory().setParams("/layer/evaluator", {{"type", "TestSilentEvaluator"}});
  agent.factory().setParams("/layer/path_planner", {{"type", "NoOpPathPlanner"}});

  std::string message;
  try {
    agent.getFactory().createPlugin<Layer>("/layer", agent, false);
  } catch (const std::runtime_error& error) {
    message = error.what();
  }
  EXPECT_EQ(message.rfind("PlanningLayer at '/layer': PlanningLayer needs a GraphGenerator, got ", 0), 0u) << message;
  EXPECT_NE(message.find("TestFlatGenerator"), std::string::npos) << message;
}

/// The std::invalid_argument createPlugin<PluginType>() throws for @p params at "/plugin", or "" when it validates.
template <class PluginType>
std::string validationError(ConfigAgent* agent, const Configurable::ParamMap& params) {
  agent->factory().setParams("/plugin", params);
  try {
    agent->getFactory().createPlugin<PluginType>("/plugin", *agent, false);
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return "";
}

TEST(ParameterRanges, BeamSearchesNeedABeamOfAtLeastOnePathAndOneEdge) {
  ConfigAgent agent;
  for (const std::string type : {"GraphNodeWiseBeamSearch", "GraphDepthWiseBeamSearch"}) {
    SCOPED_TRACE(type);
    const Configurable::ParamMap valid{{"type", type}, {"objective", "path_gain"}, {"beam_width", "1"}, {"beam_depth", "1"}};
    EXPECT_EQ(validationError<PathPlanner>(&agent, valid), "");

    Configurable::ParamMap params = valid;
    params["beam_width"] = "0";
    EXPECT_EQ(validationError<PathPlanner>(&agent, params), type + " at '/plugin': Invalid parameters: beam_width expected >= 1");
    params = valid;
    params["beam_depth"] = "0";
    EXPECT_EQ(validationError<PathPlanner>(&agent, params), type + " at '/plugin': Invalid parameters: beam_depth expected >= 1");
  }
}

TEST(ParameterRanges, RotateInPlaceNeedsAPositiveStepAndANonNegativeInitialSweep) {
  ConfigAgent agent;
  for (const std::string type : {"RotateInPlace", "RotateReverse"}) {
    SCOPED_TRACE(type);
    EXPECT_EQ(validationError<Backtracker>(&agent, {{"type", type}, {"initial_rotate_angle", "0.0"}}), "");
    EXPECT_NE(validationError<Backtracker>(&agent, {{"type", type}, {"rotation_step", "0.0"}}).find("rotation_step expected > 0.0"),
              std::string::npos);
    EXPECT_NE(validationError<Backtracker>(&agent, {{"type", type}, {"initial_rotate_angle", "-1.0"}})
                  .find("initial_rotate_angle expected >= 0.0"),
              std::string::npos);
  }
}

/// Whether a generator that has seen no setup refuses to split @p edge at @p node_index.
template <class GeneratorType, class NodeType>
bool refusesSplit(Edge* edge, int node_index) {
  ConfigAgent agent;
  GeneratorType generator(agent);
  NodeType start;
  NodeType goal;
  // A refusal hands back the whole edge, ending at the goal.
  const std::pair<Node*, Edge*> result = generator.insertIntermediateNode(&start, &goal, edge, node_index);
  return result.first == &goal && result.second == edge;
}

TEST(IntermediateNodeBounds, AnEmptyTrajectoryIsNotSplit) {
  Edge empty;
  EXPECT_TRUE((refusesSplit<generator::RRAG, GraphNode>(&empty, 0)));
  EXPECT_TRUE((refusesSplit<generator::RRT, TreeNode>(&empty, 0)));
}

TEST(IntermediateNodeBounds, NeitherTheEndNorANegativeIndexIsSplit) {
  Edge edge;
  edge.trajectory.resize(3);
  for (int node_index : {-1, 2, 3}) {
    SCOPED_TRACE(node_index);
    EXPECT_TRUE((refusesSplit<generator::RRAG, GraphNode>(&edge, node_index)));
    EXPECT_TRUE((refusesSplit<generator::RRT, TreeNode>(&edge, node_index)));
  }
}

}  // namespace
}  // namespace open_apero
