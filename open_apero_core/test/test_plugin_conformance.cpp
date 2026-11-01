/**
 * @file test_plugin_conformance.cpp
 * @brief Every plugin type registered in the core builds from a minimal configuration.
 *
 * Each type is created through PluginFactoryConfig in strict mode, reset, and exercised through
 * its base class, including getVisualization() where the base offers one. None needs ROS.
 * SimpleYawPlanningEvaluator also builds its following evaluator from a redirected namespace.
 */

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/gain/gain_anchor.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/graph/graph_node.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/graph/tree_node.h"
#include "open_apero_core/plugin/backtracker/backtracker.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"
#include "open_apero_core/plugin/cost_computer/trajectory_time.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/evaluator/simulated_sensor_evaluator.h"
#include "open_apero_core/plugin/generator/graph_generator.h"
#include "open_apero_core/plugin/layer/layer.h"
#include "open_apero_core/plugin/path_planner/path_planner.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "open_apero_core/plugin/sensor_model/sensor_model.h"
#include "open_apero_core/state/robot_state.h"
#include "open_apero_core/visualization/visualization_markers.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Base class a type is created and exercised as.
enum class Role { kRobot, kCostComputer, kSensorModel, kEvaluator, kGenerator, kPathPlanner, kBacktracker, kLayer };

/// The minimal configuration of one type: its own keys ("" namespace, "type" added) and its children's.
struct Recipe {
  Role role;
  std::map<std::string, Configurable::ParamMap> namespaces;
};

const std::map<std::string, Recipe>& recipes() {
  const Configurable::ParamMap caster{{"type", "IterativeRayCaster"}};
  const Configurable::ParamMap objective{{"objective", "path_gain"}};
  static const std::map<std::string, Recipe> kRecipes = {
      {"SimpleRobot", {Role::kRobot, {}}},
      {"SimpleFallbackRobot", {Role::kRobot, {{"", {{"ompl_planner", "RRTstar"}, {"ompl_log_level", "none"}}}}}},
      {"IdentityCost", {Role::kCostComputer, {}}},
      {"TrajectoryLength", {Role::kCostComputer, {}}},
      {"TrajectoryTime", {Role::kCostComputer, {}}},
      {"IterativeRayCaster", {Role::kSensorModel, {}}},
      {"SimpleRayCaster", {Role::kSensorModel, {}}},
      {"IterativeRayCasterLidar", {Role::kSensorModel, {}}},
      {"NaiveEvaluator", {Role::kEvaluator, {{"/sensor_model", caster}}}},
      {"SurfaceFrontierEvaluator", {Role::kEvaluator, {{"/sensor_model", caster}}}},
      {"VoxelWeightEvaluator", {Role::kEvaluator, {{"/sensor_model", caster}}}},
      {"SimpleYawPlanningEvaluator",
       {Role::kEvaluator, {{"/following_evaluator", {{"type", "NaiveEvaluator"}}}, {"/following_evaluator/sensor_model", caster}}}},
      {"RRAG", {Role::kGenerator, {}}},
      {"RRT", {Role::kGenerator, {}}},
      {"RRTStar", {Role::kGenerator, {}}},
      {"GraphNodeWiseBeamSearch", {Role::kPathPlanner, {{"", objective}}}},
      {"GraphDepthWiseBeamSearch", {Role::kPathPlanner, {{"", objective}}}},
      {"GraphShortestPathTreeSearch", {Role::kPathPlanner, {{"", objective}}}},
      {"GraphTspSearch", {Role::kPathPlanner, {}}},
      {"TreeBestValue", {Role::kPathPlanner, {{"", objective}}}},
      {"NoOpPathPlanner", {Role::kPathPlanner, {}}},
      {"RotateInPlace", {Role::kBacktracker, {}}},
      {"Reverse", {Role::kBacktracker, {}}},
      {"RotateReverse", {Role::kBacktracker, {}}},
      {"PlanningLayer",
       {Role::kLayer,
        {{"/generator", {{"type", "RRAG"}}},
         {"/evaluator", {{"type", "NaiveEvaluator"}}},
         {"/evaluator/sensor_model", caster},
         {"/path_planner", {{"type", "GraphNodeWiseBeamSearch"}, {"objective", "path_gain"}}}}}},
      {"BacktrackingLayer", {Role::kLayer, {{"/backtracker", {{"type", "RotateReverse"}}}}}},
  };
  return kRecipes;
}

/// Types this test binary registers itself, which are not core types.
bool isTestType(const std::string& type) {
  return type.rfind("Test", 0) == 0;
}

RobotState stateAt(const RobotModel& robot, const Eigen::Vector3d& position, double yaw = 0.0) {
  RobotState state;
  robot.placeRobotAtSensorPose(&state, {position, quaternionFromYaw(yaw)});
  return state;
}

/// The world every type is built into: a free TSDF, a set-up SimpleRobot and the agent's cost computer.
class PluginConformanceTest : public ::testing::Test {
 protected:
  PluginConformanceTest() {
    agent_.provide(&map_, &robot_, &cost_computer_);
    Configurable::ParamMap robot_params{{"param_namespace", "/agent_robot"}};
    robot_.setupFromParamMap(&robot_params);
    agent_.factory().setStrictParams(true);
  }

  /// Installs @p recipe for @p type under @p ns.
  void configure(const std::string& type, const Recipe& recipe, const std::string& ns) {
    Configurable::ParamMap own{{"type", type}};
    for (const auto& [child, params] : recipe.namespaces) {
      if (child.empty()) {
        own.insert(params.begin(), params.end());
      } else {
        agent_.factory().setParams(ns + child, params);
      }
    }
    agent_.factory().setParams(ns, own);
  }

  void exercise(const std::string& type, Role role, const std::string& ns);

  ConfigAgent agent_;
  FreeTsdfMap map_{agent_};
  robot_model::SimpleRobot robot_{agent_};
  cost_computer::TrajectoryTime cost_computer_{agent_};
};

void PluginConformanceTest::exercise(const std::string& type, Role role, const std::string& ns) {
  PluginFactory& factory = agent_.getFactory();
  VisualizationMarkers markers;
  const RobotState start = stateAt(robot_, Eigen::Vector3d::Zero());

  switch (role) {
    case Role::kRobot: {
      auto robot = factory.createPlugin<RobotModel>(ns, agent_, false);
      robot->resetEpisode();
      const RobotState goal = stateAt(*robot, Eigen::Vector3d(1.0, 0.0, 0.0));
      RobotTrajectory trajectory;
      EXPECT_TRUE(robot->planTrajectory(start, goal, &map_, &trajectory));
      robot->getVisualization(trajectory, &markers);
      break;
    }
    case Role::kCostComputer: {
      auto cost_computer = factory.createPlugin<CostComputer>(ns, agent_, false);
      cost_computer->resetEpisode();
      Edge edge;
      robot_.planTrajectory(start, stateAt(robot_, Eigen::Vector3d(1.0, 0.0, 0.0)), &map_, &edge.trajectory);
      EXPECT_TRUE(cost_computer->computeCost(&edge));
      EXPECT_GT(edge.cost, 0.0);
      break;
    }
    case Role::kSensorModel: {
      auto sensor_model = factory.createPlugin<SensorModel>(ns, agent_, false);
      sensor_model->resetEpisode();
      GraphNode node;
      node.robot_state = start;
      sensor_model->visualizeSensorView(&markers, &node);
      EXPECT_EQ(markers.getMarkers().size(), 1u);
      VoxelOccupancyMap voxels;
      EXPECT_TRUE(sensor_model->getVisibleVoxelsFromNode(&voxels, &node));
      EXPECT_FALSE(voxels.empty());
      break;
    }
    case Role::kEvaluator: {
      auto evaluator = factory.createPlugin<Evaluator>(ns, agent_, false);
      evaluator->resetEpisode();
      GraphNode node;
      node.robot_state = start;
      if (evaluator->as<evaluator::SimulatedSensorEvaluator>() != nullptr) {
        // An observed, empty world holds nothing unknown to gain.
        EXPECT_TRUE(evaluator->evaluate({&node}));
        EXPECT_DOUBLE_EQ(node.getSingleNodeGain(), 0.0);
      }
      evaluator->getVisualization(&markers, &node);
      evaluator->getPathVisualization(&markers, {&node});
      evaluator->getRealAccumulatedGain();
      break;
    }
    case Role::kGenerator: {
      auto generator = factory.createPlugin<GraphGenerator>(ns, agent_, false);
      generator->resetEpisode();
      Node* root = generator->generateInitNode(start).second;
      ASSERT_NE(root, nullptr);
      EXPECT_TRUE(generator->containsNode(root));
      generator->getVisualization(root, &markers);
      generator->getVisualization(&markers);
      generator->resetEpisode();
      break;
    }
    case Role::kPathPlanner: {
      auto path_planner = factory.createPlugin<PathPlanner>(ns, agent_, false);
      path_planner->resetEpisode();
      if (type == "NoOpPathPlanner") {
        EXPECT_TRUE(path_planner->planPath(nullptr, nullptr).first.empty());
      } else if (type == "TreeBestValue") {
        TreeNode root;
        TreeNode* child = root.spawnChild();
        child->edge.cost = 1.0;
        child->gain_anchors = {std::make_shared<GainAnchor>(1.0)};
        const std::vector<Node*> expected{&root, child};
        EXPECT_EQ(path_planner->planPath(&root, nullptr).first, expected);
      } else {
        GraphType graph;
        auto from = std::make_shared<GraphNode>();
        auto to = std::make_shared<GraphNode>();
        to->gain_anchors = {std::make_shared<GainAnchor>(1.0)};
        const NodeId from_id = boost::add_vertex(from, graph);
        const NodeId to_id = boost::add_vertex(to, graph);
        for (const auto& [u, v] : {std::make_pair(from_id, to_id), std::make_pair(to_id, from_id)}) {
          auto edge = std::make_shared<Edge>();
          edge->cost = 1.0;
          boost::add_edge(u, v, edge, graph);
        }
        const std::vector<Node*> expected{from.get(), to.get()};
        EXPECT_EQ(path_planner->planPath(from.get(), &graph).first, expected);
      }
      break;
    }
    case Role::kBacktracker: {
      auto backtracker = factory.createPlugin<Backtracker>(ns, agent_, false);
      backtracker->resetEpisode();
      backtracker->getVisualization(&markers);
      RobotTrajectory recovery;
      // Reverse replays executed edges and has none yet; the others rotate in place.
      EXPECT_EQ(backtracker->buildTrajectory(start, &recovery), type != "Reverse");
      break;
    }
    case Role::kLayer: {
      auto layer = factory.createPlugin<Layer>(ns, agent_, false);
      layer->resetEpisode();
      layer->onEpisodeStart(start);
      layer->getVisualization(nullptr, &markers);
      layer->resetEpisode();
      break;
    }
  }
}

TEST_F(PluginConformanceTest, EveryRegisteredCoreTypeBuildsResetsAndVisualizes) {
  std::size_t num_exercised = 0;
  for (const std::string& type : PluginRegistry::registeredNames()) {
    if (isTestType(type)) {
      continue;
    }
    SCOPED_TRACE(type);
    const auto recipe = recipes().find(type);
    ASSERT_NE(recipe, recipes().end()) << "no conformance recipe for the registered type '" << type << "'";
    const std::string ns = "/conformance/" + type;
    configure(type, recipe->second, ns);
    ASSERT_NO_THROW(exercise(type, recipe->second.role, ns));
    ++num_exercised;
  }
  // A recipe for a type that is no longer registered would silently test nothing.
  EXPECT_EQ(num_exercised, recipes().size());
}

TEST_F(PluginConformanceTest, SimpleYawPlanningEvaluatorReadsItsFollowingEvaluatorFromTheRedirect) {
  // Nothing at /yaw/following_evaluator: only the redirected namespace can supply the child.
  agent_.factory().setParams("/yaw", {{"type", "SimpleYawPlanningEvaluator"}, {"following_evaluator_args", "/task_evaluator"}});
  agent_.factory().setParams("/task_evaluator", {{"type", "NaiveEvaluator"}, {"visualize_sensor_view", "1"}});
  agent_.factory().setParams("/task_evaluator/sensor_model", {{"type", "IterativeRayCaster"}});

  std::unique_ptr<Evaluator> evaluator;
  ASSERT_NO_THROW(evaluator = agent_.getFactory().createPlugin<Evaluator>("/yaw", agent_, false));
  evaluator->resetEpisode();

  // A node without yaw info goes to the child, whose sensor view is drawn only because the
  // redirected namespace sets visualize_sensor_view.
  GraphNode node;
  node.robot_state = stateAt(robot_, Eigen::Vector3d::Zero());
  VisualizationMarkers markers;
  evaluator->getVisualization(&markers, &node);
  EXPECT_EQ(markers.getMarkers().size(), 1u);
}

}  // namespace
}  // namespace open_apero
