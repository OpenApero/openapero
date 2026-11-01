/**
 * @file test_param_tracking.cpp
 * @brief Keys a plugin's setup never reads are reported once: a warning, or a throw in strict mode.
 *
 * setParam() records what it reads; the factory reports the rest of the namespace, sparing reserved
 * keys, keys of a sub-namespace and keys a base class or a hand parser consumed.
 */

#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/geometry/bounding_volume.h"
#include "open_apero_core/logging/logging.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"
#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"
#include "open_apero_core/plugin/robot_model/simple_robot.h"
#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// Reads one key with setParam() and one by hand, which it then records itself.
class TestHandParsedCostComputer : public CostComputer {
 public:
  explicit TestHandParsedCostComputer(AgentInterface& agent) : CostComputer(agent) {}

  void setupFromParamMap(ParamMap* param_map) override {
    setParam<double>(param_map, "unit_cost", &unit_cost_, 1.0);
    if (param_map->find("hand_parsed_scale") != param_map->end()) {
      markParamConsumed(param_map, "hand_parsed_scale");
    }
  }

  bool computeCost(Edge* edge) override {
    edge->cost = unit_cost_;
    return true;
  }

  static PluginRegistry::Registration<TestHandParsedCostComputer> registration;

 private:
  double unit_cost_ = 1.0;
};
PluginRegistry::Registration<TestHandParsedCostComputer> TestHandParsedCostComputer::registration("TestHandParsedCostComputer");

/// Collects every warning logged while it is alive.
class WarningCapture {
 public:
  WarningCapture() {
    logging::setSink([this](logging::Severity severity, const std::string& text) {
      if (severity == logging::Severity::kWarning) {
        warnings.push_back(text);
      }
    });
  }
  ~WarningCapture() { logging::setSink(nullptr); }

  /// Warnings that report ignored parameters.
  std::vector<std::string> ignoredReports() const {
    std::vector<std::string> reports;
    for (const auto& warning : warnings) {
      if (warning.find("ignores parameters") != std::string::npos) {
        reports.push_back(warning);
      }
    }
    return reports;
  }

  std::vector<std::string> warnings;
};

class ParamTrackingTest : public ::testing::Test {
 protected:
  ParamTrackingTest() { agent_.provide(&map_, nullptr); }

  ConfigAgent agent_;
  FreeTsdfMap map_{agent_};
};

TEST(ConsumedParams, SetParamRecordsEveryKeyItReads) {
  class Reader : public Configurable {
   public:
    void setupFromParamMap(ParamMap* param_map) override {
      setParam<int>(param_map, "present", &value_, 0);
      setParam<int>(param_map, "absent", &value_, 0);
      std::vector<double> list;
      setParam<double>(param_map, "list", &list, std::vector<double>());
    }

   private:
    int value_ = 0;
  };

  Configurable::ParamMap params{{"present", "3"}, {"list", "1.0, 2.0"}, {"unread", "x"}};
  Reader reader;
  reader.setupFromParamMap(&params);
  EXPECT_EQ(Configurable::consumedParams(params), (std::set<std::string>{"present", "absent", "list"}));
  EXPECT_TRUE(Configurable::consumedParams(Configurable::ParamMap{}).empty());
}

TEST(ConsumedParams, ASuppliedKeyNamedLikeTheRecordIsAnOrdinaryKey) {
  const Configurable::ParamMap params{{"consumed_params", "present"}};
  EXPECT_TRUE(Configurable::consumedParams(params).empty());
}

TEST_F(ParamTrackingTest, AnUnknownKeyIsReportedOnceAsAWarning) {
  agent_.factory().setParams("/cost", {{"type", "IdentityCost"}, {"bogus", "1"}, {"also_bogus", "2"}});
  WarningCapture capture;
  EXPECT_NO_THROW(agent_.factory().createPlugin<CostComputer>("/cost", agent_, false));
  const auto reports = capture.ignoredReports();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_NE(reports.front().find("IdentityCost ignores parameters: also_bogus, bogus"), std::string::npos) << reports.front();
  EXPECT_NE(reports.front().find("'/cost'"), std::string::npos) << reports.front();
}

TEST_F(ParamTrackingTest, StrictModeThrowsForAnUnknownKey) {
  agent_.factory().setParams("/cost", {{"type", "IdentityCost"}, {"bogus", "1"}});
  agent_.factory().setStrictParams(true);
  EXPECT_TRUE(agent_.factory().isStrictParams());
  try {
    agent_.factory().createPlugin<CostComputer>("/cost", agent_, false);
    FAIL() << "expected std::runtime_error for the unread key";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("IdentityCost ignores parameters: bogus"), std::string::npos) << error.what();
  }
}

TEST_F(ParamTrackingTest, StrictModeDefaultsOff) {
  EXPECT_FALSE(agent_.factory().isStrictParams());
}

TEST_F(ParamTrackingTest, KeysOfAChildNamespaceAreNotReportedByTheParent) {
  // The sensor model's keys live in its own namespace; one also spelled as a sub-key of the parent.
  agent_.factory().setParams("/eval", {{"type", "NaiveEvaluator"}, {"sensor_model/ray_length", "3.0"}});
  agent_.factory().setParams("/eval/sensor_model", {{"type", "IterativeRayCaster"}, {"ray_length", "3.0"}, {"focal_length", "320.0"}});
  agent_.factory().setStrictParams(true);
  WarningCapture capture;
  EXPECT_NO_THROW(agent_.factory().createPlugin<Evaluator>("/eval", agent_, false));
  EXPECT_TRUE(capture.ignoredReports().empty());
}

TEST_F(ParamTrackingTest, KeysReadByABaseClassOrARedirectAreNotReported) {
  // bounding_volume_args is read by Evaluator, the frontier ratios by SimulatedSensorEvaluator and
  // sensor_model_args moves the child to another namespace.
  agent_.factory().setParams("/eval", {{"type", "NaiveEvaluator"},
                                       {"bounding_volume_args", "/volume"},
                                       {"frontier_min_unknown_ratio", "0.8"},
                                       {"sensor_model_args", "/caster"}});
  agent_.factory().setParams("/caster", {{"type", "IterativeRayCaster"}});
  agent_.factory().setStrictParams(true);
  EXPECT_NO_THROW(agent_.factory().createPlugin<Evaluator>("/eval", agent_, false));
}

TEST_F(ParamTrackingTest, AKeyRecordedByAHandParserIsNotReported) {
  agent_.factory().setParams("/cost", {{"type", "TestHandParsedCostComputer"}, {"unit_cost", "2.0"}, {"hand_parsed_scale", "3"}});
  agent_.factory().setStrictParams(true);
  EXPECT_NO_THROW(agent_.factory().createPlugin<CostComputer>("/cost", agent_, false));
}

TEST_F(ParamTrackingTest, AChildReportsItsOwnUnknownKeys) {
  agent_.factory().setParams("/eval", {{"type", "NaiveEvaluator"}});
  agent_.factory().setParams("/eval/sensor_model", {{"type", "IterativeRayCaster"}, {"bogus", "1"}});
  WarningCapture capture;
  agent_.factory().createPlugin<Evaluator>("/eval", agent_, false);
  const auto reports = capture.ignoredReports();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_NE(reports.front().find("IterativeRayCaster ignores parameters: bogus"), std::string::npos) << reports.front();
}

TEST_F(ParamTrackingTest, ConfigureReportsTheConfigurablesUnknownKeys) {
  agent_.factory().setParams("/volume", {{"x_min", "-1.0"}, {"x_max", "1.0"}, {"color", "red"}});
  WarningCapture capture;
  BoundingVolume volume;
  agent_.factory().configure(volume, "/volume", false);
  const auto reports = capture.ignoredReports();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_NE(reports.front().find("BoundingVolume ignores parameters: color"), std::string::npos) << reports.front();

  agent_.factory().setStrictParams(true);
  BoundingVolume strict_volume;
  EXPECT_THROW(agent_.factory().configure(strict_volume, "/volume", false), std::runtime_error);
}

TEST_F(ParamTrackingTest, AShippedStyleRobotBlockReadsEveryKey) {
  // The robot keys of open_apero_ros/config/planners/_common.yaml, as the ROS factory would pass them.
  agent_.factory().setParams("/robot", {{"type", "SimpleFallbackRobot"},
                                        {"visualize", "0"},
                                        {"max_linear_velocity", "0.5"},
                                        {"max_yaw_rate", "1.6"},
                                        {"time_step", "0.2"},
                                        {"sensor_mount_type", "fixed"},
                                        {"collision_sphere_x", "0.0"},
                                        {"collision_sphere_y", "0.0"},
                                        {"collision_sphere_z", "0.0"},
                                        {"collision_sphere_r", "0.3"},
                                        {"bounding_volume_args", "/map_bounding_volume"},
                                        {"ompl_planner", "RRTstar"},
                                        {"ompl_state_space_type", "R2"},
                                        {"ompl_log_level", "none"},
                                        {"ompl_planning_time", "0.002"},
                                        {"ompl_resolution", "0.10"},
                                        {"ompl_max_range", "3.0"},
                                        {"safe_radius", "0.0"},
                                        {"num_orientation_candidates", "8"},
                                        {"sample_z", "1"}});
  agent_.factory().setStrictParams(true);
  EXPECT_NO_THROW(agent_.factory().createPlugin<RobotModel>("/robot", agent_, false));
}

}  // namespace
}  // namespace open_apero
