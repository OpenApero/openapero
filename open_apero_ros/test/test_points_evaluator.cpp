// PointsEvaluator without a ROS master: registry construction, point loading and collection.
// setupFromParamMap() advertises /collectible_points and so needs a master; these tests bypass it.

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <open_apero_core/agent/agent_interface.h>
#include <open_apero_core/common/typedef.h>
#include <open_apero_core/geometry/bounding_volume.h>
#include <open_apero_core/graph/edge.h>
#include <open_apero_core/graph/node.h>
#include <open_apero_core/plugin/evaluator/evaluator.h>
#include <open_apero_core/plugin/map/occupancy_map.h>
#include <open_apero_core/plugin/plugin_factory_config.h>
#include <open_apero_core/plugin/plugin_registry.h>
#include <open_apero_core/state/history_data.h>
#include <open_apero_core/state/robot_state.h>

#include "open_apero_ros/plugin/evaluator/points_evaluator.h"

namespace open_apero {
namespace {

/// Observed, empty world: every voxel is free.
class FreeMap : public map::OccupancyMap {
 public:
  explicit FreeMap(AgentInterface& agent) : OccupancyMap(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kFree; }
  double getVoxelSize() override { return 0.1; }
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    *center = point;
    return true;
  }
};

/// Agent that owns nothing but a factory and records warnings.
class StubAgent : public AgentInterface {
 public:
  const Eigen::Vector3d& getCurrentBasePosition() const override { return zero_; }
  const Eigen::Vector3d& getCurrentSensorPosition() const override { return zero_; }
  const Eigen::Vector3d& getInitialSensorPosition() const override { return zero_; }
  HistoryData& getHistoryData() override { return history_; }
  PluginFactory& getFactory() override { return factory_; }
  Map& getMap() override { return map_; }
  RobotModel& getRobot() override { throw std::logic_error("StubAgent: no robot"); }
  bool isLoggingPerformance() const override { return false; }
  double getOriginalCostBudget() const override { return 100.0; }
  double getCurrentCostBudget() const override { return 100.0; }
  double getUsedCostBudget() const override { return 0.0; }
  bool isCostBudgetPositive() const override { return true; }
  void publishVisualization(const VisualizationMarkers& /*markers*/) override {}
  void printInfo(const std::string& /*text*/) override {}
  void printWarning(const std::string& text) override { warnings.push_back(text); }
  void printError(const std::string& text) override { errors.push_back(text); }

  std::vector<std::string> warnings;
  std::vector<std::string> errors;

 private:
  Eigen::Vector3d zero_ = Eigen::Vector3d::Zero();
  HistoryData history_;
  PluginFactoryConfig factory_;
  FreeMap map_{*this};
};

/// Exposes the loading path setupFromParamMap() would take, with an inactive (all-containing) volume.
class PointsEvaluatorHarness : public evaluator::PointsEvaluator {
 public:
  PointsEvaluatorHarness(AgentInterface& agent, map::OccupancyMap* map) : PointsEvaluator(agent) {
    bounding_volume_ = std::make_unique<BoundingVolume>();
    map_ = map;
  }
  void load(const std::string& filename) { loadPointsFromFile(filename); }
  std::size_t numPoints() const { return points_.size(); }
};

std::string writeCsv(const std::string& text) {
  char path[] = "/tmp/open_apero_points_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_GE(fd, 0);
  close(fd);
  std::ofstream(path) << text;
  return path;
}

TEST(PointsEvaluatorTest, IsRegisteredAsAnEvaluator) {
  StubAgent agent;
  auto evaluator = PluginRegistry::instance().createPlugin<Evaluator>("PointsEvaluator", agent, false);
  ASSERT_NE(dynamic_cast<evaluator::PointsEvaluator*>(evaluator.get()), nullptr);
  EXPECT_EQ(evaluator->getRealAccumulatedGain(), 0.0);
}

TEST(PointsEvaluatorTest, LoadsFiveColumnRowsAndSkipsTheRest) {
  StubAgent agent;
  FreeMap map(agent);
  PointsEvaluatorHarness evaluator(agent, &map);
  const std::string path = writeCsv("1,0,0,0.5,3\n\n2,0,0,0.5\n5,0,0,0.5,7,extra\nx,0,0,0.5,1\n");
  evaluator.load(path);
  std::remove(path.c_str());

  EXPECT_EQ(evaluator.numPoints(), 2u);
  ASSERT_EQ(agent.warnings.size(), 1u);  // the non-numeric row; the short row is skipped silently
  EXPECT_EQ(evaluator.getRealAccumulatedGain(), 0.0);
}

TEST(PointsEvaluatorTest, CollectsAPointTheTrajectoryPassesAndResetsPerEpisode) {
  StubAgent agent;
  FreeMap map(agent);
  PointsEvaluatorHarness evaluator(agent, &map);
  const std::string path = writeCsv("1,0,0,0.5,3\n5,0,0,0.5,7\n");
  evaluator.load(path);
  std::remove(path.c_str());

  RobotState passing;
  passing.setSensorPosition(Eigen::Vector3d(1.2, 0.0, 0.0));
  Edge edge;
  edge.trajectory.push_back(passing);
  Node node;
  node.robot_state = passing;

  evaluator.onNodeReached(&node, &edge);
  EXPECT_EQ(evaluator.getRealAccumulatedGain(), 3.0);
  evaluator.onNodeReached(&node, &edge);  // a collected point counts once
  EXPECT_EQ(evaluator.getRealAccumulatedGain(), 3.0);

  evaluator.resetEpisode();
  EXPECT_EQ(evaluator.getRealAccumulatedGain(), 0.0);
}

}  // namespace
}  // namespace open_apero
