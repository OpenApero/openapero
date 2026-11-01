// VoxbloxMap without a ROS master: only construction. setupFromParamMap() starts a voxblox
// EsdfServer, whose subscribers and publishers register with the master.

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <open_apero_core/agent/agent_interface.h>
#include <open_apero_core/common/typedef.h>
#include <open_apero_core/plugin/map/map.h>
#include <open_apero_core/plugin/map/occupancy_map.h>
#include <open_apero_core/plugin/map/tsdf_map.h>
#include <open_apero_core/plugin/plugin_factory_config.h>
#include <open_apero_core/plugin/plugin_registry.h>
#include <open_apero_core/state/history_data.h>

#include "open_apero_voxblox/plugin/map/voxblox_map.h"

namespace open_apero {
namespace {

/// Agent that provides a factory and nothing else a map constructor could reach for.
class StubAgent : public AgentInterface {
 public:
  const Eigen::Vector3d& getCurrentBasePosition() const override { return zero_; }
  const Eigen::Vector3d& getCurrentSensorPosition() const override { return zero_; }
  const Eigen::Vector3d& getInitialSensorPosition() const override { return zero_; }
  HistoryData& getHistoryData() override { return history_; }
  PluginFactory& getFactory() override { return factory_; }
  Map& getMap() override { throw std::logic_error("StubAgent: no map"); }
  RobotModel& getRobot() override { throw std::logic_error("StubAgent: no robot"); }
  bool isLoggingPerformance() const override { return false; }
  double getOriginalCostBudget() const override { return 0.0; }
  double getCurrentCostBudget() const override { return 0.0; }
  double getUsedCostBudget() const override { return 0.0; }
  bool isCostBudgetPositive() const override { return false; }
  void publishVisualization(const VisualizationMarkers& /*markers*/) override {}
  void printInfo(const std::string& /*text*/) override {}
  void printWarning(const std::string& /*text*/) override {}
  void printError(const std::string& /*text*/) override {}

 private:
  Eigen::Vector3d zero_ = Eigen::Vector3d::Zero();
  HistoryData history_;
  PluginFactoryConfig factory_;
};

TEST(VoxbloxMapTest, IsRegisteredAsATsdfMap) {
  StubAgent agent;
  auto map = PluginRegistry::instance().createPlugin<Map>("VoxbloxMap", agent, false);
  EXPECT_NE(dynamic_cast<map::VoxbloxMap*>(map.get()), nullptr);
  EXPECT_NE(dynamic_cast<map::TsdfMap*>(map.get()), nullptr);
  EXPECT_NE(dynamic_cast<map::OccupancyMap*>(map.get()), nullptr);
}

TEST(VoxbloxMapTest, ConstructsWithoutStartingAServer) {
  StubAgent agent;
  map::VoxbloxMap map(agent);
  SUCCEED();
}

TEST(VoxbloxMapTest, ReportsTheServerKeysAsReadThroughTheMapInterface) {
  StubAgent agent;
  map::VoxbloxMap voxblox_map(agent);
  const Map& base = voxblox_map;
  EXPECT_EQ(base.agentParamsReadByBackend(), map::VoxbloxMap::nodeParamsReadByVoxblox());
}

TEST(VoxbloxNodeParams, ListIsSortedUniqueAndCoversTheLaunchFileKeys) {
  const std::vector<std::string>& keys = map::VoxbloxMap::nodeParamsReadByVoxblox();
  ASSERT_FALSE(keys.empty());
  for (size_t i = 1; i < keys.size(); ++i) {
    EXPECT_LT(keys[i - 1], keys[i]) << keys[i];
  }
  for (const char* launch_key : {"use_tf_transforms", "tsdf_voxel_size", "tsdf_voxels_per_side", "update_esdf_every_n_sec",
                                 "publish_map_every_n_sec", "update_mesh_every_n_sec", "publish_traversable", "truncation_distance"}) {
    EXPECT_TRUE(std::binary_search(keys.begin(), keys.end(), std::string(launch_key))) << launch_key;
  }
}

}  // namespace
}  // namespace open_apero
