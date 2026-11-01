// PluginFactoryRos's namespace filter, on parameter names as getParamNames() lists them.

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_ros/plugin/plugin_factory_ros.h"

namespace open_apero {
namespace ros {
namespace {

using Keys = std::vector<std::string>;

TEST(PluginFactoryRosTest, TakesTheLeavesDirectlyInsideTheNamespace) {
  const Keys names = {"/node/map/type", "/node/map/voxel_size", "/node/map/child/type", "/other/map/type"};
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/node/map"), (Keys{"type", "voxel_size"}));
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/node/map/"), (Keys{"type", "voxel_size"}));
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/node/map/child"), (Keys{"type"}));
  EXPECT_TRUE(PluginFactoryRos::leafParamKeys(names, "/missing").empty());
}

TEST(PluginFactoryRosTest, IgnoresSiblingsThatOnlyShareThePrefix) {
  // map_frame and map_args share /node/map's text prefix but lie outside its namespace.
  const Keys names = {"/node/map/type", "/node/map_frame", "/node/map_args", "/node/mapping/type"};
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/node/map"), (Keys{"type"}));
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/node/mapping"), (Keys{"type"}));
}

TEST(PluginFactoryRosTest, SkipsTheNamespaceItselfWhenItIsAParameter) {
  EXPECT_TRUE(PluginFactoryRos::leafParamKeys({"/node/map"}, "/node/map").empty());
}

TEST(PluginFactoryRosTest, ResolvesAnAbsoluteRedirectOutsideTheNodeNamespace) {
  // evaluator_args: /task_evaluator, as the shipped planner configs set it.
  const Keys names = {"/node/layers/planning/type", "/node/layers/planning/evaluator_args", "/task_evaluator/type",
                      "/task_evaluator/frontier_min_unknown_ratio", "/task_evaluator/sensor_model/type"};
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/node/layers/planning"), (Keys{"type", "evaluator_args"}));
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/task_evaluator"), (Keys{"type", "frontier_min_unknown_ratio"}));
  EXPECT_EQ(PluginFactoryRos::leafParamKeys(names, "/task_evaluator/sensor_model"), (Keys{"type"}));
}

}  // namespace
}  // namespace ros
}  // namespace open_apero
