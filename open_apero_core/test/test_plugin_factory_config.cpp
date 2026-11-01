/**
 * @file test_plugin_factory_config.cpp
 * @brief Focused tests for the ROS-free PluginFactoryConfig.
 */

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_factory_config.h"

namespace open_apero {
namespace {

TEST(PluginFactoryConfigTest, NormalizesNamespacesOnStoreAndLookup) {
  PluginFactoryConfig factory;
  factory.setParams("planner/map/", {{"type", "OccupancyGrid"}, {"voxel_size", "0.1"}});

  // Any equivalent spelling of the namespace resolves to the same entry.
  for (const std::string& query : {"/planner/map", "planner/map", "//planner//map//"}) {
    Configurable::ParamMap map;
    std::string type;
    ASSERT_TRUE(factory.getParamMapAndType(&map, &type, query)) << query;
    EXPECT_EQ(type, "OccupancyGrid") << query;
    EXPECT_EQ(map.at("voxel_size"), "0.1") << query;
    EXPECT_EQ(map.at("param_namespace"), "/planner/map") << query;
    EXPECT_EQ(map.count("type"), 0u) << query;  // "type" is extracted, not a leaf param
  }
}

TEST(PluginFactoryConfigTest, SetParamsReplaces) {
  PluginFactoryConfig factory;
  factory.setParams("/planner/map", {{"type", "A"}, {"stale", "1"}});
  factory.setParams("/planner/map", {{"type", "B"}});

  Configurable::ParamMap map;
  std::string type;
  factory.getParamMapAndType(&map, &type, "/planner/map");
  EXPECT_EQ(type, "B");
  EXPECT_EQ(map.count("stale"), 0u);  // replace, not merge
}

TEST(PluginFactoryConfigTest, AbsentNamespaceYieldsEmptyTypeAndNamespaceKey) {
  PluginFactoryConfig factory;
  Configurable::ParamMap map;
  std::string type;
  ASSERT_TRUE(factory.getParamMapAndType(&map, &type, "/nope"));
  EXPECT_TRUE(type.empty());
  EXPECT_EQ(map.at("param_namespace"), "/nope");
}

TEST(PluginFactoryConfigTest, ListChildNamespacesIsSortedAndImmediate) {
  PluginFactoryConfig factory;
  factory.setParams("/planner/layers/planning", {{"type", "PlanningLayer"}});
  factory.setParams("/planner/layers/planning/evaluator", {{"type", "NaiveEvaluator"}});
  factory.setParams("/planner/layers/backtracking", {{"type", "BacktrackingLayer"}});
  factory.setParams("/planner/map", {{"type", "OccupancyGrid"}});

  const std::vector<std::string> children = factory.listChildNamespaces("/planner/layers");
  // Immediate children only ("planning" appears once despite the nested
  // evaluator), sorted.
  EXPECT_EQ(children, (std::vector<std::string>{"backtracking", "planning"}));
}

TEST(PluginFactoryConfigTest, ListChildNamespacesEmptyWhenAbsent) {
  PluginFactoryConfig factory;
  EXPECT_TRUE(factory.listChildNamespaces("/planner/layers").empty());
}

TEST(PluginFactoryConfigTest, ListsChildrenFromRootNamespace) {
  PluginFactoryConfig factory;
  factory.setParams("/planner", {{"type", "Agent"}});
  factory.setParams("/planner/map", {{"type", "DenseGridMap"}});
  factory.setParams("/experiment", {{"name", "test"}});

  EXPECT_EQ(factory.listChildNamespaces("/"), (std::vector<std::string>{"experiment", "planner"}));
}

TEST(PluginFactoryConfigTest, ClearForgetsEverything) {
  PluginFactoryConfig factory;
  factory.setParams("/planner/map", {{"type", "A"}});
  factory.clear();
  Configurable::ParamMap map;
  std::string type;
  factory.getParamMapAndType(&map, &type, "/planner/map");
  EXPECT_TRUE(type.empty());
}

}  // namespace
}  // namespace open_apero
