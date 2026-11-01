/**
 * @file test_plugin_registry.cpp
 * @brief Unit tests for the plugin registry's construction contract and its introspection.
 */

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

// The registry only forwards the agent to plugin constructors, so nothing here is exercised beyond identity.
using FakeAgent = ConfigAgent;

// Two unrelated plugin bases, so we can ask the registry for an incompatible one.
class FakeRoleA : public Plugin {
 public:
  explicit FakeRoleA(AgentInterface& agent) : Plugin(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
};

class FakeRoleB : public Plugin {
 public:
  explicit FakeRoleB(AgentInterface& agent) : Plugin(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
};

class RegisteredA : public FakeRoleA {
 public:
  explicit RegisteredA(AgentInterface& agent) : FakeRoleA(agent) {}
  static PluginRegistry::Registration<RegisteredA> registration;
};
PluginRegistry::Registration<RegisteredA> RegisteredA::registration("TestRegisteredA");

TEST(PluginRegistry, CreatesRegisteredType) {
  FakeAgent agent;
  auto plugin = PluginRegistry::instance().createPlugin<FakeRoleA>("TestRegisteredA", agent, false);
  ASSERT_NE(plugin, nullptr);
  EXPECT_NE(dynamic_cast<RegisteredA*>(plugin.get()), nullptr);
}

TEST(PluginRegistry, UnknownTypeThrowsAndListsAvailableTypes) {
  FakeAgent agent;
  try {
    PluginRegistry::instance().createPlugin<FakeRoleA>("NoSuchType", agent, false);
    FAIL() << "expected std::runtime_error for an unregistered type";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("NoSuchType"), std::string::npos);
    // The message must help the user recover by naming what *is* available.
    EXPECT_NE(msg.find("Available types are"), std::string::npos);
    EXPECT_NE(msg.find("TestRegisteredA"), std::string::npos);
  }
}

TEST(PluginRegistry, IncompatibleBaseTypeThrows) {
  FakeAgent agent;
  // TestRegisteredA is a FakeRoleA, never a FakeRoleB.
  EXPECT_THROW(PluginRegistry::instance().createPlugin<FakeRoleB>("TestRegisteredA", agent, false), std::runtime_error);
}

TEST(PluginRegistry, RegisteredNamesAreSortedAndIncludeCorePlugins) {
  const std::vector<std::string> names = PluginRegistry::registeredNames();
  EXPECT_TRUE(std::is_sorted(names.begin(), names.end()));
  // A couple of always-registered core plugins must be present.
  EXPECT_NE(std::find(names.begin(), names.end(), "PlanningLayer"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "NaiveEvaluator"), names.end());
}

}  // namespace
}  // namespace open_apero
