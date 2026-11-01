/**
 * @file duplicate_plugin.cpp
 * @brief A shared library that registers the name loaded_plugin.cpp takes, so loading it after that one is refused.
 */

#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {
namespace {

class TestDuplicatePlugin : public Plugin {
 public:
  explicit TestDuplicatePlugin(AgentInterface& agent) : Plugin(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  void resetEpisode() override {}
};

PluginRegistry::Registration<TestDuplicatePlugin> registration("TestLoadedEvaluator");

}  // namespace
}  // namespace open_apero
