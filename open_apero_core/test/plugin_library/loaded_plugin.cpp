/**
 * @file loaded_plugin.cpp
 * @brief An evaluator extending NaiveEvaluator, in a shared library of its own that test_plugin_library.cpp loads.
 */

#include "open_apero_core/plugin/evaluator/naive_evaluator.h"
#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {
namespace {

class TestLoadedEvaluator : public evaluator::NaiveEvaluator {
 public:
  explicit TestLoadedEvaluator(AgentInterface& agent) : NaiveEvaluator(agent) {}
};

PluginRegistry::Registration<TestLoadedEvaluator> registration("TestLoadedEvaluator");

}  // namespace
}  // namespace open_apero
