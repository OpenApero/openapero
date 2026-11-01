/**
 * @file test_plugin_library.cpp
 * @brief Tests for loading plugin libraries at runtime.
 *
 * The libraries under test/plugin_library/ are built as shared libraries nothing links; CMake
 * passes their paths in as OPEN_APERO_TEST_LOADED_PLUGIN_LIBRARY and OPEN_APERO_TEST_DUPLICATE_PLUGIN_LIBRARY.
 */

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/plugin/evaluator/evaluator.h"
#include "open_apero_core/plugin/evaluator/naive_evaluator.h"
#include "open_apero_core/plugin/plugin_library.h"
#include "open_apero_core/plugin/plugin_registry.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

bool isRegistered(const std::string& name) {
  const std::vector<std::string> names = PluginRegistry::registeredNames();
  return std::find(names.begin(), names.end(), name) != names.end();
}

TEST(PluginLibrary, AMissingLibraryThrowsWithItsName) {
  const std::string library = "libopen_apero_no_such_plugin_library.so";
  try {
    loadPluginLibrary(library);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find(library), std::string::npos) << error.what();
  }
}

TEST(PluginLibrary, LoadingRegistersAnEvaluatorThatExtendsACoreEvaluator) {
  loadPluginLibrary(OPEN_APERO_TEST_LOADED_PLUGIN_LIBRARY);
  ASSERT_TRUE(isRegistered("TestLoadedEvaluator"));
  ConfigAgent agent;
  // Built as an Evaluator, as a layer builds its evaluator; NaiveEvaluator lives in the core library.
  auto evaluator = PluginRegistry::instance().createPlugin<Evaluator>("TestLoadedEvaluator", agent, false);
  EXPECT_NE(dynamic_cast<evaluator::NaiveEvaluator*>(evaluator.get()), nullptr);
}

TEST(PluginLibrary, LoadingALoadedLibraryAgainRegistersNothingTwice) {
  loadPluginLibrary(OPEN_APERO_TEST_LOADED_PLUGIN_LIBRARY);
  const std::size_t num_duplicates = PluginRegistry::duplicateNames().size();
  EXPECT_NO_THROW(loadPluginLibrary(OPEN_APERO_TEST_LOADED_PLUGIN_LIBRARY));
  EXPECT_EQ(PluginRegistry::duplicateNames().size(), num_duplicates);
}

TEST(PluginLibrary, ALibraryRegisteringATakenNameIsRefused) {
  loadPluginLibrary(OPEN_APERO_TEST_LOADED_PLUGIN_LIBRARY);
  try {
    loadPluginLibrary(OPEN_APERO_TEST_DUPLICATE_PLUGIN_LIBRARY);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("TestLoadedEvaluator"), std::string::npos) << error.what();
  }
  // The first registration stands.
  EXPECT_TRUE(isRegistered("TestLoadedEvaluator"));
}

}  // namespace
}  // namespace open_apero
