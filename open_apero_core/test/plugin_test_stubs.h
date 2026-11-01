#pragma once

/**
 * @file plugin_test_stubs.h
 * @brief Maps, agents and a factory for building core plugins in tests.
 *
 * The maps describe an observed, empty world; ConfigAgent serves parameters from memory through
 * PluginFactoryConfig and records what plugins log. SearchOnlyAgent supplies only what a path
 * search reads, and StubFactory configures nothing.
 */

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "open_apero_core/agent/agent_interface.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/plugin/cost_computer/cost_computer.h"
#include "open_apero_core/plugin/map/map.h"
#include "open_apero_core/plugin/map/occupancy_map.h"
#include "open_apero_core/plugin/map/tsdf_map.h"
#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_factory.h"
#include "open_apero_core/plugin/plugin_factory_config.h"
#include "open_apero_core/plugin/robot_model/robot_model.h"
#include "open_apero_core/state/history_data.h"

namespace open_apero {
namespace {

/// Voxel size of the stub maps [m].
constexpr double kStubVoxelSize = 0.2;

/// Distance to the nearest surface everywhere in the stub TSDF [m].
constexpr double kStubClearance = 10.0;

/// A map that is neither an OccupancyMap nor a TsdfMap.
class PlainMap : public Map {
 public:
  explicit PlainMap(AgentInterface& agent) : Map(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
};

/// An observed, empty world with occupancy but no distance field.
class FreeOccupancyMap : public map::OccupancyMap {
 public:
  explicit FreeOccupancyMap(AgentInterface& agent) : OccupancyMap(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kFree; }
  double getVoxelSize() override { return kStubVoxelSize; }
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    *center = ((point / kStubVoxelSize).array().floor() + 0.5).matrix() * kStubVoxelSize;
    return true;
  }
};

/// An observed, empty world as a TSDF: every point free and kStubClearance from any surface.
class FreeTsdfMap : public map::TsdfMap {
 public:
  explicit FreeTsdfMap(AgentInterface& agent) : TsdfMap(agent) {}
  void setupFromParamMap(ParamMap* /*param_map*/) override {}
  Occupancy getVoxelOccupancy(const Eigen::Vector3d& /*point*/) override { return Occupancy::kFree; }
  double getVoxelSize() override { return kStubVoxelSize; }
  bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) override {
    *center = ((point / kStubVoxelSize).array().floor() + 0.5).matrix() * kStubVoxelSize;
    return true;
  }
  bool getDistanceAtPosition(const Eigen::Vector3d& /*position*/, double* distance) override {
    *distance = kStubClearance;
    return true;
  }
  double getVoxelWeight(const Eigen::Vector3d& /*point*/) override { return 0.0; }
  double getMaximumWeight() override { return 0.0; }
};

/// Cost budget SearchOnlyAgent reports.
constexpr double kSearchBudget = 1000.0;

/// A factory that configures nothing: every namespace yields an empty parameter map.
class StubFactory : public PluginFactory {
 public:
  bool getParamMapAndType(Configurable::ParamMap* map, std::string* /*type*/, std::string /*args*/) override {
    map->clear();
    return true;
  }

 protected:
  void printVerbose(const Configurable::ParamMap& /*map*/) override {}
  void printError(const std::string& /*message*/) override {}
};

/// AgentInterface stand-in for plugin tests: an in-memory PluginFactoryConfig or a factory the test supplies, the
/// test's map, robot and cost computer, a cost budget and a seed; it keeps every line logged to it.
class ConfigAgent : public AgentInterface {
 public:
  const Eigen::Vector3d& getCurrentBasePosition() const override { return zero_; }
  const Eigen::Vector3d& getCurrentSensorPosition() const override { return zero_; }
  const Eigen::Vector3d& getInitialSensorPosition() const override { return zero_; }
  HistoryData& getHistoryData() override { return history_; }
  PluginFactory& getFactory() override { return factory_in_use_ != nullptr ? *factory_in_use_ : factory_; }
  Map& getMap() override {
    if (map_ == nullptr) {
      throw std::logic_error("ConfigAgent: no map provided");
    }
    return *map_;
  }
  RobotModel& getRobot() override {
    if (robot_ == nullptr) {
      throw std::logic_error("ConfigAgent: no robot provided");
    }
    return *robot_;
  }
  CostComputer* getCostComputer() override { return cost_computer_; }
  bool isLoggingPerformance() const override { return log_performance_; }
  double getOriginalCostBudget() const override { return original_budget_; }
  double getCurrentCostBudget() const override { return budget_; }
  double getUsedCostBudget() const override { return original_budget_ - budget_; }
  bool isCostBudgetPositive() const override { return budget_ > 0.0; }
  int getRandomSeed() const override { return random_seed_; }
  void publishVisualization(const VisualizationMarkers& /*markers*/) override {}
  void printInfo(const std::string& text) override { infos.push_back(text); }
  void printWarning(const std::string& text) override { warnings.push_back(text); }
  void printError(const std::string& text) override { errors.push_back(text); }

  /// The in-memory factory, served unless useFactory() named another.
  PluginFactoryConfig& factory() { return factory_; }

  /// Serves @p factory instead of the in-memory one; the caller keeps it alive.
  void useFactory(PluginFactory* factory) { factory_in_use_ = factory; }

  void provide(Map* map, RobotModel* robot, CostComputer* cost_computer = nullptr) {
    map_ = map;
    robot_ = robot;
    cost_computer_ = cost_computer;
  }

  /// Sets the original and the remaining cost budget.
  void setCostBudget(double budget) {
    original_budget_ = budget;
    budget_ = budget;
  }

  /// Sets the remaining cost budget, as spending it in a run does; the original stays.
  void setRemainingCostBudget(double budget) { budget_ = budget; }

  void setRandomSeed(int random_seed) { random_seed_ = random_seed; }

  /// Switches the layers' performance timers on, which stay off by default.
  void setLoggingPerformance(bool log_performance) { log_performance_ = log_performance; }

  /// Does any of @p messages contain @p phrase?
  static bool contains(const std::vector<std::string>& messages, const std::string& phrase) {
    for (const auto& message : messages) {
      if (message.find(phrase) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> infos;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;

 private:
  Eigen::Vector3d zero_ = Eigen::Vector3d::Zero();
  HistoryData history_;
  PluginFactoryConfig factory_;
  PluginFactory* factory_in_use_ = nullptr;
  Map* map_ = nullptr;
  RobotModel* robot_ = nullptr;
  CostComputer* cost_computer_ = nullptr;
  double original_budget_ = 100.0;
  double budget_ = 100.0;
  int random_seed_ = 7;
  bool log_performance_ = false;
};

/// ConfigAgent with kSearchBudget and seed 0, the only things a search reads.
class SearchOnlyAgent : public ConfigAgent {
 public:
  SearchOnlyAgent() {
    setCostBudget(kSearchBudget);
    setRandomSeed(0);
  }
};

}  // namespace
}  // namespace open_apero
