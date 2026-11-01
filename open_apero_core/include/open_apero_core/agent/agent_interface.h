#pragma once

/**
 * @file agent_interface.h
 * @brief Abstract agent interface providing plugin access to agent state.
 *
 * AgentInterface defines the contract the agent implements so that plugins
 * (evaluators, generators, path planners, etc.) can query robot state, access
 * the map, and publish visualizations without depending on a specific agent
 * implementation.
 */

#include <string>

#include <Eigen/Core>

#include "open_apero_core/state/history_data.h"
#include "open_apero_core/visualization/visualization_markers.h"

namespace open_apero {

// These types depend directly or transitively on AgentInterface through
// Plugin. Including their definitions here would create an include cycle.
class PluginFactory;
class Map;
class RobotModel;
class CostComputer;

/**
 * @brief Abstract interface that plugins use to access agent state and services.
 *
 * Provides accessors for robot position, map, plugins, and utility methods
 * for visualization and logging.
 */
class AgentInterface {
 public:
  virtual ~AgentInterface() = default;

  // state accessors
  /** @brief Whether a current robot state is held. Between episodes there is none and the
   *  position getters throw, so callbacks that can fire then check this first. */
  virtual bool hasCurrentState() const { return true; }

  /** @brief Returns the robot's current base position in the world frame. */
  virtual const Eigen::Vector3d& getCurrentBasePosition() const = 0;

  /** @brief Returns the robot's current sensor position in the world frame. */
  virtual const Eigen::Vector3d& getCurrentSensorPosition() const = 0;

  /** @brief Returns the sensor position at the start of the planning session. */
  virtual const Eigen::Vector3d& getInitialSensorPosition() const = 0;

  /** @brief Returns a reference to the agent's history data. */
  virtual HistoryData& getHistoryData() = 0;

  /** @brief Returns a reference to the plugin factory. */
  virtual PluginFactory& getFactory() = 0;

  /** @brief Returns a reference to the map. */
  virtual Map& getMap() = 0;

  /** @brief Returns a reference to the robot model. */
  virtual RobotModel& getRobot() = 0;

  /** @brief The agent's edge-cost metric, shared by every layer. Agent always
   *  configures one; nullptr when an implementation has none. */
  virtual CostComputer* getCostComputer() { return nullptr; }

  /** @brief Whether the agent is writing a per-cycle CSV log. Used by layers
   *  to decide whether to arm their ScopedTimer instances. */
  virtual bool isLoggingPerformance() const = 0;

  /** @brief Whether high-detail troubleshooting diagnostics are enabled.
   *
   *  Defaults off so external and test AgentInterface implementations do not
   *  need to opt into potentially noisy terminal output.
   */
  virtual bool isDebugDiagnosticsEnabled() const { return false; }

  /** @brief The episode's configured cost budget, shared by all layers: the agent parameter cost_budget (+inf when
   *  unbounded) unless Agent::setCostBudget() overrode it. It does not shrink as the episode spends; see
   *  getCurrentCostBudget(). */
  virtual double getOriginalCostBudget() const = 0;
  /** @brief The budget left: the configured budget minus the cost of every edge executed this episode. Path planners
   *  prune candidate paths that would exceed it. */
  virtual double getCurrentCostBudget() const = 0;
  /** @brief The budget spent so far: getOriginalCostBudget() - getCurrentCostBudget(). */
  virtual double getUsedCostBudget() const = 0;
  /** @brief Whether any budget is left; the agent ends the episode once it is not. */
  virtual bool isCostBudgetPositive() const = 0;

  /** @brief Episode-wide random seed, or -1 for nondeterministic behavior.
   *  Stochastic plugins seed their own engine from this so a run reproduces. */
  virtual int getRandomSeed() const { return -1; }

  // methods
  /**
   * @brief Publishes visualization markers to the output layer (e.g., RViz).
   * @param markers The markers to publish.
   */
  virtual void publishVisualization(const VisualizationMarkers& markers) = 0;

  /**
   * @brief Publishes one named group of visualization markers to its own output
   *        (e.g., a dedicated RViz MarkerArray topic) so each concern -- a planner
   *        layer, the map, the robot footprint, etc. -- can be toggled
   *        independently. Default no-op; ROS agents override.
   * @param name    Topic-relative name identifying this group of markers.
   * @param markers The markers for this group.
   */
  virtual void publishNamedVisualization(const std::string& name, const VisualizationMarkers& markers);

  /**
   * @brief Logs an informational message.
   * @param text The message text.
   */
  virtual void printInfo(const std::string& text) = 0;

  /**
   * @brief Logs a warning message.
   * @param text The message text.
   */
  virtual void printWarning(const std::string& text) = 0;

  /**
   * @brief Logs an error message.
   * @param text The message text.
   */
  virtual void printError(const std::string& text) = 0;
};

inline void AgentInterface::publishNamedVisualization(const std::string& /*name*/, const VisualizationMarkers& /*markers*/) {}

}  // namespace open_apero
