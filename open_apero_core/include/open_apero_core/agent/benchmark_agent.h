#pragma once

/**
 * @file benchmark_agent.h
 * @brief Agent without a robot: movement finishes at once and visualization is discarded, so the planner runs
 *        without ROS, as the graph benchmark drives it.
 */

#include "open_apero_core/agent/agent.h"

namespace open_apero {

/**
 * @brief Agent stub that runs without a live robot, so path-planner benchmarks
 *        can drive the graph search without hardware.
 *
 * Overrides sendMovement with a no-op that immediately finishes the cursor. A caller that wants markers, as the
 * graph benchmark node does, publishes them itself; the in-class publishVisualization is a no-op.
 */
class BenchmarkAgent : public Agent {
 public:
  /**
   * @brief Construct a BenchmarkAgent.
   * @param factory Plugin factory; the benchmark passes an empty PluginFactoryConfig, since it builds the planners
   *        itself and configures them through setupRaw().
   * @param param_map Parameter map (may be nullptr for raw benchmarks).
   */
  BenchmarkAgent(PluginFactory* factory, Configurable::ParamMap* param_map) : Agent(factory, param_map) {}

  ~BenchmarkAgent() override = default;

  /**
   * @brief No-op visualization; markers are discarded.
   * @param markers Visualization markers (ignored).
   */
  void publishVisualization([[maybe_unused]] const VisualizationMarkers& markers) override {}

  /**
   * @brief Set the episode random seed.
   *
   * Benchmarks construct the agent with a null parameter map, so setupFromParamMap() never runs and the seed
   * would otherwise keep its nondeterministic default (-1). Path planners pull this through
   * AgentInterface::getRandomSeed() when seeding their tie-break RNGs.
   * @param random_seed Episode seed; negative means nondeterministic.
   */
  void setRandomSeed(int random_seed) { p_random_seed_ = random_seed; }

 protected:
  /**
   * @brief No-op movement send; simulates a successful execution by immediately finishing the cursor.
   * @param cursor The edge cursor containing the edge to execute.
   * @return Always true.
   */
  bool sendMovement(EdgeCursor* cursor) override {
    if (cursor) {
      cursor->finish();
    }
    return true;
  }
};

}  // namespace open_apero
