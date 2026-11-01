#pragma once

/**
 * @file agent_metrics.h
 * @brief Agent-scope metrics: cumulative path totals and timings the agent
 *        itself controls. Per-layer metrics live with each layer; see
 *        layer_metrics.h.
 */

#include "open_apero_core/logging/metrics.h"

namespace open_apero {

/**
 * @brief Per-cycle and cumulative metrics owned by Agent.
 *
 * Per-cycle fields are reset after every replan by resetCycle(); cumulative and row-identity
 * fields only by resetEpisode(). Subclasses add fields by overriding the virtuals and chaining.
 */
struct AgentMetrics : Metrics {
  // --- Per-cycle: timing (seconds) ---
  double plan_path_total_s =
      0.0;  ///< Aggregate Layer::planPath time across every layer attempted this cycle (the priority sort is outside the timer).
  double visualization_s = 0.0;     ///< Time building and publishing all markers (graph, map, robot, path, trajectory).
  double replan_time_wall_s = 0.0;  ///< Wall-clock elapsed time since the previous replan call [s].

  // --- Per-cycle: frame ledger ---
  int commands_dispatched = 0;  ///< Movement commands published since the previous replan.
  int frames_mapped = 0;        ///< Frames the map reports integrating since the previous replan.

  // --- Cumulative: running totals ---
  double total_path_length = 0.0;    ///< Sum of executed trajectory lengths [m].
  double total_path_duration = 0.0;  ///< Sum of executed trajectory durations [s].
  double total_cost = 0.0;           ///< Sum of executed edge costs (unit set by the cost computer).
  int num_visited_nodes = 0;         ///< Executed edges (steps) since planning started, recovery steps included.

  // --- Row identity: written every cycle, monotonic within an episode ---
  int cycle_index = 0;               ///< Zero-based index of this planning cycle within the episode.
  double elapsed_wall_time_s = 0.0;  ///< Monotonic (steady_clock) seconds since episode start.

  void resetCycle() override {
    plan_path_total_s = 0.0;
    visualization_s = 0.0;
    replan_time_wall_s = 0.0;
    commands_dispatched = 0;
    frames_mapped = 0;
  }

  void resetEpisode() override {
    resetCycle();
    total_path_length = 0.0;
    total_path_duration = 0.0;
    total_cost = 0.0;
    num_visited_nodes = 0;
    cycle_index = 0;
    elapsed_wall_time_s = 0.0;
  }

  void declareFields(MetricsFields& fields) const override {
    fields.add("TotalVisitedNodes", &num_visited_nodes, "count");
    fields.add("TotalPathLength", &total_path_length, "m");
    fields.add("TotalPathDuration", &total_path_duration, "s");
    fields.add("TotalCost", &total_cost, "cost");
    fields.add("PlanPathTotal", &plan_path_total_s, "s");
    fields.add("Visualization", &visualization_s, "s");
    fields.add("ReplanTimeWall", &replan_time_wall_s, "s");
    fields.add("CycleIndex", &cycle_index);
    fields.add("ElapsedWallTimeSec", &elapsed_wall_time_s, "s");
    fields.add("CommandsDispatched", &commands_dispatched, "count");
    fields.add("FramesMapped", &frames_mapped, "count");
  }
};

}  // namespace open_apero
