#pragma once

/**
 * @file benchmark_status.h
 * @brief Fixed-shape text of the graph benchmark's RViz status overlay.
 */

#include <cstddef>
#include <string>

namespace open_apero {

/// Compact values rendered in the graph benchmark's RViz status panel.
struct BenchmarkStatus {
  std::string graph_instance;
  std::string planner;
  std::string objective;
  std::string replan_type;
  std::string phase;
  std::size_t step = 0;
  int node_id = 0;
  double step_gain = 0.0;
  double total_gain = 0.0;
  double total_cost = 0.0;
  double cost_budget = 0.0;
};

constexpr std::size_t kBenchmarkStatusLineCount = 5;
constexpr std::size_t kBenchmarkStatusMaxColumns = 63;

/**
 * @brief Format a fixed five-line status panel without any line exceeding
 *        kBenchmarkStatusMaxColumns monospace characters.
 *
 * Lines: planner and graph instance; objective and replanning mode; step, current node and phase; step and total
 * gain; cost against the budget.
 */
std::string formatBenchmarkStatus(const BenchmarkStatus& status);

}  // namespace open_apero
