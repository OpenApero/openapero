#pragma once

/**
 * @file episode_summary.h
 * @brief How an episode ended and its final totals, written as episode_summary.yaml.
 */

#include <string>
#include <utility>
#include <vector>

namespace open_apero {

/// Current summary format. Bump on any change to the keys or their meaning.
constexpr int kEpisodeSummaryFormatVersion = 1;

struct EpisodeSummary {
  int format_version = kEpisodeSummaryFormatVersion;
  std::string end_reason;            ///< budget_exhausted, goal_reached, no_step or stopped.
  int cycles = 0;                    ///< Planning cycles that dispatched a step: the rows of performance_log.csv.
  double total_cost = 0.0;           ///< Sum of executed edge costs.
  double cost_budget = 0.0;          ///< The episode's cost budget; written as .inf when unbounded.
  double total_path_length = 0.0;    ///< [m]
  double total_path_duration = 0.0;  ///< [s]
  int num_visited_nodes = 0;         ///< Executed edges (steps), recovery steps included.
  double elapsed_wall_time_s = 0.0;  ///< Steady clock since episode start [s].
  std::vector<std::pair<std::string, double>> real_accumulated_gain;  ///< Layer instance name -> gain, layers that report one.

  /** @brief Renders the YAML: keys in declaration order, doubles with 15 significant digits. */
  std::string serialize() const;

  /**
   * @brief Writes the YAML to @p path, atomically replacing an earlier summary.
   * @return False on failure, leaving any earlier file and no temporary behind; @p error_message,
   *         when given, names the failed operation and errno cause.
   */
  bool write(const std::string& path, std::string* error_message = nullptr) const;
};

}  // namespace open_apero
