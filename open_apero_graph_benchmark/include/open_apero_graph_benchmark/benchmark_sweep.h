#pragma once

/**
 * @file benchmark_sweep.h
 * @brief What one benchmark sweep consists of, and which files it can produce.
 *
 * Header-only and free of ROS so benchmark_node's executable-only code is not
 * the sole caller: the test binary includes this directly.
 */

#include <regex>
#include <string>
#include <vector>

namespace open_apero {

/**
 * @brief The beam widths one geometric sweep visits: min, min*step, min*step^2, ... while <= @p max_width.
 *
 * benchmark_node computes the widths once, before any run, both to count the planned runs and to run every block.
 *
 * The step is applied only when the product provably cannot exceed @p max_width. Signed overflow is undefined
 * behavior and a sweep bound near INT_MAX reaches it: the node's own default max_beam_width_dbs is 50000000.
 *
 * Total by construction: a non-positive @p min_width or a @p step of 1 or less yields an empty vector rather
 * than looping forever or dividing by zero. main()'s validate_beam_sweep rejects both before any run starts,
 * so those branches are unreachable in production; they exist so this is safe to call with any input.
 */
inline std::vector<int> geometricBeamWidths(int min_width, int max_width, int step) {
  std::vector<int> widths;
  if (min_width <= 0 || step <= 1) {
    return widths;
  }
  for (int width = min_width; width <= max_width;) {
    widths.push_back(width);
    if (width > max_width / step) {
      break;
    }
    width *= step;
  }
  return widths;
}

/**
 * @brief True when @p filename is one the benchmark's own sweeps could have written.
 *
 * Mirrors load_and_group_files() in python/open_apero_graph_benchmark/benchmark_ranges.py exactly: a `.txt` extension, then
 * GROUPING_PATTERN against the *stem* with that function's prefix-match semantics (Python's re.match, not
 * fullmatch). It therefore accepts exactly the files the plot scripts load, `dbs_1_copy.txt` included.
 *
 * The `_` required after the method name keeps names such as `dbsx_1.txt` out, so neither implementation claims
 * them.
 */
inline bool isBenchmarkResultFilename(const std::string& filename) {
  // Constructed once rather than per call: a default sweep asks this of every file in 60 folders.
  static const std::regex kResultPattern(R"(^(nbs|dbs|spt|tsp)_[0-9]+(\.[0-9]+)?)");

  const std::string::size_type dot = filename.rfind('.');
  if (dot == std::string::npos || filename.compare(dot, std::string::npos, ".txt") != 0) {
    return false;
  }
  return std::regex_search(filename.substr(0, dot), kResultPattern);
}

}  // namespace open_apero
