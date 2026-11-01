#include "open_apero_core/logging/episode_summary.h"

#include <sstream>
#include <string>

#include "yaml_file.h"

namespace open_apero {

using yaml_file::writeFileAtomically;
using yaml_file::yamlDouble;
using yaml_file::yamlQuote;

std::string EpisodeSummary::serialize() const {
  std::ostringstream out;
  out << "format_version: " << format_version << "\n";
  out << "end_reason: " << end_reason << "\n";
  out << "cycles: " << cycles << "\n";
  out << "total_cost: " << yamlDouble(total_cost) << "\n";
  out << "cost_budget: " << yamlDouble(cost_budget) << "\n";
  out << "total_path_length: " << yamlDouble(total_path_length) << "\n";
  out << "total_path_duration: " << yamlDouble(total_path_duration) << "\n";
  out << "num_visited_nodes: " << num_visited_nodes << "\n";
  out << "elapsed_wall_time_s: " << yamlDouble(elapsed_wall_time_s) << "\n";
  out << "real_accumulated_gain:" << (real_accumulated_gain.empty() ? " {}" : "") << "\n";
  for (const auto& [instance_name, gain] : real_accumulated_gain) {
    out << "  " << yamlQuote(instance_name) << ": " << yamlDouble(gain) << "\n";
  }
  return out.str();
}

bool EpisodeSummary::write(const std::string& path, std::string* error_message) const {
  if (error_message) error_message->clear();
  // A later summary of the run replaces an earlier one.
  return writeFileAtomically(path, serialize(), /*replace_existing=*/true, error_message);
}

}  // namespace open_apero
