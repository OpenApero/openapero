#include "open_apero_graph_benchmark/benchmark_status.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace open_apero {
namespace {
// Each panel column is both padded to and truncated at the same width, so name it once. Passing different
// values to std::setw and truncateText would silently misalign the panel.
constexpr std::size_t kNameColumnWidth = 13;
constexpr std::size_t kReplanColumnWidth = 9;
constexpr std::size_t kPhaseColumnWidth = 12;

std::string truncateText(const std::string& text, std::size_t width) {
  if (text.size() <= width) {
    return text;
  }
  if (width <= 3) {
    return text.substr(0, width);
  }
  return text.substr(0, width - 3) + "...";
}

std::string alignRight(std::string text, std::size_t width) {
  if (text.size() > width) {
    text.resize(width);
  }
  return std::string(width - text.size(), ' ') + text;
}

std::string formatScientific(double value, std::size_t width, bool show_positive) {
  for (int precision = 2; precision >= 0; --precision) {
    std::ostringstream ss;
    if (show_positive) {
      ss << std::showpos;
    }
    ss << std::scientific << std::setprecision(precision) << value;
    std::string text = ss.str();
    const std::string::size_type exponent_plus = text.find("e+");
    if (exponent_plus != std::string::npos) {
      text.erase(exponent_plus + 1, 1);
    }
    if (text.size() <= width) {
      return alignRight(text, width);
    }
  }
  return std::string(width, '#');
}

std::string formatNumber(double value, std::size_t width, bool show_positive = false) {
  std::string special;
  if (std::isnan(value)) {
    special = "nan";
  } else if (std::isinf(value)) {
    special = value < 0.0 ? "-inf" : (show_positive ? "+inf" : "inf");
  }
  if (!special.empty()) {
    return alignRight(special, width);
  }

  std::ostringstream fixed;
  if (show_positive) {
    fixed << std::showpos;
  }
  fixed << std::fixed << std::setprecision(1) << value;
  if (fixed.str().size() <= width) {
    return alignRight(fixed.str(), width);
  }
  return formatScientific(value, width, show_positive);
}

template <typename Integer>
std::string formatInteger(Integer value, std::size_t width) {
  const std::string decimal = std::to_string(value);
  if (decimal.size() <= width) {
    return alignRight(decimal, width);
  }
  return formatScientific(static_cast<double>(value), width, false);
}

}  // namespace

std::string formatBenchmarkStatus(const BenchmarkStatus& status) {
  std::ostringstream ss;
  ss << "Test  " << std::left << std::setw(kNameColumnWidth) << truncateText(status.planner, kNameColumnWidth) << "on "
     << std::setw(kNameColumnWidth) << truncateText(status.graph_instance, kNameColumnWidth) << "\n"
     << "Using " << std::setw(kNameColumnWidth) << truncateText(status.objective, kNameColumnWidth) + ","
     << "replan is " << std::setw(kReplanColumnWidth) << truncateText(status.replan_type, kReplanColumnWidth) << '\n'
     << std::right << "step = " << formatInteger(status.step, 5) + ","
     << "  # node = " << formatInteger(status.node_id, 5) << ",  " << truncateText(status.phase, kPhaseColumnWidth) << "\n"
     << "gain " << formatNumber(status.step_gain, 6, true) << " -> " << formatNumber(status.total_gain, 8) << "\n"
     << "cost " << formatNumber(status.total_cost, 6) << '/' << formatNumber(status.cost_budget, 8);
  return ss.str();
}

}  // namespace open_apero
