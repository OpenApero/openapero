#include "open_apero_core/logging/performance_log.h"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <variant>

namespace open_apero {

namespace {

std::string qualify(const std::string& prefix, const std::string& name) {
  return prefix.empty() ? name : prefix + "." + name;
}

std::string resolveParent(const ParentRef& parent, const std::string& prefix) {
  switch (parent.scope) {
    case ParentRef::Scope::kLocal:
      return qualify(prefix, parent.name);
    case ParentRef::Scope::kGlobal:
      return parent.name;
    case ParentRef::Scope::kNone:
      break;
  }
  return "";
}

std::string typeName(const MetricsFields::Value& value) {
  return std::holds_alternative<const int*>(value) ? "int" : "double";
}

}  // namespace

void PerformanceLog::addMetrics(const Metrics& metrics, const std::string& prefix) {
  if (csv_.isOpen()) {
    // A column added now would produce row values the header does not declare.
    throw std::logic_error("PerformanceLog::addMetrics called after open().");
  }
  MetricsFields fields;
  metrics.declareFields(fields);
  for (const auto& field : fields.fields()) {
    columns_.push_back({qualify(prefix, field.name), field.value, typeName(field.value), field.unit, resolveParent(field.parent, prefix)});
  }
}

bool PerformanceLog::open(const std::string& file_path) {
  if (!csv_.open(file_path)) {
    return false;
  }
  std::vector<std::string> header;
  header.reserve(columns_.size());
  for (const auto& column : columns_) {
    header.push_back(column.name);
  }
  csv_.writeRow(header);
  return true;
}

void PerformanceLog::writeRow() {
  if (!csv_.isOpen()) {
    return;
  }
  std::vector<std::string> cells;
  cells.reserve(columns_.size());
  for (const auto& column : columns_) {
    // Doubles get 15 significant digits in the default format (0.1 stays "0.1"); ints are unaffected.
    std::ostringstream cell;
    cell << std::setprecision(15);
    std::visit([&cell](const auto* field) { cell << *field; }, column.value);
    cells.push_back(cell.str());
  }
  csv_.writeRow(cells);
}

void PerformanceLog::close() {
  csv_.close();
}

}  // namespace open_apero
