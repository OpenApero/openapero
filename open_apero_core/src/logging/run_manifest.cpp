#include "open_apero_core/logging/run_manifest.h"

#include <ctime>
#include <sstream>
#include <string>

#include "open_apero_core/logging/build_info.h"
#include "open_apero_core/logging/schema_fingerprint.h"
#include "yaml_file.h"

namespace open_apero {

using yaml_file::writeFileAtomically;
using yaml_file::yamlDouble;
using yaml_file::yamlQuote;

namespace {

std::string utcNowIso8601() {
  std::time_t now = std::time(nullptr);
  std::tm tm_utc{};
  gmtime_r(&now, &tm_utc);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return std::string(buffer);
}

}  // namespace

std::string RunManifest::serialize() const {
  std::ostringstream out;

  out << "format_version: " << format_version << "\n";
  out << "schema_fingerprint: " << yamlQuote(schemaFingerprint(columns)) << "\n";
  out << "build_git_sha: " << yamlQuote(build_info::kGitSha) << "\n";
  out << "build_git_dirty: " << (build_info::kGitDirty ? "true" : "false") << "\n";
  // Every repository that contributed source, named, because build_git_sha above
  // describes only the innermost one. A nested build needs each enclosing
  // superproject's revision too: that is what decides which code actually ran.
  out << "build_components:\n";
  for (std::size_t i = 0; i < build_info::kNumComponents; ++i) {
    const auto& component = build_info::kComponents[i];
    out << "  - {name: " << yamlQuote(component.name) << ", sha: " << yamlQuote(component.sha)
        << ", dirty: " << (component.dirty ? "true" : "false") << "}\n";
  }
  out << "start_utc: " << yamlQuote(utcNowIso8601()) << "\n";

  if (cost_budget.has_value()) {
    out << "cost_budget: " << yamlDouble(*cost_budget) << "\n";
  }
  if (random_seed.has_value()) {
    out << "random_seed: " << *random_seed << "\n";
  }
  out << "layers:\n";
  for (const auto& [instance_name, layer_name] : layers) {
    out << "  " << yamlQuote(instance_name) << ": " << yamlQuote(layer_name) << "\n";
  }

  // The column schema, in output order. Keys match the fingerprint field order
  // (name, type, unit, parent) so a reader can recompute the fingerprint.
  out << "columns:\n";
  for (const auto& column : columns) {
    out << "  - {name: " << yamlQuote(column.name) << ", type: " << yamlQuote(column.type) << ", unit: " << yamlQuote(column.unit)
        << ", parent: " << yamlQuote(column.qualified_parent) << "}\n";
  }

  return out.str();
}

bool RunManifest::write(const std::string& path, std::string* error_message) const {
  if (error_message) error_message->clear();

  // Render before touching the filesystem: a serialization problem then cannot
  // leave anything behind.
  const std::string content = serialize();

  // Callers rely on an existing manifest never being overwritten.
  return writeFileAtomically(path, content, /*replace_existing=*/false, error_message);
}

}  // namespace open_apero
