#pragma once

/**
 * @file run_manifest.h
 * @brief Self-describing metadata for one run, written as run_manifest.yaml.
 *
 * Records the column schema (names, types, units, timer nesting), its fingerprint, the build's
 * git provenance, the UTC start time, the map layer instance name -> layer kind (Layer::layerName(), e.g.
 * "planning") and, when provided, the cost budget and the random seed. open_apero_analysis reads it.
 */

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "open_apero_core/logging/performance_log.h"

namespace open_apero {

/// Current manifest format. Bump on any change to the manifest/CSV semantics.
constexpr int kRunManifestFormatVersion = 1;

struct RunManifest {
  int format_version = kRunManifestFormatVersion;
  std::vector<PerformanceLog::Column> columns;              ///< Schema: drives units + fingerprint.
  std::vector<std::pair<std::string, std::string>> layers;  ///< instanceName -> layerName.
  std::optional<double> cost_budget;                        ///< Omitted from YAML when unset.
  std::optional<int> random_seed;                           ///< Agent random_seed (-1 = nondeterministic); omitted when unset.

  /**
   * @brief Renders the complete manifest YAML.
   *
   * Fills the schema fingerprint, git provenance (build-time), and UTC start
   * time internally. Pure: touches no filesystem, so `write()` can fail before
   * it has created anything.
   */
  std::string serialize() const;

  /**
   * @brief Publishes the manifest YAML at `path`, completely or not at all.
   *
   * Returns false, leaving `path` untouched and no temporary behind, if `path` already exists
   * or any step fails; `error_message`, when given, names the failed operation and errno cause.
   */
  bool write(const std::string& path, std::string* error_message = nullptr) const;
};

}  // namespace open_apero
