#pragma once

/**
 * @file performance_log.h
 * @brief CSV writer for the per-cycle performance log.
 */

#include <string>
#include <utility>
#include <vector>

#include "open_apero_core/logging/csv_writer.h"
#include "open_apero_core/logging/metrics.h"

namespace open_apero {

/**
 * @brief Writes registered metric groups to a per-cycle performance CSV.
 *
 * Header and rows derive from the same field declarations; column prefixes are applied here.
 * Registered Metrics objects must outlive this log.
 */
class PerformanceLog {
 public:
  /**
   * @brief Registers a metric group's fields as columns.
   *
   * Must be called for all groups before open(); registering afterwards
   * throws std::logic_error, as the header is already written. Columns appear
   * in registration order, fields in declaration order within each group.
   *
   * @param metrics The metric group; must outlive this log.
   * @param prefix  Optional column prefix, emitted as "<prefix>.<Name>".
   */
  void addMetrics(const Metrics& metrics, const std::string& prefix = "");

  /**
   * @brief Opens the CSV file and writes the header line.
   * @param file_path Path of the CSV file to create.
   * @return True if the file was opened successfully.
   */
  bool open(const std::string& file_path);

  bool isOpen() const { return csv_.isOpen(); }

  /**
   * @brief Appends one row with the current value of every registered field.
   *
   * Flushes after the row so data survives a crash mid-episode.
   */
  void writeRow();

  void close();

  /**
   * @brief One resolved output column: name, live value, and manifest metadata.
   *
   * Populated at addMetrics(); the run manifest reads these to record units and
   * the fully-qualified timer nesting, and to compute the schema fingerprint.
   */
  struct Column {
    std::string name;              ///< Fully-qualified column name (prefix applied).
    MetricsFields::Value value;    ///< Live field pointer read on writeRow().
    std::string type;              ///< "double" or "int".
    std::string unit;              ///< Unit string, or "" if none/dynamic.
    std::string qualified_parent;  ///< Resolved parent column name, or "".
  };

  /// Registered columns in output order (for the manifest / fingerprint).
  const std::vector<Column>& columns() const { return columns_; }

 private:
  std::vector<Column> columns_;
  CsvWriter csv_;
};

}  // namespace open_apero
