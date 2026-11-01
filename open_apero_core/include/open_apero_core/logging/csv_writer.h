#pragma once

/**
 * @file csv_writer.h
 * @brief Minimal append-only CSV writer with per-row flush.
 *
 * Shared by PerformanceLog (per-cycle metrics) and the map log so both get the
 * same crash-survivable behavior: every row is flushed as it is written, so a
 * hard crash mid-run cannot lose an already-written row. The first row written
 * is the header; rows are newline-separated with no trailing newline.
 */

#include <fstream>
#include <string>
#include <vector>

namespace open_apero {

class CsvWriter {
 public:
  /// @brief Opens (truncating) @p path. Returns false if it cannot be opened.
  bool open(const std::string& path);

  bool isOpen() const { return file_.is_open(); }

  /**
   * @brief Appends one row of pre-formatted cells, comma-joined, and flushes.
   *
   * The first call writes the header (no leading newline); each later row is
   * prefixed with a newline. No-op if the file is not open.
   */
  void writeRow(const std::vector<std::string>& cells);

  void close();

 private:
  std::ofstream file_;
  bool wrote_any_ = false;
};

}  // namespace open_apero
