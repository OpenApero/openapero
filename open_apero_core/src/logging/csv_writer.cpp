#include "open_apero_core/logging/csv_writer.h"

namespace open_apero {

bool CsvWriter::open(const std::string& path) {
  file_.open(path.c_str());
  wrote_any_ = false;
  return file_.is_open();
}

void CsvWriter::writeRow(const std::vector<std::string>& cells) {
  if (!file_.is_open()) {
    return;
  }
  if (wrote_any_) {
    file_ << "\n";
  }
  bool first = true;
  for (const auto& cell : cells) {
    if (!first) {
      file_ << ",";
    }
    first = false;
    file_ << cell;
  }
  wrote_any_ = true;
  file_.flush();  // survive a crash mid-run
}

void CsvWriter::close() {
  if (file_.is_open()) {
    file_.close();
  }
}

}  // namespace open_apero
