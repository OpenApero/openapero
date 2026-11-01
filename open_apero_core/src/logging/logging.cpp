#include "open_apero_core/logging/logging.h"

#include <ctime>
#include <iostream>
#include <mutex>
#include <utility>

namespace open_apero {
namespace logging {

namespace {

std::string timestamp() {
  std::time_t now = std::time(nullptr);
  std::tm tm_local{};
  localtime_r(&now, &tm_local);
  char buffer[20];
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm_local);
  return std::string(buffer);
}

void defaultSink(Severity severity, const std::string& text) {
  switch (severity) {
    case Severity::kDebug:
      std::cout << "[" << timestamp() << "] Debug: " << text << std::endl;
      break;
    case Severity::kInfo:
      std::cout << "[" << timestamp() << "] Info: " << text << std::endl;
      break;
    case Severity::kWarning:
      std::cerr << "[" << timestamp() << "] Warning: " << text << std::endl;
      break;
    case Severity::kError:
      std::cerr << "[" << timestamp() << "] Error: " << text << std::endl;
      break;
    case Severity::kFatal:
      std::cerr << "[" << timestamp() << "] Fatal: " << text << std::endl;
      break;
  }
}

// Guards both the sink and the minimum severity; installation and emission may
// race across threads (ROS callbacks, test setup) without corrupting state.
std::mutex& mutex() {
  static std::mutex instance;
  return instance;
}

Sink& sink() {
  static Sink instance = defaultSink;
  return instance;
}

Severity& minSeverity() {
  static Severity instance = Severity::kInfo;
  return instance;
}

void emit(Severity severity, const std::string& text) {
  std::lock_guard<std::mutex> lock(mutex());
  if (severity < minSeverity()) {
    return;
  }
  sink()(severity, text);
}

}  // namespace

void setSink(Sink new_sink) {
  std::lock_guard<std::mutex> lock(mutex());
  sink() = new_sink ? std::move(new_sink) : defaultSink;
}

void setMinSeverity(Severity min_severity) {
  std::lock_guard<std::mutex> lock(mutex());
  minSeverity() = min_severity;
}

void debug(const std::string& text) {
  emit(Severity::kDebug, text);
}
void info(const std::string& text) {
  emit(Severity::kInfo, text);
}
void warning(const std::string& text) {
  emit(Severity::kWarning, text);
}
void error(const std::string& text) {
  emit(Severity::kError, text);
}
void fatal(const std::string& text) {
  emit(Severity::kFatal, text);
}

}  // namespace logging
}  // namespace open_apero
