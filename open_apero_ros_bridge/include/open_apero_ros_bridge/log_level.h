#pragma once

/**
 * @file log_level.h
 * @brief Wires the node's logging to rosconsole: installs the sink and applies
 *        verbosity (both rosconsole and the core min-severity) from ~log_level.
 */

#include <string>

#include <open_apero_core/logging/logging.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace open_apero {

// Deliberately not in open_apero::ros (unlike this package's conversion
// headers): node files inside `namespace open_apero` refer to ::ros
// unqualified, which a nested ros namespace would shadow.

/**
 * @brief Routes every open_apero logging::* message to rosconsole.
 *
 * Installed once from main() after ros::init, before any agent is constructed,
 * so both agent messages (Agent::printX forwards here) and free-function calls
 * from code without an agent reach the same backend. rosconsole adds its own
 * timestamp/severity/node-name formatting.
 */
inline void installRosconsoleSink() {
  logging::setSink([](logging::Severity severity, const std::string& text) {
    switch (severity) {
      case logging::Severity::kDebug:
        ROS_DEBUG_STREAM(text);
        break;
      case logging::Severity::kInfo:
        ROS_INFO_STREAM(text);
        break;
      case logging::Severity::kWarning:
        ROS_WARN_STREAM(text);
        break;
      case logging::Severity::kError:
        ROS_ERROR_STREAM(text);
        break;
      case logging::Severity::kFatal:
        ROS_FATAL_STREAM(text);
        break;
    }
  });
}

/**
 * @brief Sets rosconsole verbosity AND the core min-severity from ~log_level.
 *
 * Accepted values: debug, info, warn, error, fatal (default: info). Unknown
 * values keep the default and emit a warning. Mapping both keeps the two
 * filters consistent: sub-level messages are dropped before reaching rosconsole.
 *
 * @param nh_private Private node handle the parameter is read from.
 */
inline void setLogLevelFromParam(const ::ros::NodeHandle& nh_private) {
  std::string log_level;
  nh_private.param<std::string>("log_level", log_level, "info");

  ::ros::console::levels::Level level = ::ros::console::levels::Info;
  logging::Severity min_severity = logging::Severity::kInfo;
  if (log_level == "debug") {
    level = ::ros::console::levels::Debug;
    min_severity = logging::Severity::kDebug;
  } else if (log_level == "info") {
    level = ::ros::console::levels::Info;
    min_severity = logging::Severity::kInfo;
  } else if (log_level == "warn") {
    level = ::ros::console::levels::Warn;
    min_severity = logging::Severity::kWarning;
  } else if (log_level == "error") {
    level = ::ros::console::levels::Error;
    min_severity = logging::Severity::kError;
  } else if (log_level == "fatal") {
    level = ::ros::console::levels::Fatal;
    min_severity = logging::Severity::kFatal;
  } else {
    ROS_WARN_STREAM("Unknown ~log_level '" << log_level << "', keeping 'info'.");
  }

  logging::setMinSeverity(min_severity);
  if (::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
    ::ros::console::notifyLoggerLevelsChanged();
  }
}

}  // namespace open_apero
