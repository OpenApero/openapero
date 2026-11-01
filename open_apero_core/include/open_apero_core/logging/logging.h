#pragma once

/**
 * @file logging.h
 * @brief Severity-tagged logging for code without an AgentInterface.
 *
 * Plugins log through AgentInterface::printInfo/printWarning/printError, which
 * the Agent forwards here; these free functions also cover call sites without
 * an agent (free functions, static registries). The platform shell installs a
 * sink (e.g. rosconsole) so every message — free-function or agent — reaches one
 * backend; without one, messages go to stdout/stderr with a timestamp.
 *
 * Messages below the configured minimum severity are dropped. Sink and minimum
 * severity are guarded by an internal mutex, so installation and emission may
 * race across threads safely.
 */

#include <functional>
#include <string>

namespace open_apero {
namespace logging {

/// Ordered by increasing severity; setMinSeverity filters against this order.
enum class Severity { kDebug, kInfo, kWarning, kError, kFatal };

using Sink = std::function<void(Severity, const std::string&)>;

/**
 * @brief Replaces the message backend. Pass nullptr to restore the default
 *        timestamped stdout/stderr sink. Safe to call concurrently with logging.
 */
void setSink(Sink sink);

/// @brief Drops messages below @p min_severity. Default is Info (Debug is off).
void setMinSeverity(Severity min_severity);

void debug(const std::string& text);
void info(const std::string& text);
void warning(const std::string& text);
void error(const std::string& text);
void fatal(const std::string& text);

}  // namespace logging
}  // namespace open_apero
