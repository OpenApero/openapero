#include "open_apero_core/plugin/plugin.h"

#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace open_apero {

Configurable::Configurable() = default;

void Configurable::assureParamsValid() {
  std::string error_message("");  // default error
  if (!checkParamsValid(&error_message)) {
    throw std::invalid_argument("Invalid parameters: " + error_message);
  }
}

void Configurable::setVerbose(bool verbose) {
  verbose_plugins_ = verbose;
}

bool Configurable::checkParamsValid(std::string* /*error_message*/) {
  return true;
}

std::set<std::string> Configurable::consumedParams(const ParamMap& map) {
  std::set<std::string> consumed;
  const auto it = map.find(kConsumedParamsKey);
  if (it == map.end()) {
    return consumed;
  }
  std::istringstream lines(it->second);
  std::string key;
  while (std::getline(lines, key)) {
    if (!key.empty()) {
      consumed.insert(key);
    }
  }
  return consumed;
}

void Configurable::markParamConsumed(ParamMap* map, const std::string& param_name) {
  std::string& record = (*map)[kConsumedParamsKey];
  record += param_name;
  record += '\n';
}

Plugin::Plugin(AgentInterface& agent) : agent_(agent) {}

}  // namespace open_apero
