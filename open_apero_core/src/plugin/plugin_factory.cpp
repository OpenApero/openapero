#include "open_apero_core/plugin/plugin_factory.h"

#include <cxxabi.h>

#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include "open_apero_core/logging/logging.h"
#include "open_apero_core/plugin/plugin.h"

namespace open_apero {

namespace {

// Keys the factories inject or reserve; no setup is expected to read them.
bool isReservedParamKey(const std::string& key) {
  return key == "type" || key == "verbose_text" || key == "param_namespace" || key == Configurable::kConsumedParamsKey;
}

std::string demangledTypeName(const std::type_info& type) {
  int status = 0;
  std::unique_ptr<char, void (*)(void*)> name(abi::__cxa_demangle(type.name(), nullptr, nullptr, &status), std::free);
  return (status == 0 && name != nullptr) ? std::string(name.get()) : std::string(type.name());
}

}  // namespace

void PluginFactory::configure(Configurable& configurable, const std::string& args, bool verbose) {
  Configurable::ParamMap param_map;
  // A directly-constructed Configurable has no registry type; the resolved type
  // string is irrelevant here and simply ignored.
  std::string type;
  getParamMapAndType(&param_map, &type, args);
  const std::vector<std::string> supplied_keys = paramKeys(param_map);
  configurable.setVerbose(verbose);
  const std::string label = demangledTypeName(typeid(configurable));
  try {
    configurable.setupFromParamMap(&param_map);
    configurable.assureParamsValid();
  } catch (const std::invalid_argument& error) {
    throw std::invalid_argument(errorContext(label, param_map) + error.what());
  } catch (const std::runtime_error& error) {
    throw std::runtime_error(errorContext(label, param_map) + error.what());
  }
  if (verbose) {
    printVerbose(param_map);
  }
  reportIgnoredParams(label, supplied_keys, param_map);
}

std::string PluginFactory::errorContext(const std::string& label, const Configurable::ParamMap& param_map) {
  const auto ns = param_map.find("param_namespace");
  return label + " at '" + (ns != param_map.end() ? ns->second : std::string()) + "': ";
}

std::vector<std::string> PluginFactory::listChildNamespaces(const std::string& /*ns*/) {
  return {};
}

std::vector<std::string> PluginFactory::paramKeys(const Configurable::ParamMap& map) {
  std::vector<std::string> keys;
  keys.reserve(map.size());
  for (const auto& entry : map) {
    keys.push_back(entry.first);
  }
  return keys;
}

void PluginFactory::reportIgnoredParams(const std::string& label, const std::vector<std::string>& supplied_keys,
                                        const Configurable::ParamMap& param_map) const {
  const std::set<std::string> consumed = Configurable::consumedParams(param_map);
  std::string ignored;
  for (const std::string& key : supplied_keys) {
    if (isReservedParamKey(key) || key.find('/') != std::string::npos || consumed.count(key) > 0) {
      continue;
    }
    ignored += (ignored.empty() ? "" : ", ") + key;
  }
  if (ignored.empty()) {
    return;
  }

  std::string message = label + " ignores parameters: " + ignored;
  const auto ns = param_map.find("param_namespace");
  if (ns != param_map.end()) {
    message += " (namespace '" + ns->second + "')";
  }
  if (strict_params_) {
    throw std::runtime_error(message);
  }
  logging::warning(message);
}

}  // namespace open_apero
