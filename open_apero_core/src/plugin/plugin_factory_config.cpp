#include "open_apero_core/plugin/plugin_factory_config.h"

#include <set>

#include "open_apero_core/logging/logging.h"

namespace open_apero {

std::string PluginFactoryConfig::normalize(const std::string& ns) {
  std::string out;
  out.reserve(ns.size() + 1);
  out.push_back('/');
  bool prev_slash = true;  // leading '/' already emitted; collapse leading input slashes too
  for (char c : ns) {
    if (c == '/') {
      if (!prev_slash) {
        out.push_back('/');
      }
      prev_slash = true;
    } else {
      out.push_back(c);
      prev_slash = false;
    }
  }
  // Drop a trailing slash (but keep the root "/").
  if (out.size() > 1 && out.back() == '/') {
    out.pop_back();
  }
  return out;
}

void PluginFactoryConfig::setParams(const std::string& ns, const Configurable::ParamMap& params) {
  params_[normalize(ns)] = params;  // replace semantics
}

void PluginFactoryConfig::clear() {
  params_.clear();
}

bool PluginFactoryConfig::getParamMapAndType(Configurable::ParamMap* map, std::string* type, std::string args) {
  const std::string ns = normalize(args);
  std::string type_default(" (default)");
  auto it = params_.find(ns);
  if (it != params_.end()) {
    for (const auto& kv : it->second) {
      if (kv.first == "type") {
        *type = kv.second;
        type_default = "";
      } else {
        (*map)[kv.first] = kv.second;
      }
    }
  }
  // Injected keys mirror PluginFactoryRos so downstream plugins behave
  // identically regardless of the configuration source.
  (*map)["verbose_text"] =
      "\n******************** New Plugin "
      "********************\nCreating Plugin '" +
      *type + "'" + type_default + " from namespace '" + ns + "' with parameters:";
  (*map)["param_namespace"] = ns;
  return true;
}

std::vector<std::string> PluginFactoryConfig::listChildNamespaces(const std::string& ns) {
  const std::string normalized = normalize(ns);
  const std::string prefix = normalized == "/" ? "/" : normalized + "/";
  std::set<std::string> children;  // sorted + de-duplicated, matching the ROS factory
  for (const auto& kv : params_) {
    const std::string& full = kv.first;
    if (full.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string rest = full.substr(prefix.size());
    const size_t slash = rest.find('/');
    const std::string child = (slash == std::string::npos) ? rest : rest.substr(0, slash);
    if (!child.empty()) {
      children.insert(child);
    }
  }
  return {children.begin(), children.end()};
}

void PluginFactoryConfig::printVerbose(const Configurable::ParamMap& map) {
  auto it = map.find("verbose_text");
  if (it != map.end()) {
    logging::info(it->second);
  }
}

void PluginFactoryConfig::printError(const std::string& message) {
  logging::error(message);
}

}  // namespace open_apero
