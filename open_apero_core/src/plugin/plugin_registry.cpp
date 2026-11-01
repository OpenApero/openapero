#include "open_apero_core/plugin/plugin_registry.h"

#include <numeric>

namespace open_apero {
namespace {

/// The refused names, in one translation unit for the same reason as the registry map.
std::vector<std::string>& duplicateNamesStorage() {
  static std::vector<std::string> names;
  return names;
}

}  // namespace

PluginRegistry& PluginRegistry::instance() {
  static PluginRegistry instance;
  return instance;
}

PluginRegistry::PluginRegistry() = default;

PluginRegistry::RegistryMap& PluginRegistry::getRegistryMap() {
  // Function-local static with a single definition in this translation unit, so
  // there is exactly one registry map for the whole process even when plugins
  // register from other shared objects.
  static RegistryMap registry_map;
  return registry_map;
}

void PluginRegistry::registerPlugin(const std::string& plugin_name, PluginConstructor constructor) {
  RegistryMap& registry = getRegistryMap();
  if (registry.find(plugin_name) == registry.end()) {
    registry.emplace(plugin_name, std::move(constructor));
  } else {
    logging::error("PluginRegistry: Cannot register already existing plugin name '" + plugin_name + "'!");
    duplicateNamesStorage().push_back(plugin_name);
  }
}

const std::vector<std::string>& PluginRegistry::duplicateNames() {
  return duplicateNamesStorage();
}

const PluginConstructor* PluginRegistry::findConstructor(const std::string& plugin_name) {
  const RegistryMap& registry = getRegistryMap();
  auto it = registry.find(plugin_name);
  return it == registry.end() ? nullptr : &it->second;
}

std::string PluginRegistry::availableTypes() {
  const RegistryMap& m = getRegistryMap();
  if (m.empty()) {
    return std::string();
  }
  return std::accumulate(std::next(m.begin()), m.end(), m.begin()->first,
                         [](const std::string& a, const std::pair<std::string, PluginConstructor>& b) { return a + ", " + b.first; });
}

std::vector<std::string> PluginRegistry::registeredNames() {
  const RegistryMap& m = getRegistryMap();
  std::vector<std::string> names;
  names.reserve(m.size());
  for (const auto& entry : m) {
    names.push_back(entry.first);
  }
  // std::map already orders keys, but state the sorted contract explicitly.
  return names;
}

}  // namespace open_apero
