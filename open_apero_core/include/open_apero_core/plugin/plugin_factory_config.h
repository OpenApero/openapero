#pragma once

/**
 * @file plugin_factory_config.h
 * @brief In-memory, ROS-free plugin factory backed by a namespace -> params map.
 */

#include <map>
#include <string>
#include <vector>

#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_factory.h"

namespace open_apero {

/**
 * @brief Concrete PluginFactory that serves parameters from an in-memory
 *        namespace -> {key: value} map, so no ROS parameter server is needed.
 *
 * Namespaces are normalized on storage and lookup (one leading '/', no trailing or duplicated
 * slashes), so "/planner/map", "planner/map/" and "//planner//map" are equivalent.
 */
class PluginFactoryConfig : public PluginFactory {
 public:
  PluginFactoryConfig() = default;
  ~PluginFactoryConfig() override = default;

  /**
   * @brief Install the leaf parameters for one namespace, replacing any existing entry.
   * @param ns Namespace the parameters belong to (normalized on storage).
   * @param params Leaf key -> value map (values are strings; "type" selects the
   *        plugin class). Values are parsed by Configurable::setParam(): write a bool as
   *        "1"/"0" ("true" throws) and a list as "a,b,c".
   */
  void setParams(const std::string& ns, const Configurable::ParamMap& params);

  /// @brief Forget all configured namespaces.
  void clear();

  bool getParamMapAndType(Configurable::ParamMap* map, std::string* type, std::string args) override;

  std::vector<std::string> listChildNamespaces(const std::string& ns) override;

 protected:
  void printVerbose(const Configurable::ParamMap& map) override;
  void printError(const std::string& message) override;

 private:
  /// @brief Normalize a namespace: ensure a single leading '/', drop trailing
  ///        and duplicated slashes. The empty/root namespace normalizes to "/".
  static std::string normalize(const std::string& ns);

  std::map<std::string, Configurable::ParamMap> params_;
};

}  // namespace open_apero
