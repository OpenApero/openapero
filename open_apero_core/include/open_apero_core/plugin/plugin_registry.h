#pragma once

/**
 * @file plugin_registry.h
 * @brief Registry for constructing plugins by name.
 */

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "open_apero_core/logging/logging.h"
#include "open_apero_core/plugin/plugin.h"

namespace open_apero {

/** @brief Constructs a plugin for an agent. */
using PluginConstructor =
    std::function<Plugin*(AgentInterface& agent)>;  // NOLINT(google-runtime-references) non-const ref to agent is intentional

/**
 * @brief Singleton registry that maps plugin type names to their constructors.
 *
 * Plugins register themselves at static-initialization time using the nested Registration
 * helper struct, when the executable or a library it links starts up, or when
 * loadPluginLibrary() loads their library. At runtime, the factory looks up a type name in
 * this registry to instantiate the corresponding plugin.
 */
class PluginRegistry {
 public:
  virtual ~PluginRegistry() = default;

  /**
   * @brief Access the singleton instance of the registry.
   * @return Reference to the global PluginRegistry.
   */
  static PluginRegistry& instance();

  /**
   * @brief Create a plugin by its registered type name.
   *
   * Never returns nullptr: throws std::runtime_error if the name is unregistered or the
   * plugin is not a PluginType (destroying it first). The verbose flag is unused.
   *
   * @tparam PluginType The expected base type for the created plugin.
   * @param plugin_name Registered name of the plugin type to create.
   * @param agent Reference to the agent that will own this plugin.
   * @return A unique pointer to the created plugin (always non-null on return).
   */
  template <class PluginType>
  std::unique_ptr<PluginType> createPlugin(
      const std::string& plugin_name,
      AgentInterface& agent,  // NOLINT(google-runtime-references) non-const ref to agent is intentional
      bool /*verbose*/) {
    // Out-of-line lookups keep one registry map inside libopen_apero_core.so, shared by every DSO.
    const PluginConstructor* constructor = findConstructor(plugin_name);
    if (constructor == nullptr) {
      throw std::runtime_error("PluginRegistry: no plugin registered under type '" + plugin_name +
                               "'. Available types are: " + availableTypes() + ".");
    }
    Plugin* raw = (*constructor)(agent);
    PluginType* typed = dynamic_cast<PluginType*>(raw);
    if (typed == nullptr) {
      delete raw;
      throw std::runtime_error("PluginRegistry: plugin '" + plugin_name + "' is not compatible with the requested base type '" +
                               typeid(PluginType).name() + "'.");
    }
    return std::unique_ptr<PluginType>(typed);
  }

  /**
   * @brief Helper struct for static self-registration of plugin types.
   *
   * Declare a static instance of Registration<MyPlugin>("MyPlugin") in a
   * translation unit to register MyPlugin with the factory at program startup.
   * The registered name is the string selected by "type:" in YAML.
   *
   * @tparam PluginType The concrete plugin class to register.
   */
  template <class PluginType>
  struct Registration {
    /**
     * @brief Register a plugin type under the given name.
     * @param plugin_name Unique name to register the plugin under.
     */
    explicit Registration(const std::string& plugin_name) {
      PluginRegistry::registerPlugin(plugin_name, [](AgentInterface& agent) -> Plugin* { return new PluginType(agent); });
    }
  };

  /**
   * @brief Register a constructor under a name. A name that already exists keeps
   *        its constructor; the attempt is logged as an error and recorded in
   *        duplicateNames(). Defined out-of-line so all callers, in any shared
   *        object, mutate the one registry map inside core.
   */
  static void registerPlugin(const std::string& plugin_name, PluginConstructor constructor);

  /**
   * @brief The names registerPlugin() refused because they were already taken, in
   *        the order of the attempts; a name repeats when refused more than once.
   */
  static const std::vector<std::string>& duplicateNames();

  /**
   * @brief Look up a constructor by name.
   * @return Pointer to the stored constructor, or nullptr if unregistered. The
   *         pointer stays valid as long as the plugin is registered (std::map
   *         node addresses are stable across further insertions).
   */
  static const PluginConstructor* findConstructor(const std::string& plugin_name);

  /// @brief Comma-separated list of all registered type names, for error messages.
  static std::string availableTypes();

  /// @brief Sorted list of all registered plugin type names (for introspection).
  static std::vector<std::string> registeredNames();

 private:
  PluginRegistry();

  using RegistryMap = std::map<std::string, PluginConstructor>;

  /// @brief The single registry map, defined out-of-line in plugin_registry.cpp.
  static RegistryMap& getRegistryMap();
};

}  // namespace open_apero
