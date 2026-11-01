#pragma once

/**
 * @file plugin_factory.h
 * @brief Factory for configured agent plugins.
 */

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/plugin/plugin.h"
#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {

/**
 * @brief Abstract factory for creating and configuring agent plugins.
 *
 * Subclass this factory to provide concrete parameter sources (e.g., ROS parameters,
 * config files). The factory delegates actual plugin instantiation to the
 * PluginRegistry and then configures each plugin via its parameter map.
 */
class PluginFactory {
 public:
  PluginFactory() = default;

  virtual ~PluginFactory() = default;

  /**
   * @brief Create, configure, and validate a plugin of the given type.
   *
   * Resolves the type from the parameter source, instantiates it via the registry, then runs
   * setup and validation. Total: returns a configured non-null plugin or throws; a missing
   * "type" is a configuration error. Keys the setup never read are reported once
   * (see setStrictParams()).
   *
   * @tparam PluginType The base plugin type to create (e.g., Evaluator).
   * @param args Configuration arguments string (interpretation depends on the concrete factory).
   * @param agent Reference to the agent that will own this plugin.
   * @param verbose If true, print plugin configuration details.
   * @return A unique pointer to the newly created and configured plugin (never null).
   * @throws std::runtime_error if no type is configured, the type is unknown, the
   *         constructed plugin is not a PluginType, a parameter is missing or does not parse
   *         (Configurable::setParam), or strict mode finds an unread key.
   * @throws std::invalid_argument if the plugin's checkParamsValid() rejects the configuration
   *         (Configurable::assureParamsValid).
   * A std::runtime_error or std::invalid_argument raised by setup or validation is thrown again with the
   * prefix "<type> at '<namespace>': ".
   */
  template <class PluginType>
  std::unique_ptr<PluginType> createPlugin(
      const std::string& args,
      AgentInterface& agent,  // NOLINT(google-runtime-references) non-const ref to agent is intentional
      bool verbose) {
    Configurable::ParamMap param_map;
    std::string type;
    getParamMapAndType(&param_map, &type, args);
    if (type.empty()) {
      throw std::runtime_error("PluginFactory: no plugin configured at '" + args + "' (missing 'type'?).");
    }
    // Construct (and type-check) before configuring, so a bad "type" throws here.
    auto plugin = PluginRegistry::instance().createPlugin<PluginType>(type, agent, verbose);
    const std::vector<std::string> supplied_keys = paramKeys(param_map);
    plugin->setVerbose(verbose);
    try {
      plugin->setupFromParamMap(&param_map);
      plugin->assureParamsValid();
    } catch (const std::invalid_argument& error) {
      throw std::invalid_argument(errorContext(type, param_map) + error.what());
    } catch (const std::runtime_error& error) {
      throw std::runtime_error(errorContext(type, param_map) + error.what());
    }
    if (verbose) {
      printVerbose(param_map);
    }
    reportIgnoredParams(type, supplied_keys, param_map);
    return plugin;
  }

  /**
   * @brief Configure an already constructed Configurable (e.g. BoundingVolume) from a parameter source.
   *
   * Runs setup and validation without any registry lookup or type resolution.
   *
   * @param configurable The object to configure (already constructed).
   * @param args Configuration arguments string (interpretation depends on the concrete factory).
   * @param verbose If true, print configuration details.
   * @throws std::runtime_error and std::invalid_argument as createPlugin() does for setup, validation and strict mode;
   *         the prefix names the configurable's class.
   */
  void configure(Configurable& configurable,  // NOLINT(google-runtime-references) mutation of the target is intentional
                 const std::string& args, bool verbose);

  /**
   * @brief List the immediate child sub-namespaces configured under @p ns.
   *
   * Returns final path segments only, sorted and de-duplicated. The default returns none;
   * overrides backed by a parameter tree must throw std::runtime_error on malformed content
   * (a scalar or list where a dictionary is required).
   *
   * @param ns Namespace whose immediate child dictionaries should be listed.
   * @return Sorted child names; empty when @p ns is absent or unconfigured.
   */
  virtual std::vector<std::string> listChildNamespaces(const std::string& ns);

  /**
   * @brief Choose how keys a setup never read are reported: a warning (default) or a throw.
   *
   * Applies to every later createPlugin() and configure(). Reserved keys ("type",
   * "verbose_text", "param_namespace") and keys of a sub-namespace ("child/key") are exempt.
   *
   * @param strict If true, an unread key throws std::runtime_error instead of warning.
   */
  void setStrictParams(bool strict) { strict_params_ = strict; }

  /** @brief Whether an unread key throws (see setStrictParams()). */
  bool isStrictParams() const { return strict_params_; }

  /**
   * @brief Parse configuration arguments into a parameter map and resolve the plugin type.
   *
   * Subclasses implement this to support different configuration sources (ROS params, files, etc.).
   * Leave @p type untouched when the source configures no "type": an empty result means
   * "not configured", which createPlugin reports as an error.
   *
   * Besides the leaf keys, an implementation must set map["param_namespace"] to the resolved namespace (Agent and
   * Layer derive their children's default namespaces from it) and map["verbose_text"] to the header printVerbose()
   * prints; both built-in factories do. Values must be strings Configurable::setParam() parses, e.g. a bool as "1"/"0".
   *
   * @param map Output parameter map to populate.
   * @param type Output string that receives the configured type name, if any.
   * @param args Configuration arguments string.
   * @return True on success. createPlugin() and configure() currently ignore it.
   */
  virtual bool getParamMapAndType(Configurable::ParamMap* map, std::string* type, std::string args) = 0;

 protected:
  /**
   * @brief Print the verbose configuration text accumulated in the parameter map.
   * @param map Parameter map containing a "verbose_text" entry.
   */
  virtual void printVerbose(const Configurable::ParamMap& map) = 0;

  /**
   * @brief Print an error message through the framework's logging mechanism.
   * @param message The error message to display.
   */
  virtual void printError(const std::string& message) = 0;

 public:
  /** @brief The keys of @p map, in map order. */
  static std::vector<std::string> paramKeys(const Configurable::ParamMap& map);

  /**
   * @brief Warn about, or in strict mode throw for, the supplied keys the setup never read.
   * @param label Name used in the message, normally the plugin type.
   * @param supplied_keys Keys the parameter source provided, captured before setup.
   * @param param_map The map after setup, holding the consumed-key record.
   */
  void reportIgnoredParams(const std::string& label, const std::vector<std::string>& supplied_keys,
                           const Configurable::ParamMap& param_map) const;

 private:
  /// @brief The prefix "<label> at '<namespace>': " of a setup or validation error.
  static std::string errorContext(const std::string& label, const Configurable::ParamMap& param_map);

  bool strict_params_ = false;
};

}  // namespace open_apero
