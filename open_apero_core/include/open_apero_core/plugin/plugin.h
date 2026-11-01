#pragma once

/**
 * @file plugin.h
 * @brief Base types for configurable objects and plugins.
 */

#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <vector>

#include "open_apero_core/agent/agent_interface.h"

namespace open_apero {

/**
 * @brief Parameter parsing and validation shared by plugins and other configurable objects.
 */
class Configurable {
 public:
  using ParamMap = std::map<std::string, std::string>;

  /** @brief Reserved key recording every key setParam() reads, one per line; its leading 0x01 byte is in no ROS name or plain YAML key. */
  static constexpr const char* kConsumedParamsKey = "\x01consumed_params";

  Configurable();

  virtual ~Configurable() = default;

  /**
   * @brief Set up the object from a parameter map.
   *
   * Besides the configured keys, a PluginFactory supplies "param_namespace" (the namespace the keys came from; a
   * plugin derives its children's default namespaces from it) and "verbose_text".
   *
   * @param param_map Pointer to a map of parameter name-value string pairs.
   */
  virtual void setupFromParamMap(ParamMap* param_map) = 0;

  /**
   * @brief Enable or disable verbose output for this object.
   * @param verbose If true, plugin creation details are printed.
   */
  virtual void setVerbose(bool verbose);

  /**
   * @brief Validate the parameters; throws std::invalid_argument on failure.
   *
   * Call after setupFromParamMap().
   */
  void assureParamsValid();

  /**
   * @brief Check whether the current parameter set is valid.
   * @param error_message Pointer to a string that will be filled with an error description on failure.
   * @return True if parameters are valid, false otherwise. Default implementation always returns true.
   */
  virtual bool checkParamsValid(std::string* error_message);

  /**
   * @brief Keys of @p map that setParam() or markParamConsumed() have read.
   * @param map Parameter map a setup has run on.
   * @return The recorded keys; empty when nothing was read.
   */
  static std::set<std::string> consumedParams(const ParamMap& map);

 protected:
  /**
   * @brief Record @p param_name as read from @p map, for a key parsed without setParam().
   * @param map Parameter map the key belongs to.
   * @param param_name Name of the key that was read.
   */
  static void markParamConsumed(ParamMap* map, const std::string& param_name);

  /**
   * @brief Parse a single parameter from the parameter map and build verbose text.
   *
   * The value is read with operator>>, which must consume all of it: a bool must be "1" or "0" ("true" and "false"
   * throw), and a std::string must be a single non-empty token without whitespace.
   * The key is recorded as consumed (see consumedParams()) whether or not it is present.
   *
   * @tparam T The parameter type (must support stream extraction).
   * @param map Pointer to the parameter map.
   * @param param_name Name of the parameter to look up.
   * @param param Pointer to the variable that will receive the parsed value.
   * @param param_default Optional default value used when the parameter is absent.
   * @throws std::runtime_error if the parameter is absent and has no default, or its value does not parse.
   */
  template <typename T>
  void setParam(ParamMap* map, std::string param_name, T* param, std::optional<T> param_default = std::nullopt) {
    markParamConsumed(map, param_name);
    std::string verbose_text("");
    ParamMap::iterator it = map->find(std::string("verbose_text"));
    if (it != map->end()) {
      verbose_text = it->second + "\n";
    }
    verbose_text += param_name + ": ";

    it = map->find(param_name);
    if (it != map->end()) {
      std::istringstream iss(it->second);
      if (!(iss >> *param) || !iss.eof()) {
        throw std::runtime_error("Failed to parse parameter '" + param_name + "' from value '" + it->second + "'.");
      }
      verbose_text += it->second;
    } else if (param_default.has_value()) {
      *param = param_default.value();
      std::ostringstream oss("");
      oss << *param;
      verbose_text += oss.str() + " (default)";
    } else {
      throw std::runtime_error("Parameter '" + param_name + "' not found in param map!");
    }
    (*map)[std::string("verbose_text")] = verbose_text;
  }

  /**
   * @brief Parse a vector parameter from the parameter map (comma-delimited) and build verbose text.
   *
   * Each comma-separated element is parsed like the scalar overload's value (a bool element is "1" or "0").
   *
   * @tparam T The element type of the vector (must support stream extraction).
   * @param map Pointer to the parameter map.
   * @param param_name Name of the parameter to look up.
   * @param param Pointer to the vector that will receive the parsed values.
   * @param param_default Optional default vector used when the parameter is absent.
   * @throws std::runtime_error if the parameter is absent and has no default, or an element does not parse.
   */
  template <typename T>
  void setParam(ParamMap* map, std::string param_name, std::vector<T>* param, std::optional<std::vector<T>> param_default = std::nullopt) {
    markParamConsumed(map, param_name);
    std::string verbose_text("");
    ParamMap::iterator it = map->find(std::string("verbose_text"));
    if (it != map->end()) {
      verbose_text = it->second + "\n";
    }
    verbose_text += param_name + ": ";

    it = map->find(param_name);
    if (it != map->end()) {
      param->clear();
      std::stringstream ss(it->second);
      std::string token;

      while (std::getline(ss, token, ',')) {
        std::istringstream iss(token);
        T val{};
        if (!(iss >> val) || !iss.eof()) {
          throw std::runtime_error("Failed to parse vector parameter '" + param_name + "' from token '" + token + "'.");
        }
        param->push_back(val);
      }
      verbose_text += it->second;

    } else if (param_default.has_value()) {
      *param = param_default.value();

      std::ostringstream oss("");
      oss << "[";
      for (size_t i = 0; i < (*param).size(); ++i) {
        oss << (*param)[i];
        if (i < (*param).size() - 1) {
          oss << ", ";
        }
      }
      oss << "]";

      verbose_text += oss.str() + " (default)";
    } else {
      throw std::runtime_error("Parameter '" + param_name + "' not found in param map!");
    }

    (*map)[std::string("verbose_text")] = verbose_text;
  }

  /** @brief Whether to display plugin creation info. */
  bool verbose_plugins_ = false;
};

/**
 * @brief Configurable object with access to its owning agent: the base of every plugin.
 */
class Plugin : public Configurable {
 public:
  /**
   * @brief Construct a Plugin with a reference to the owning agent.
   * @param agent Reference to the agent interface.
   */
  explicit Plugin(AgentInterface& agent);

  virtual ~Plugin() = default;

  /**
   * @brief Reset all per-episode plugin state.
   */
  virtual void resetEpisode() = 0;

  /**
   * @brief Safely downcast this plugin to a derived const type.
   * @tparam T Target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  const T* as() const {
    return dynamic_cast<const T*>(this);
  }

  /**
   * @brief Safely downcast this plugin to a derived type.
   * @tparam T Target derived type.
   * @return Pointer to the derived type, or nullptr if the cast fails.
   */
  template <class T>
  T* as() {
    return dynamic_cast<T*>(this);
  }

 protected:
  /** @brief Reference to the owning agent. */
  AgentInterface& agent_;
};
}  // namespace open_apero
