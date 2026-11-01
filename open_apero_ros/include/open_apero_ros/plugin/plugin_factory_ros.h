#pragma once

/**
 * @file plugin_factory_ros.h
 * @brief ROS parameter server-based plugin factory for constructing agent plugins.
 */

#include <string>
#include <vector>

#include <open_apero_core/plugin/plugin_factory.h>

namespace open_apero {
namespace ros {

/**
 * @brief Concrete plugin factory that reads plugin parameters from the ROS parameter server.
 *
 * Extends PluginFactory to resolve plugin types and parameter maps from
 * the ROS parameter server namespace hierarchy.
 */
class PluginFactoryRos : public PluginFactory {
  friend PluginFactory;

 public:
  ~PluginFactoryRos() = default;

  /// @brief Default constructor.
  PluginFactoryRos();

  /**
   * @brief List the immediate child sub-namespaces of @p ns on the ROS parameter server.
   *
   * Reads @p ns as a single XmlRpc struct and returns its member names. Throws
   * if @p ns is present but is not a dictionary, or if any direct member is not
   * itself a dictionary (so a stray scalar next to valid layer sections is a
   * hard error rather than being silently skipped). Returns an empty list when
   * @p ns is absent.
   *
   * @param ns Absolute ROS parameter namespace to enumerate.
   * @return Member names, sorted (XmlRpc structs are map-backed).
   */
  std::vector<std::string> listChildNamespaces(const std::string& ns) override;

  /**
   * @brief Select the leaf parameters directly inside a namespace.
   * @param names Absolute parameter names, as ros::NodeHandle::getParamNames() lists them.
   * @param ns Absolute namespace; a trailing '/' is ignored.
   * @return The names below `ns + "/"` with no further '/', relative to it.
   */
  static std::vector<std::string> leafParamKeys(const std::vector<std::string>& names, std::string ns);

 protected:
  /**
   * @brief Retrieve plugin parameters and type string from the ROS parameter server.
   * @param map Output parameter map populated from the ROS parameter namespace.
   * @param type Output string set to the plugin type (e.g., from the "type" param).
   * @param args ROS namespace used to look up parameters.
   * @return True on success.
   */
  bool getParamMapAndType(Configurable::ParamMap* map, std::string* type, std::string args) override;

  /**
   * @brief Print the parameter map contents as a ROS info message.
   * @param map The parameter map to print.
   */
  void printVerbose(const Configurable::ParamMap& map) override;

  /**
   * @brief Print an error message via ROS logging.
   * @param message The error string to output.
   */
  void printError(const std::string& message) override;
};

}  // namespace ros
}  // namespace open_apero
