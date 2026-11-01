#include "open_apero_ros/plugin/plugin_factory_ros.h"

#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace open_apero {
namespace ros {
PluginFactoryRos::PluginFactoryRos() : PluginFactory() {}

std::vector<std::string> PluginFactoryRos::leafParamKeys(const std::vector<std::string>& names, std::string ns) {
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }
  const std::string prefix = ns + "/";
  std::vector<std::string> keys;
  for (const std::string& name : names) {
    // A sibling that only shares the prefix (/a/map_frame for /a/map) is not in the namespace.
    if (name.compare(0, prefix.size(), prefix) != 0) {
      continue;
    }
    const std::string key = name.substr(prefix.size());
    if (!key.empty() && key.find('/') == std::string::npos) {
      keys.push_back(key);
    }
  }
  return keys;
}

// Populate a parameter map by reading all ROS parameters under the given namespace.
// For the ROS factory, `args` is the absolute ROS parameter namespace of the plugin.
// Only leaf parameters (no nested namespaces) are included.
// The "type" parameter specifies which plugin class to instantiate.
bool PluginFactoryRos::getParamMapAndType(Configurable::ParamMap* map, std::string* type, std::string args) {
  std::vector<std::string> names;
  XmlRpc::XmlRpcValue value;
  ::ros::NodeHandle nh(args);
  nh.getParamNames(names);

  // Resolved: absolute, without "//" or a trailing '/', so a child default ns + "/name" is well formed.
  std::string ns = nh.getNamespace();
  if (!ns.empty() && ns.back() == '/') {
    ns.pop_back();  // the root namespace "/"
  }

  for (const std::string& key : leafParamKeys(names, ns)) {
    nh.getParam(key, value);
    std::stringstream ss("");
    ss << value;
    (*map)[key] = ss.str();
  }

  // get type + verbose
  std::string type_default("");
  if (!nh.getParam("type", *type)) {
    type_default = " (default)";
  }
  (*map)["verbose_text"] =
      "\n******************** New Plugin "
      "********************\nCreating Plugin '" +
      *type + "'" + type_default + " from namespace '" + args + "' with parameters:";
  (*map)["param_namespace"] = ns;  // Required by every factory: default args resolve against it.
  return true;
}

// Enumerate the immediate child dictionaries of a namespace by reading it as a
// single XmlRpc struct. Reading the struct wholesale (rather than filtering
// getParamNames) lets us see members that are scalars/lists and reject them,
// instead of silently keeping only the well-formed children.
std::vector<std::string> PluginFactoryRos::listChildNamespaces(const std::string& ns) {
  ::ros::NodeHandle nh;
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(ns, value)) {
    return {};  // Absent: the caller (Agent) reports the missing-'layers' error.
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error("PluginFactoryRos: '" + ns + "' must be a dictionary of named sub-namespaces, not a scalar or list.");
  }
  std::vector<std::string> child_names;
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("PluginFactoryRos: entry '" + it->first + "' under '" + ns +
                               "' must be a dictionary (a nested namespace), not a scalar or list.");
    }
    child_names.push_back(it->first);
  }
  // XmlRpc structs are backed by std::map, so the names come out sorted.
  return child_names;
}

void PluginFactoryRos::printVerbose(const Configurable::ParamMap& map) {
  ROS_INFO("%s",
           map.at("verbose_text").c_str());  // Will warn about formatting otherwise
}

void PluginFactoryRos::printError(const std::string& message) {
  ROS_ERROR("%s", message.c_str());  // Will warn about formatting otherwise
}

}  // namespace ros
}  // namespace open_apero
