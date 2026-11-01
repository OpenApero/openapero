#include <stdexcept>
#include <string>
#include <vector>

#include <open_apero_core/logging/logging.h>
#include <open_apero_core/plugin/plugin_library.h>
#include <open_apero_ros_bridge/log_level.h>

#include "open_apero_ros/agent/ros_agent.h"
#include "open_apero_ros/plugin/plugin_factory_ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "open_apero_node");

  // node handles
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Route all open_apero logging to rosconsole once, before any agent exists,
  // and apply the ~log_level filter to both rosconsole and the core.
  open_apero::installRosconsoleSink();
  open_apero::setLogLevelFromParam(nh_private);

  // A plugin library registers its types when it loads; the agent then selects them by name.
  // getParam() also fails on a value of another type, which must not fall back to the default.
  std::vector<std::string> plugin_libraries{"libopen_apero_voxblox.so"};
  if (nh_private.hasParam("plugin_libraries") && !nh_private.getParam("plugin_libraries", plugin_libraries)) {
    throw std::runtime_error("open_apero_node: ~plugin_libraries must be a list of library names.");
  }
  for (const std::string& library : plugin_libraries) {
    open_apero::loadPluginLibrary(library);
    open_apero::logging::info("open_apero_node: loaded plugin library '" + library + "'.");
  }

  open_apero::ros::PluginFactoryRos factory;
  open_apero::Configurable::ParamMap param_map;
  open_apero::ros::RosAgent::setupFactoryAndParams(factory, &param_map, nh_private);

  auto node = open_apero::ros::RosAgent::create(nh, nh_private, &factory, &param_map);
  node->planningLoop();
}
