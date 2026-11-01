#include <algorithm>
#include <exception>
#include <filesystem>
#include <regex>
#include <string>

#include <open_apero_ros_bridge/log_level.h>
#include <ros/ros.h>

#include "open_apero_graph_benchmark/graph_manager.h"

using namespace open_apero;

std::string getNextGraphFolder(const std::string& base_folder) {
  int max_index = -1;
  std::regex graph_folder_pattern(R"(graph_(\d+))");

  for (const auto& entry : std::filesystem::directory_iterator(base_folder)) {
    if (entry.is_directory()) {
      std::smatch match;
      std::string folder_name = entry.path().filename().string();
      if (std::regex_match(folder_name, match, graph_folder_pattern)) {
        int index = std::stoi(match[1].str());
        max_index = std::max(max_index, index);
      }
    }
  }

  return base_folder + "/graph_" + std::to_string(max_index + 1);
}

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "generate_graph_instance_node");

  ::ros::NodeHandle nh("~");

  // Routes open_apero_core's logging::* (the gain-model messages) through rosconsole.
  open_apero::installRosconsoleSink();

  // The inputs are read only; the instance goes below output_folder, which the launch file puts below the data root.
  auto require_param = [&nh](const std::string& name, std::string& value) {
    if (nh.getParam(name, value) && !value.empty()) {
      return true;
    }
    ROS_FATAL_STREAM("Required parameter '" << name << "' is unset or empty.");
    return false;
  };
  std::string topology_file;
  std::string gain_model_file;
  std::string output_folder;
  if (!require_param("topology_file", topology_file) || !require_param("gain_model_file", gain_model_file) ||
      !require_param("output_folder", output_folder)) {
    return 1;
  }

  // Negative means nondeterministic, matching the repository-wide seeding convention.
  int random_seed;
  nh.param("random_seed", random_seed, -1);

  GraphManager graph_manager;
  graph_manager.setGainSamplingSeed(random_seed);

  try {
    graph_manager.readGraphDataFromFile(topology_file);
    graph_manager.generateGraphFromData(0);

    graph_manager.readGainModelFromFile(gain_model_file);
    graph_manager.assignNewGains();

    std::filesystem::create_directories(output_folder);
    std::string new_graph_folder = getNextGraphFolder(output_folder);
    std::filesystem::create_directories(new_graph_folder);

    std::string new_graph_filepath = new_graph_folder + "/graph.txt";
    graph_manager.writeGraphToFile(new_graph_filepath);
    ROS_INFO_STREAM("Wrote new graph instance to " << new_graph_filepath);
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Cannot generate a graph instance in " << output_folder << ": " << error.what());
    return 1;
  }

  return 0;
}
