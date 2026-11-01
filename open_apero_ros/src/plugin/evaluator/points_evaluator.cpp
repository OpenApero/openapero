#include "open_apero_ros/plugin/evaluator/points_evaluator.h"

#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <open_apero_msgs/CollectiblePoint.h>
#include <open_apero_msgs/CollectiblePointArray.h>

namespace open_apero {
namespace evaluator {

PluginRegistry::Registration<PointsEvaluator> PointsEvaluator::registration("PointsEvaluator");

PointsEvaluator::PointsEvaluator(AgentInterface& agent) : SimulatedSensorEvaluator(agent) {}

double PointsEvaluator::getRealAccumulatedGain() const {
  double total_gain = 0.0;
  for (const auto& point : points_) {
    auto point_sphere = point->as<PointSphere>();
    if (point_sphere->status == PointStatus::kCollected) {
      total_gain += point_sphere->gain;
    }
  }

  return total_gain;
}

void PointsEvaluator::setupFromParamMap(Configurable::ParamMap* param_map) {
  SimulatedSensorEvaluator::setupFromParamMap(param_map);

  setParam<std::string>(param_map, "points_file", &p_points_file_, "");
  if (p_points_file_ != "") {
    loadPointsFromFile(p_points_file_);
  }

  // Public handle with a relative name: resolves to the same /collectible_points the
  // visualizer subscribes to, but stays remappable from a launch file.
  ::ros::NodeHandle nh;
  collectible_point_pub_ = nh.advertise<open_apero_msgs::CollectiblePointArray>("collectible_points", 1);
}

void PointsEvaluator::onNodeReached(Node* reached_node, const Edge* traversed_edge) {
  SimulatedSensorEvaluator::onNodeReached(reached_node, traversed_edge);

  for (auto& point : points_) {
    auto point_sphere = point->as<PointSphere>();

    const Eigen::Vector3d& position = point_sphere->position;
    if (point_sphere->status == PointStatus::kUnseen && map_->getVoxelOccupancy(position) == Occupancy::kFree) {
      point_sphere->status = PointStatus::kObserved;
    }

    if (point_sphere->status != PointStatus::kCollected && traversed_edge) {
      for (const auto& robot_state : traversed_edge->trajectory) {
        double dist = (position - robot_state.getSensorPosition()).norm();
        if (dist < point_sphere->radius) {
          point_sphere->status = PointStatus::kCollected;
          break;
        }
      }
    }
  }
}

bool PointsEvaluator::evaluateFromVisibleVoxels(Node* node) {
  node->gain_anchors.clear();
  if (!node->info) {
    return false;
  }

  // Check if there are any observed but not collected points around the node
  for (auto& point : points_) {
    auto point_sphere = point->as<PointSphere>();

    if (point_sphere->status == PointStatus::kObserved &&
        (node->robot_state.getSensorPosition() - point_sphere->position).norm() < point_sphere->radius) {
      node->gain_anchors.push_back(point);
    }
  }

  return true;
}

void PointsEvaluator::getVisualization(VisualizationMarkers* markers, const Node* node) {
  SimulatedSensorEvaluator::getVisualization(markers, node);

  open_apero_msgs::CollectiblePointArray point_array_msg;
  open_apero_msgs::CollectiblePoint point_msg;
  for (auto& point : points_) {
    const auto& point_sphere = point->as<PointSphere>();
    point_msg.id = point_sphere->id;
    point_msg.x = point_sphere->position.x();
    point_msg.y = point_sphere->position.y();
    point_msg.z = point_sphere->position.z();
    point_msg.radius = point_sphere->radius;
    point_msg.gain = point_sphere->gain;

    switch (point_sphere->status) {
      case PointStatus::kUnseen:
        point_msg.status = open_apero_msgs::CollectiblePoint::UNSEEN;
        break;
      case PointStatus::kObserved:
        point_msg.status = open_apero_msgs::CollectiblePoint::OBSERVED;
        break;
      case PointStatus::kCollected:
        point_msg.status = open_apero_msgs::CollectiblePoint::COLLECTED;
        break;
    }

    point_array_msg.points.push_back(point_msg);
  }

  collectible_point_pub_.publish(point_array_msg);
}

void PointsEvaluator::loadPointsFromFile(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    agent_.printError("[PointsEvaluator] Error opening file: " + filename);
    return;
  }

  points_.clear();
  std::string line;
  int id = 0;

  // Rows are x,y,z,radius,gain; extra columns are ignored, shorter rows skipped. Points outside the
  // bounding volume are dropped, so ids number only the points kept.
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::istringstream line_stream(line);
    std::string cell;
    std::vector<std::string> row;
    while (std::getline(line_stream, cell, ',')) {
      row.push_back(cell);
    }

    if (row.size() >= 5) {
      try {
        double x = std::stod(row[0]);
        double y = std::stod(row[1]);
        double z = std::stod(row[2]);
        double radius = std::stod(row[3]);
        int gain = std::stoi(row[4]);

        if (bounding_volume_->contains(Eigen::Vector3d(x, y, z))) {
          points_.push_back(std::make_shared<PointSphere>(gain, x, y, z, radius, PointStatus::kUnseen, id));
          ++id;
        }
      } catch (const std::invalid_argument&) {
        agent_.printWarning("[PointsEvaluator] Invalid number format in line: " + line);
      } catch (const std::out_of_range&) {
        agent_.printWarning("[PointsEvaluator] Number out of range in line: " + line);
      }
    }
  }

  file.close();

  agent_.printInfo("[PointsEvaluator] Loaded " + std::to_string(points_.size()) + " points from " + filename);
}

}  // namespace evaluator
}  // namespace open_apero
