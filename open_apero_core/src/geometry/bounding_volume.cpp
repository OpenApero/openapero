#include "open_apero_core/geometry/bounding_volume.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "open_apero_core/common/utils.h"

namespace open_apero {

namespace {
// Rotate dataset coordinates into the agent frame before measuring the AABB.
void computeBoundingBox(const aiScene* scene, Eigen::Vector3d& min, Eigen::Vector3d& max, const Eigen::Quaterniond& rotation,
                        double margin = 0.0) {
  min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

  if (!scene) {
    throw std::runtime_error("Invalid Assimp scene pointer.");
  }

  for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh* mesh = scene->mMeshes[m];

    for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
      const aiVector3D& vertex = mesh->mVertices[v];
      Eigen::Vector3d point(vertex.x, vertex.y, vertex.z);
      point = rotation * point;
      min = min.cwiseMin(point);
      max = max.cwiseMax(point);
    }
  }

  Eigen::Vector3d margin_3d(margin, margin, margin);
  min -= margin_3d;
  max += margin_3d;
}
}  // namespace

// Mesh-derived bounds require dataset-specific frame rotations.
void BoundingVolume::setupFromParamMap(Configurable::ParamMap* param_map) {
  std::string mesh_folder = "";
  setParam<std::string>(param_map, "mesh_folder", &mesh_folder, "");

  std::string scene_id = "";
  setParam<std::string>(param_map, "scene_id", &scene_id, "");

  double margin = 0.0;
  setParam<double>(param_map, "margin", &margin, 0.0);

  if (mesh_folder != "" && scene_id != "") {
    std::string mesh_type;
    std::string mesh_file;

    size_t dashPos = scene_id.find('-');
    if (dashPos != std::string::npos) {
      mesh_type = "HM3D";

      std::string scene_code = scene_id.substr(dashPos + 1);
      mesh_file = mesh_folder + "/" + scene_id + "/" + scene_code + ".glb";
    } else {
      mesh_type = "HSSD";

      mesh_file = mesh_folder + "/" + scene_id + ".glb";
    }

    if (info_logger_) {
      info_logger_("[BoundingVolume] Mesh file: " + mesh_file);
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(mesh_file, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
      throw std::runtime_error(std::string("Assimp error: ") + importer.GetErrorString());
    }

    // HM3D and HSSD encode mesh axes differently.
    Eigen::Quaterniond C_Ros_HM3D = quaternionFromYaw(-M_PI / 2.0);
    Eigen::Quaterniond C_Ros_HSSD =
        Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d min, max;
    if (mesh_type == "HM3D") {
      computeBoundingBox(scene, min, max, C_Ros_HM3D, margin);
    } else {
      computeBoundingBox(scene, min, max, C_Ros_HSSD, margin);
    }

    x_min = min.x();
    x_max = max.x();
    y_min = min.y();
    y_max = max.y();
    z_min = min.z();
    z_max = max.z();

    std::ostringstream ss;
    ss << "[BoundingVolume] Bounding Box:\n"
       << "x: [" << x_min << ", " << x_max << "]\n"
       << "y: [" << y_min << ", " << y_max << "]\n"
       << "z: [" << z_min << ", " << z_max << "]";
    if (info_logger_) {
      info_logger_(ss.str());
    }
  } else {
    setParam<double>(param_map, "x_min", &x_min, 0.0);
    setParam<double>(param_map, "x_max", &x_max, 0.0);
    setParam<double>(param_map, "y_min", &y_min, 0.0);
    setParam<double>(param_map, "y_max", &y_max, 0.0);
    setParam<double>(param_map, "z_min", &z_min, 0.0);
    setParam<double>(param_map, "z_max", &z_max, 0.0);
  }

  setParam<double>(param_map, "rotation", &rotation, 0.0);
  rotation_quat = Eigen::AngleAxis<double>(rotation * M_PI / -180.0, Eigen::Vector3d::UnitZ());

  // Check params span a valid volume (This is not a compulsory check to allow
  // the unspecified (default) case, where the volume is not checked)
  if (x_max > x_min && y_max > y_min && z_max > z_min) {
    is_setup = true;
  }
}

// Uninitialized volumes accept every point.
bool BoundingVolume::contains(const Eigen::Vector3d& point) {
  if (!is_setup) {
    return true;
  }  // Uninitialized boxes always return true, so can check them by default
  Eigen::Vector3d rotated_point = rotation_quat * point;
  if (rotated_point.x() < x_min) {
    return false;
  }
  if (rotated_point.x() > x_max) {
    return false;
  }
  if (rotated_point.y() < y_min) {
    return false;
  }
  if (rotated_point.y() > y_max) {
    return false;
  }
  if (rotated_point.z() < z_min) {
    return false;
  }
  if (rotated_point.z() > z_max) {
    return false;
  }
  return true;
}

void BoundingVolume::visualize(VisualizationMarkers* markers) {
  VisualizationMarker marker;
  marker.ns = "bounding_volume";
  marker.type = VisualizationMarker::LINE_LIST;
  marker.scale.x() = 0.06;
  // Orange stays legible against both RViz's dark background and light map surfaces.
  marker.color = Color::orange();
  marker.color.a = 1.0;

  // contains() rotates world points into the volume frame; visualization needs
  // the inverse transform to put the configured corners back in the world.
  const Eigen::Quaterniond volume_to_world = rotation_quat.inverse();
  Eigen::Vector3d p1 = volume_to_world * Eigen::Vector3d(x_min, y_min, z_min);
  Eigen::Vector3d p2 = volume_to_world * Eigen::Vector3d(x_max, y_min, z_min);
  Eigen::Vector3d p3 = volume_to_world * Eigen::Vector3d(x_max, y_max, z_min);
  Eigen::Vector3d p4 = volume_to_world * Eigen::Vector3d(x_min, y_max, z_min);
  Eigen::Vector3d p5 = volume_to_world * Eigen::Vector3d(x_min, y_min, z_max);
  Eigen::Vector3d p6 = volume_to_world * Eigen::Vector3d(x_max, y_min, z_max);
  Eigen::Vector3d p7 = volume_to_world * Eigen::Vector3d(x_max, y_max, z_max);
  Eigen::Vector3d p8 = volume_to_world * Eigen::Vector3d(x_min, y_max, z_max);

  // Add lines to form the bounding box
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p4);
  marker.points.push_back(p1);
  marker.points.push_back(p5);
  marker.points.push_back(p6);
  marker.points.push_back(p6);
  marker.points.push_back(p7);
  marker.points.push_back(p7);
  marker.points.push_back(p8);
  marker.points.push_back(p8);
  marker.points.push_back(p5);
  marker.points.push_back(p1);
  marker.points.push_back(p5);
  marker.points.push_back(p2);
  marker.points.push_back(p6);
  marker.points.push_back(p3);
  marker.points.push_back(p7);
  marker.points.push_back(p4);
  marker.points.push_back(p8);

  // Add the marker to the visualization
  markers->addMarker(marker);
}

}  // namespace open_apero
