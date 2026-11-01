#pragma once

/**
 * @file visualization_markers.h
 * @brief ROS-independent visualization marker data structures.
 *
 * Provides Color, VisualizationMarker, and VisualizationMarkers types that
 * mirror ROS visualization_msgs but can be used without ROS dependencies.
 * The ROS layer converts these to actual ROS marker messages for RViz display.
 */

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace open_apero {

/**
 * @brief RGBA color representation with named preset colors.
 */
struct Color {
  double r = 0;
  double g = 0;
  double b = 0;
  double a = 1;

  Color() = default;
  Color(double red, double green, double blue, double alpha) : r(red), g(green), b(blue), a(alpha) {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}

  static const Color white() { return Color(1.0, 1.0, 1.0); }
  static const Color black() { return Color(0.0, 0.0, 0.0); }
  static const Color gray() { return Color(0.5, 0.5, 0.5); }
  static const Color red() { return Color(1.0, 0.0, 0.0); }
  static const Color green() { return Color(0.0, 1.0, 0.0); }
  static const Color blue() { return Color(0.0, 0.0, 1.0); }
  static const Color yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color orange() { return Color(1.0, 0.5, 0.0); }
  static const Color purple() { return Color(0.5, 0.0, 1.0); }
  static const Color chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color teal() { return Color(0.0, 1.0, 1.0); }
  static const Color pink() { return Color(1.0, 0.0, 0.5); }
};

/**
 * @brief A single visualization marker with shape, pose, color, and optional mesh.
 *
 * Type and action constants mirror ROS visualization_msgs/Marker.
 */
struct VisualizationMarker {
  // Type and action constants from ROS visualization_msgs/Marker
  const static unsigned char ARROW = 0;
  const static unsigned char CUBE = 1;
  const static unsigned char SPHERE = 2;
  const static unsigned char CYLINDER = 3;
  const static unsigned char LINE_STRIP = 4;
  const static unsigned char LINE_LIST = 5;
  const static unsigned char CUBE_LIST = 6;
  const static unsigned char SPHERE_LIST = 7;
  const static unsigned char POINTS = 8;
  const static unsigned char TEXT_VIEW_FACING = 9;
  const static unsigned char MESH_RESOURCE = 10;
  const static unsigned char TRIANGLE_LIST = 11;
  const static unsigned char ADD = 0;
  const static unsigned char MODIFY = 0;
  const static unsigned char DELETE = 2;
  const static unsigned char DELETEALL = 3;

  int type = 0;
  int action = 0;
  int id = 0;
  std::string ns = "";
  std::string text = "";
  Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
  Eigen::Quaterniond orientation = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d scale = Eigen::Vector3d(0, 0, 0);
  Color color = Color();
  std::vector<Eigen::Vector3d> points;  // empty initialization
  std::vector<Color> colors;

  // mesh
  std::string mesh_resource = "";
  bool mesh_use_embedded_materials = false;
};

/**
 * @brief Container for a collection of visualization markers with auto-incrementing IDs.
 */
struct VisualizationMarkers {
  /**
   * @brief Appends a single marker with an auto-assigned ID.
   * @param marker The marker to add.
   */
  void addMarker(const VisualizationMarker& marker);

  /**
   * @brief Appends multiple markers, each with an auto-assigned ID.
   * @param markers The markers to add.
   */
  void addMarkers(const std::vector<VisualizationMarker>& markers);

  /**
   * @brief Returns the id after the last stored marker's (0 when empty); nothing is incremented.
   * @return Next marker ID.
   */
  int getNextVisualizationId();

  /**
   * @brief Returns a const reference to the list of stored markers.
   * @return Const reference to the markers vector.
   */
  const std::vector<VisualizationMarker>& getMarkers() const;

  /**
   * @brief Returns a mutable reference to the list of stored markers.
   * @return Reference to the markers vector.
   */
  std::vector<VisualizationMarker>& getMarkers();

 protected:
  std::vector<VisualizationMarker> markers_;
};

}  // namespace open_apero
