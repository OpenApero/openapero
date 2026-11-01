#include <string>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <open_apero_core/visualization/visualization_markers.h>
#include <open_apero_ros_bridge/marker_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace open_apero {
namespace ros {
namespace {

VisualizationMarker marker(const std::string& ns, int type, int action) {
  VisualizationMarker result;
  result.ns = ns;
  result.type = type;
  result.action = action;
  return result;
}

TEST(MarkerConversionTest, LeavesOutOnlyAnAddOfAPointListWithoutPoints) {
  VisualizationMarker filled = marker("filled", visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::ADD);
  filled.points.push_back(Eigen::Vector3d(1.0, 2.0, 3.0));

  VisualizationMarkers markers;
  markers.addMarker(marker("empty", visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::ADD));
  markers.addMarker(marker("deleted", visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::DELETE));
  markers.addMarker(marker("all", visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::DELETEALL));
  markers.addMarker(filled);
  markers.addMarker(marker("sphere", visualization_msgs::Marker::SPHERE, visualization_msgs::Marker::ADD));

  visualization_msgs::MarkerArray msg;
  visualizationMarkersToMsg(markers, &msg);

  ASSERT_EQ(msg.markers.size(), 4u);
  EXPECT_EQ(msg.markers[0].ns, "deleted");
  EXPECT_EQ(msg.markers[0].action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(msg.markers[1].ns, "all");
  EXPECT_EQ(msg.markers[1].action, visualization_msgs::Marker::DELETEALL);
  EXPECT_EQ(msg.markers[2].ns, "filled");
  ASSERT_EQ(msg.markers[2].points.size(), 1u);
  EXPECT_EQ(msg.markers[2].points[0].y, 2.0);
  EXPECT_EQ(msg.markers[3].ns, "sphere");
}

TEST(MarkerConversionTest, EveryPointListTypeWithoutPointsIsLeftOut) {
  for (const int type :
       {visualization_msgs::Marker::POINTS, visualization_msgs::Marker::LINE_STRIP, visualization_msgs::Marker::LINE_LIST,
        visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::SPHERE_LIST, visualization_msgs::Marker::TRIANGLE_LIST}) {
    SCOPED_TRACE(type);
    VisualizationMarkers markers;
    markers.addMarker(marker("empty", type, visualization_msgs::Marker::ADD));

    visualization_msgs::MarkerArray msg;
    visualizationMarkersToMsg(markers, &msg);
    EXPECT_TRUE(msg.markers.empty());
  }
}

}  // namespace
}  // namespace ros
}  // namespace open_apero
