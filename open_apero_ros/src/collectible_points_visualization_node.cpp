#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <open_apero_msgs/CollectiblePointArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class CollectiblePointsVisualizer {
 public:
  // Topics go on the public handle under relative names so a launch file can
  // remap them; parameters go on the private handle, where ~name lives.
  CollectiblePointsVisualizer() : nh_private_("~"), runner_frame_(0) {
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("collectible_point_markers", 1);
    ring_sub_ = nh_.subscribe("collectible_points", 1, &CollectiblePointsVisualizer::ringCallback, this);

    nh_private_.param<std::string>("map_frame", p_map_frame_, "map");
    nh_private_.param<std::string>("camera_frame", p_camera_frame_, "camera_frame");
    nh_private_.param("rotate_speed", p_rotate_speed_, 0.1);
    nh_private_.param("publish_rate", p_publish_rate_, 30.0);
    nh_private_.param("shrink_duration", p_shrink_duration_, 1.0);

    // meshes/ ships the even frames runner_0 ... runner_32 only.
    nh_private_.param("runner_frame_step", p_runner_frame_step_, 2);
    nh_private_.param("runner_frame_max", p_runner_frame_max_, 32);

    std::srand(std::time(nullptr));
  }

  void spin() {
    ros::Rate rate(p_publish_rate_);
    while (ros::ok()) {
      visualization_msgs::MarkerArray marker_array;
      ros::Time now = ros::Time::now();

      getRotatingRingMarkers(marker_array, now);
      getRunnerMarker(marker_array, now);

      if (!marker_array.markers.empty()) {
        marker_pub_.publish(marker_array);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher marker_pub_;
  ros::Subscriber ring_sub_;

  std::unordered_map<int, double> ring_rotations_;
  std::unordered_map<int, ros::Time> collected_rings_;
  std::unordered_set<int> deleted_rings_;  // Collected rings whose DELETE was already sent.
  open_apero_msgs::CollectiblePointArray latest_rings_;

  std::string p_map_frame_;     // Frame of the ring positions.
  std::string p_camera_frame_;  // Frame the running figure follows.
  double p_rotate_speed_;
  double p_publish_rate_;
  double p_shrink_duration_;
  int runner_frame_;
  int p_runner_frame_step_;
  int p_runner_frame_max_;

  void ringCallback(const open_apero_msgs::CollectiblePointArray::ConstPtr& msg) {
    for (const auto& ring : msg->points) {
      if (ring.status == open_apero_msgs::CollectiblePoint::OBSERVED) {
        if (ring_rotations_.find(ring.id) == ring_rotations_.end()) {
          ring_rotations_[ring.id] = static_cast<double>(std::rand()) / RAND_MAX * 2 * M_PI;
        }
      } else if (ring.status == open_apero_msgs::CollectiblePoint::COLLECTED) {
        if (collected_rings_.find(ring.id) == collected_rings_.end()) {
          collected_rings_[ring.id] = ros::Time::now();
        }
      }
    }
    latest_rings_ = *msg;
  }

  void getRotatingRingMarkers(visualization_msgs::MarkerArray& marker_array, const ros::Time& now) {
    if (latest_rings_.points.empty()) {
      return;
    }

    for (const auto& ring : latest_rings_.points) {
      if (ring.status == open_apero_msgs::CollectiblePoint::UNSEEN) {
        continue;
      }

      visualization_msgs::Marker marker;
      marker.header.frame_id = p_map_frame_;
      marker.header.stamp = now;
      marker.ns = "rotating_rings";
      marker.id = ring.id;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://open_apero_ros/meshes/collectible_ring.dae";
      marker.mesh_use_embedded_materials = true;

      marker.pose.position.x = ring.x;
      marker.pose.position.y = ring.y;
      marker.pose.position.z = ring.z;

      ring_rotations_[ring.id] = std::fmod(ring_rotations_[ring.id] + p_rotate_speed_ / p_publish_rate_, 2 * M_PI);
      Eigen::Quaterniond rotation(Eigen::AngleAxisd(ring_rotations_[ring.id], Eigen::Vector3d::UnitZ()));
      marker.pose.orientation.x = rotation.x();
      marker.pose.orientation.y = rotation.y();
      marker.pose.orientation.z = rotation.z();
      marker.pose.orientation.w = rotation.w();

      double scale_factor = ring.gain * 0.0025 + 0.025;

      if (ring.status == open_apero_msgs::CollectiblePoint::COLLECTED) {
        if (collected_rings_.find(ring.id) != collected_rings_.end()) {
          ros::Duration elapsed = now - collected_rings_[ring.id];
          double shrink_ratio = std::max(0.0, 1.0 - elapsed.toSec() / p_shrink_duration_);
          if (shrink_ratio <= 0.05) {
            if (!deleted_rings_.insert(ring.id).second) {
              continue;
            }
            visualization_msgs::Marker delete_marker;
            delete_marker.header = marker.header;
            delete_marker.ns = marker.ns;
            delete_marker.id = marker.id;
            delete_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(delete_marker);
            continue;
          }
          scale_factor *= shrink_ratio;
        }
      }

      marker.scale.x = scale_factor;
      marker.scale.y = scale_factor;
      marker.scale.z = scale_factor;
      marker_array.markers.push_back(marker);
    }
  }

  void getRunnerMarker(visualization_msgs::MarkerArray& marker_array, const ros::Time& now) {
    visualization_msgs::Marker runner_marker;
    runner_marker.header.frame_id = p_camera_frame_;
    runner_marker.header.stamp = now;
    runner_marker.ns = "runner";
    runner_marker.id = 0;
    runner_marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    runner_marker.mesh_resource = "package://open_apero_ros/meshes/runner_" + std::to_string(runner_frame_) + ".dae";
    runner_marker.mesh_use_embedded_materials = true;

    runner_marker.pose.position.x = 0.0;
    runner_marker.pose.position.y = 0.0;
    runner_marker.pose.position.z = -0.3;
    runner_marker.pose.orientation.w = 1.0;

    runner_marker.scale.x = 100.0;
    runner_marker.scale.y = 100.0;
    runner_marker.scale.z = 100.0;

    marker_array.markers.push_back(runner_marker);

    runner_frame_ += p_runner_frame_step_;
    if (runner_frame_ > p_runner_frame_max_) {
      runner_frame_ = 0;
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "collectible_points_visualization_node");
  CollectiblePointsVisualizer visualizer;
  visualizer.spin();
  return 0;
}
