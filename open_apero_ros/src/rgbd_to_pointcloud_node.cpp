#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

inline bool calculatePointFromIntrinsics(Eigen::Vector3d& p, const Eigen::Vector4d& fx_fy_cx_cy, int i, int j, float depth,
                                         double min_range = -1.0, double max_range = -1.0) {
  // Invalid depth: below 1e-4 (zero or negative) or NaN.
  if (depth < 1e-4 || std::isnan(depth)) {
    return false;
  }

  if (min_range > 1e-4 && depth < min_range) {
    return false;
  }

  if (max_range > 1e-4 && depth > max_range) {
    return false;
  }

  p.z() = depth;
  p.x() = (j - fx_fy_cx_cy(2)) * p.z() / fx_fy_cx_cy(0);
  p.y() = (i - fx_fy_cx_cy(3)) * p.z() / fx_fy_cx_cy(1);
  return true;
}

class RgbdToPointCloudConverter {
 public:
  using CameraInfoMsg = sensor_msgs::CameraInfo;
  using ImageMsg = sensor_msgs::Image;
  using RgbdSync = message_filters::TimeSynchronizer<CameraInfoMsg, ImageMsg, ImageMsg>;

  RgbdToPointCloudConverter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private) {
    nh_private_.param("ray_min_range", p_ray_min_range_, 0.0);
    nh_private_.param("ray_max_range", p_ray_max_range_, 0.0);
    nh_private_.param("decimation_factor", p_decimation_factor_, 1);
    if (p_decimation_factor_ < 1) {
      ROS_FATAL("~decimation_factor must be at least 1, got %d.", p_decimation_factor_);
      ros::shutdown();
      return;
    }

    camera_info_sub_.subscribe(nh_, "camera_info", 5);
    rgb_image_sub_.subscribe(nh_, "rgb_image", 5);
    depth_image_sub_.subscribe(nh_, "depth_image", 5);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 5, false);

    rgbd_sync_ = std::make_shared<RgbdSync>(camera_info_sub_, rgb_image_sub_, depth_image_sub_, 5);
    rgbd_sync_->registerCallback(boost::bind(&RgbdToPointCloudConverter::imageToPointCloudCallback, this, _1, _2, _3));
  }

 private:
  void imageToPointCloudCallback(const CameraInfoMsg::ConstPtr& camera_info, const ImageMsg::ConstPtr& rgb_image,
                                 const ImageMsg::ConstPtr& depth_image) {
    if (pointcloud_pub_.getNumSubscribers() == 0) return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!getPointCloudFromRgbd(camera_info, rgb_image, depth_image, cloud, p_decimation_factor_)) return;

    if (!cloud->empty()) {
      sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*cloud, *cloud_msg);
      cloud_msg->header = depth_image->header;
      pointcloud_pub_.publish(*cloud_msg);
    } else {
      sensor_msgs::PointCloud2 empty_cloud_msg;
      empty_cloud_msg.header = depth_image->header;
      empty_cloud_msg.height = 1;
      empty_cloud_msg.width = 0;
      pointcloud_pub_.publish(empty_cloud_msg);
      ROS_WARN("Empty point cloud generated from RGB-D.");
    }
  }

  bool getPointCloudFromRgbd(const CameraInfoMsg::ConstPtr& camera_info, const ImageMsg::ConstPtr& rgb_image,
                             const ImageMsg::ConstPtr& depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pointcloud,
                             int decimation_factor) {
    const Eigen::Vector4d fx_fy_cx_cy(camera_info->K[0], camera_info->K[4], camera_info->K[2], camera_info->K[5]);

    cv_bridge::CvImagePtr rgb_cv_ptr;
    try {
      // Converts any color encoding cv_bridge supports (e.g. bgra8) to rgb8; others throw.
      rgb_cv_ptr = cv_bridge::toCvCopy(rgb_image, "rgb8");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

    cv_bridge::CvImagePtr depth_cv_ptr;
    try {
      depth_cv_ptr = cv_bridge::toCvCopy(depth_image, depth_image->encoding);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

    // RGB must be 8-bit with 3 channels, depth 32-bit float with 1 channel.
    if (rgb_cv_ptr->image.depth() != CV_8U) {
      ROS_ERROR("RGB image must be of type CV_8U");
      return false;
    }
    if (depth_cv_ptr->image.depth() != CV_32F) {
      ROS_ERROR("Depth image must be of type CV_32F");
      return false;
    }

    if (rgb_cv_ptr->image.channels() != 3) {
      ROS_ERROR("RGB image must have 3 channels");
      return false;
    }
    if (depth_cv_ptr->image.channels() != 1) {
      ROS_ERROR("Depth image must have 1 channel");
      return false;
    }

    const auto& rgb_mat = rgb_cv_ptr->image;
    const auto& depth_mat = depth_cv_ptr->image;
    if (rgb_mat.size() != depth_mat.size()) {
      ROS_ERROR_THROTTLE(5.0, "RGB image (%dx%d) and depth image (%dx%d) must have the same size", rgb_mat.cols, rgb_mat.rows,
                         depth_mat.cols, depth_mat.rows);
      return false;
    }

    for (int i = 0; i < rgb_mat.rows; i += decimation_factor) {
      const float* d_ptr = depth_mat.ptr<float>(i);  // First pixel of row i.

      for (int j = 0; j < rgb_mat.cols; j += decimation_factor) {
        auto rgb = rgb_mat.at<cv::Vec3b>(i, j);

        const float depth = d_ptr[j];

        Eigen::Vector3d point_sensor_frame;
        if (calculatePointFromIntrinsics(point_sensor_frame, fx_fy_cx_cy, i, j, depth, p_ray_min_range_, p_ray_max_range_)) {
          pcl::PointXYZRGB point;
          point.x = point_sensor_frame.x();
          point.y = point_sensor_frame.y();
          point.z = point_sensor_frame.z();
          point.r = rgb[0];
          point.g = rgb[1];
          point.b = rgb[2];
          rgb_pointcloud->push_back(point);
        }
      }
    }

    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pointcloud_pub_;

  message_filters::Subscriber<CameraInfoMsg> camera_info_sub_;
  message_filters::Subscriber<ImageMsg> rgb_image_sub_;
  message_filters::Subscriber<ImageMsg> depth_image_sub_;
  std::shared_ptr<RgbdSync> rgbd_sync_;

  double p_ray_min_range_;
  double p_ray_max_range_;
  int p_decimation_factor_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rgbd_to_pointcloud_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  RgbdToPointCloudConverter node(nh, nh_private);

  ros::spin();
  return 0;
}
