#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <Eigen/Dense>
#include "../utils/math_utils.hpp"
#include "../utils/ros_utils.hpp"

namespace eos_slam {

struct DepthResult {
  cv::Mat colormap_image;
  sensor_msgs::msg::PointCloud2::SharedPtr points_3d;
  std::vector<Eigen::Vector3d> points_3d_list;
  std_msgs::msg::Header header;
};

class DepthProcessor {
public:
  explicit DepthProcessor(rclcpp::Node* node);

  DepthResult process(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

  DepthResult processWithKeypoints(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    const std::vector<cv::KeyPoint>& keypoints);

  void setCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

private:
  ros::CameraIntrinsics intrinsics_;
  double min_depth_;
  double max_depth_;
  bool has_intrinsics_;

  cv::Mat applyColormap(const cv::Mat& depth_raw);
  Eigen::Vector3d depthTo3D(double u, double v, double depth);
};

}  // namespace eos_slam
