#include "eos_slam/sensor_processors/depth_processor.hpp"

namespace eos_slam {

DepthProcessor::DepthProcessor(rclcpp::Node* node)
    : has_intrinsics_(false) {
  node->declare_parameter("depth.min_depth", 0.1);
  node->declare_parameter("depth.max_depth", 10.0);

  min_depth_ = node->get_parameter("depth.min_depth").as_double();
  max_depth_ = node->get_parameter("depth.max_depth").as_double();
}

void DepthProcessor::setCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
  intrinsics_ = ros::CameraIntrinsics::fromMsg(*info);
  has_intrinsics_ = true;
}

DepthResult DepthProcessor::process(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
  DepthResult result;
  result.header = depth_msg->header;

  if (!has_intrinsics_) {
    setCameraInfo(info_msg);
  }

  cv::Mat depth_32f = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
  result.colormap_image = applyColormap(depth_32f);

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->header.frame_id = depth_msg->header.frame_id;

  for (int v = 0; v < depth_32f.rows; ++v) {
    for (int u = 0; u < depth_32f.cols; ++u) {
      float d = depth_32f.at<float>(v, u);
      if (std::isfinite(d) && d >= min_depth_ && d <= max_depth_) {
        Eigen::Vector3d pt = depthTo3D(u, v, d);
        pcl::PointXYZ pcl_pt;
        pcl_pt.x = pt.x(); pcl_pt.y = pt.y(); pcl_pt.z = pt.z();
        cloud->push_back(pcl_pt);
        result.points_3d_list.push_back(pt);
      }
    }
  }

  cloud->width  = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;

  result.points_3d = pcl::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud, *result.points_3d);
  result.points_3d->header = depth_msg->header;

  return result;
}

DepthResult DepthProcessor::processWithKeypoints(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    const std::vector<cv::KeyPoint>& keypoints) {
  DepthResult result;
  result.header = depth_msg->header;

  if (!has_intrinsics_) {
    setCameraInfo(info_msg);
  }

  cv::Mat depth_32f = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
  result.colormap_image = applyColormap(depth_32f);

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->header.frame_id = depth_msg->header.frame_id;

  for (const auto& kp : keypoints) {
    int u = cvRound(kp.pt.x);
    int v = cvRound(kp.pt.y);
    if (u < 0 || u >= depth_32f.cols || v < 0 || v >= depth_32f.rows) continue;

    float d = depth_32f.at<float>(v, u);
    if (std::isfinite(d) && d >= min_depth_ && d <= max_depth_) {
      Eigen::Vector3d pt = depthTo3D(u, v, d);
      pcl::PointXYZ pcl_pt;
      pcl_pt.x = pt.x(); pcl_pt.y = pt.y(); pcl_pt.z = pt.z();
      cloud->push_back(pcl_pt);
      result.points_3d_list.push_back(pt);
    }
  }

  cloud->width  = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;

  result.points_3d = pcl::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud, *result.points_3d);
  result.points_3d->header = depth_msg->header;

  return result;
}

cv::Mat DepthProcessor::applyColormap(const cv::Mat& depth_raw) {
  cv::Mat depth_vis;
  cv::normalize(depth_raw, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::Mat colored;
  cv::applyColorMap(depth_vis, colored, cv::COLORMAP_VIRIDIS);
  return colored;
}

Eigen::Vector3d DepthProcessor::depthTo3D(double u, double v, double depth) {
  return math::projectDepthTo3D(u, v, depth,
    intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy);
}

}  // namespace eos_slam
