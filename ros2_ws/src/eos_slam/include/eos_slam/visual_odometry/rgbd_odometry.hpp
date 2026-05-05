#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../utils/math_utils.hpp"
#include "../utils/ros_utils.hpp"

namespace eos_slam {

struct VOResult {
  Eigen::Isometry3d current_pose;
  nav_msgs::msg::Path path;
  cv::Mat debug_matches;
  bool valid;
};

class RGBDOdometry {
public:
  explicit RGBDOdometry(rclcpp::Node* node);

  VOResult process(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

private:
  ros::CameraIntrinsics intrinsics_;
  int max_features_;
  double min_depth_;
  double max_depth_;
  int keyframe_interval_;

  cv::Ptr<cv::ORB> orb_;
  cv::BFMatcher matcher_;

  cv::Mat prev_gray_;
  std::vector<cv::KeyPoint> prev_kps_;
  cv::Mat prev_descs_;
  Eigen::Isometry3d accumulated_pose_;
  nav_msgs::msg::Path path_;
  int frame_count_;
  bool initialized_;

  VOResult computePose(
    const cv::Mat& gray, const cv::Mat& depth,
    const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs);

  std::vector<cv::DMatch> matchFeatures(
    const cv::Mat& desc1, const cv::Mat& desc2);

  std::vector<cv::Point2f> getCorrespondencesWithDepth(
    const std::vector<cv::KeyPoint>& kps1,
    const std::vector<cv::KeyPoint>& kps2,
    const std::vector<cv::DMatch>& matches,
    const cv::Mat& depth,
    std::vector<cv::Point3f>& points_3d);

  cv::Mat drawMatchesDebug(
    const cv::Mat& img1, const std::vector<cv::KeyPoint>& kps1,
    const cv::Mat& img2, const std::vector<cv::KeyPoint>& kps2,
    const std::vector<cv::DMatch>& matches);

  Eigen::Vector3d depthLookup(const cv::Mat& depth, double u, double v);
};

}  // namespace eos_slam
