#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace eos_slam {

struct RGBResult {
  cv::Mat raw_gray;
  cv::Mat debug_image;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std_msgs::msg::Header header;
};

class RGBProcessor {
public:
  explicit RGBProcessor(rclcpp::Node* node);

  RGBResult process(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  int max_features_;
  double quality_level_;
  double min_distance_;
  cv::Ptr<cv::ORB> orb_;

  cv::Mat drawKeypointsDebug(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& kps);
};

}  // namespace eos_slam
