#include "eos_slam/sensor_processors/rgb_processor.hpp"

namespace eos_slam {

RGBProcessor::RGBProcessor(rclcpp::Node* node) {
  node->declare_parameter("rgb.max_features", 500);
  node->declare_parameter("rgb.quality_level", 1.0);
  node->declare_parameter("rgb.min_distance", 7.0);

  max_features_   = node->get_parameter("rgb.max_features").as_int();
  quality_level_  = node->get_parameter("rgb.quality_level").as_double();
  min_distance_   = node->get_parameter("rgb.min_distance").as_double();

  orb_ = cv::ORB::create(max_features_, 1.2f, 8, 31, 0, 2,
                         cv::ORB::HARRIS_SCORE, 31, quality_level_);
}

RGBResult RGBProcessor::process(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  RGBResult result;
  result.header = msg->header;

  cv::Mat color = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::cvtColor(color, result.raw_gray, cv::COLOR_BGR2GRAY);

  orb_->detectAndCompute(result.raw_gray, cv::noArray(),
                         result.keypoints, result.descriptors);

  result.debug_image = drawKeypointsDebug(color, result.keypoints);

  return result;
}

cv::Mat RGBProcessor::drawKeypointsDebug(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& kps) {
  cv::Mat out;
  cv::drawKeypoints(img, kps, out, cv::Scalar(0, 255, 0),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  return out;
}

}  // namespace eos_slam
