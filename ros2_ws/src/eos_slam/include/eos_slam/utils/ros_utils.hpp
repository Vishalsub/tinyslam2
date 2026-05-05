#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

namespace eos_slam::ros {

struct CameraIntrinsics {
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  int width = 0, height = 0;

  static CameraIntrinsics fromMsg(const sensor_msgs::msg::CameraInfo& msg) {
    CameraIntrinsics ci;
    ci.fx = msg.k[0];
    ci.fy = msg.k[4];
    ci.cx = msg.k[2];
    ci.cy = msg.k[5];
    ci.width  = msg.width;
    ci.height = msg.height;
    return ci;
  }
};

inline cv::Mat imageMsgToCv(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const std::string& encoding = "mono8") {
  return cv_bridge::toCvShare(msg, encoding)->image;
}

inline sensor_msgs::msg::Image::SharedPtr cvToImageMsg(
    const cv::Mat& img, const std::string& encoding,
    const std_msgs::msg::Header& header) {
  return cv_bridge::CvImage(header, encoding, img).toImageMsg();
}

inline rclcpp::QoS sensor_data_qos() {
  return rclcpp::SensorDataQoS();
}

inline rclcpp::QoS reliable_qos(int depth = 10) {
  rclcpp::QoS qos(depth);
  qos.reliable();
  return qos;
}

}  // namespace eos_slam::ros
