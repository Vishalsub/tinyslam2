#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace eos_slam {

class LidarProcessor {
public:
  explicit LidarProcessor(rclcpp::Node* node);

  sensor_msgs::msg::PointCloud2::SharedPtr process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

private:
  double voxel_size_;
  double min_range_;
  double max_range_;
};

}  // namespace eos_slam
