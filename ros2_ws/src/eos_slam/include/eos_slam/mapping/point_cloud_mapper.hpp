#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mutex>
#include <string>

namespace eos_slam {

class PointCloudMapper {
public:
  explicit PointCloudMapper(rclcpp::Node* node);

  // Transform cloud from its sensor frame into the map frame via TF2,
  // then accumulate it into the global map.
  void addCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

  sensor_msgs::msg::PointCloud2::SharedPtr getMapCloud(const std::string& frame_id = "map");
  nav_msgs::msg::OccupancyGrid::SharedPtr  getOccupancyGrid();

  bool hasNewData() const { return new_data_; }
  void clearNewData()     { new_data_ = false; }

private:
  rclcpp::Node* node_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double map_voxel_size_;
  double grid_resolution_;
  int    grid_width_;
  int    grid_height_;
  int    publish_every_n_;
  double min_obstacle_z_;   // m — ignore points below this (floor)
  double max_obstacle_z_;   // m — ignore points above this (ceiling)

  int  clouds_added_{0};
  bool new_data_{false};

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  mutable std::mutex map_mutex_;

  void downsampleMap();
};

}  // namespace eos_slam
