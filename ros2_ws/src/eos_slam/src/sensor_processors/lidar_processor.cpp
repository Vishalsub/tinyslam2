#include "eos_slam/sensor_processors/lidar_processor.hpp"

namespace eos_slam {

LidarProcessor::LidarProcessor(rclcpp::Node* node) {
  node->declare_parameter("lidar.voxel_size", 0.1);
  node->declare_parameter("lidar.min_range", 0.5);
  node->declare_parameter("lidar.max_range", 30.0);

  voxel_size_ = node->get_parameter("lidar.voxel_size").as_double();
  min_range_  = node->get_parameter("lidar.min_range").as_double();
  max_range_  = node->get_parameter("lidar.max_range").as_double();
}

sensor_msgs::msg::PointCloud2::SharedPtr LidarProcessor::process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);

  pcl::PassThrough<pcl::PointXYZ> pt_filter;
  pt_filter.setInputCloud(cloud);
  pt_filter.setFilterFieldName("z");
  pt_filter.setFilterLimits(min_range_, max_range_);
  auto cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pt_filter.filter(*cloud_filtered);

  pt_filter.setFilterFieldName("x");
  pt_filter.setInputCloud(cloud_filtered);
  auto cloud_x = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pt_filter.filter(*cloud_x);

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_x);
  vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  auto cloud_down = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  vg.filter(*cloud_down);

  auto out = pcl::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud_down, *out);
  out->header = msg->header;
  return out;
}

}  // namespace eos_slam
