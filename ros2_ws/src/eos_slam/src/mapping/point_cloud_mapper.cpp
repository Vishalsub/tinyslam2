#include "eos_slam/mapping/point_cloud_mapper.hpp"

#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>

namespace eos_slam {

// Convert a ROS TransformStamped to Eigen::Isometry3d.
static Eigen::Isometry3d toIsometry(const geometry_msgs::msg::TransformStamped& tf) {
  const auto& t = tf.transform.translation;
  const auto& r = tf.transform.rotation;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << t.x, t.y, t.z;
  T.rotate(Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized());
  return T;
}

PointCloudMapper::PointCloudMapper(rclcpp::Node* node)
    : node_(node),
      global_map_(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {
  node->declare_parameter("map.voxel_size",      0.05);
  node->declare_parameter("map.grid_resolution", 0.05);
  node->declare_parameter("map.grid_width",      400);
  node->declare_parameter("map.grid_height",     400);
  node->declare_parameter("map.publish_every_n", 5);
  node->declare_parameter("map.min_obstacle_z",  0.10);
  node->declare_parameter("map.max_obstacle_z",  2.00);

  map_voxel_size_   = node->get_parameter("map.voxel_size").as_double();
  grid_resolution_  = node->get_parameter("map.grid_resolution").as_double();
  grid_width_       = node->get_parameter("map.grid_width").as_int();
  grid_height_      = node->get_parameter("map.grid_height").as_int();
  publish_every_n_  = node->get_parameter("map.publish_every_n").as_int();
  min_obstacle_z_   = node->get_parameter("map.min_obstacle_z").as_double();
  max_obstacle_z_   = node->get_parameter("map.max_obstacle_z").as_double();

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PointCloudMapper::addCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
  // Look up the transform from the cloud's sensor frame → map at message time.
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
        "map", cloud_msg->header.frame_id,
        cloud_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.15));
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "TF lookup map←%s failed: %s",
                         cloud_msg->header.frame_id.c_str(), e.what());
    return;
  }

  const Eigen::Isometry3d T_map_sensor = toIsometry(tf_stamped);

  pcl::PointCloud<pcl::PointXYZ> cloud_sensor;
  pcl::fromROSMsg(*cloud_msg, cloud_sensor);

  if (cloud_sensor.empty()) return;

  std::lock_guard<std::mutex> lock(map_mutex_);

  for (const auto& pt : cloud_sensor) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    Eigen::Vector3d p_map = T_map_sensor * Eigen::Vector3d(pt.x, pt.y, pt.z);

    // Height filter applied in the map frame.
    if (p_map.z() < min_obstacle_z_ || p_map.z() > max_obstacle_z_) continue;

    global_map_->emplace_back(
        static_cast<float>(p_map.x()),
        static_cast<float>(p_map.y()),
        static_cast<float>(p_map.z()));
  }

  ++clouds_added_;
  if (clouds_added_ % publish_every_n_ == 0) {
    downsampleMap();
    new_data_ = true;
  }
}

void PointCloudMapper::downsampleMap() {
  if (global_map_->empty()) return;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(global_map_);
  vg.setLeafSize(
      static_cast<float>(map_voxel_size_),
      static_cast<float>(map_voxel_size_),
      static_cast<float>(map_voxel_size_));
  auto ds = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  vg.filter(*ds);
  global_map_ = ds;
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudMapper::getMapCloud(
    const std::string& frame_id) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (global_map_->empty()) return nullptr;
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*global_map_, *msg);
  msg->header.frame_id = frame_id;
  msg->header.stamp    = node_->get_clock()->now();
  return msg;
}

nav_msgs::msg::OccupancyGrid::SharedPtr PointCloudMapper::getOccupancyGrid() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (global_map_->empty()) return nullptr;

  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid->header.stamp    = node_->get_clock()->now();
  grid->header.frame_id = "map";
  grid->info.resolution = static_cast<float>(grid_resolution_);
  grid->info.width      = static_cast<uint32_t>(grid_width_);
  grid->info.height     = static_cast<uint32_t>(grid_height_);
  grid->info.origin.position.x = -(grid_width_  * grid_resolution_) / 2.0;
  grid->info.origin.position.y = -(grid_height_ * grid_resolution_) / 2.0;
  grid->info.origin.orientation.w = 1.0;
  // -1 = unknown (grey in RViz)
  grid->data.assign(static_cast<size_t>(grid_width_ * grid_height_), -1);

  for (const auto& pt : *global_map_) {
    // Points here are already height-filtered; project directly onto XY plane.
    int cx = static_cast<int>((pt.x - grid->info.origin.position.x) / grid_resolution_);
    int cy = static_cast<int>((pt.y - grid->info.origin.position.y) / grid_resolution_);
    if (cx < 0 || cx >= grid_width_ || cy < 0 || cy >= grid_height_) continue;
    grid->data[static_cast<size_t>(cy * grid_width_ + cx)] = 100;
  }

  return grid;
}

}  // namespace eos_slam
