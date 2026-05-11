#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "eos_slam/mapping/point_cloud_mapper.hpp"

namespace eos_slam {

class MappingNode : public rclcpp::Node {
public:
  MappingNode() : Node("eos_mapper") {
    mapper_ = std::make_unique<PointCloudMapper>(this);

    auto sensor_qos = rclcpp::SensorDataQoS();

    lidar_front_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/eos/lidar/front_filtered", sensor_qos,
        std::bind(&MappingNode::lidar_callback, this, std::placeholders::_1));

    lidar_back_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/eos/lidar/back_filtered", sensor_qos,
        std::bind(&MappingNode::lidar_callback, this, std::placeholders::_1));

    map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/eos/map/pointcloud", rclcpp::QoS(1).transient_local());

    map_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/eos/map/grid", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(),
        "Mapping node started — using TF2 to transform LiDAR clouds into map frame");
  }

private:
  std::unique_ptr<PointCloudMapper> mapper_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_back_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_grid_pub_;

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    mapper_->addCloud(msg);
    if (!mapper_->hasNewData()) return;
    mapper_->clearNewData();

    auto cloud = mapper_->getMapCloud("map");
    if (cloud) map_cloud_pub_->publish(*cloud);

    auto grid = mapper_->getOccupancyGrid();
    if (grid) map_grid_pub_->publish(*grid);
  }
};

}  // namespace eos_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eos_slam::MappingNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
