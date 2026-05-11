#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "eos_slam/sensor_processors/rgb_processor.hpp"
#include "eos_slam/sensor_processors/depth_processor.hpp"
#include "eos_slam/sensor_processors/lidar_processor.hpp"
#include "eos_slam/utils/ros_utils.hpp"

namespace eos_slam {

class SensorProcessorNode : public rclcpp::Node {
public:
  SensorProcessorNode() : Node("eos_sensor_processor") {
    rgb_proc_    = std::make_unique<RGBProcessor>(this);
    depth_proc_  = std::make_unique<DepthProcessor>(this);
    lidar_proc_  = std::make_unique<LidarProcessor>(this);

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/eos/input/rgb", ros::sensor_data_qos(),
      std::bind(&SensorProcessorNode::rgb_callback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/eos/input/depth", ros::sensor_data_qos(),
      std::bind(&SensorProcessorNode::depth_callback, this, std::placeholders::_1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/eos/input/camera_info", ros::sensor_data_qos(),
      std::bind(&SensorProcessorNode::info_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/eos/input/odom", ros::sensor_data_qos(),
      std::bind(&SensorProcessorNode::odom_callback, this, std::placeholders::_1));

    lidar_front_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/eos/input/lidar_front", ros::sensor_data_qos(),
      std::bind(&SensorProcessorNode::lidar_front_callback, this, std::placeholders::_1));

    lidar_back_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/eos/input/lidar_back", ros::sensor_data_qos(),
      std::bind(&SensorProcessorNode::lidar_back_callback, this, std::placeholders::_1));

    features_pub_       = create_publisher<sensor_msgs::msg::Image>("/eos/features/image", 10);
    depth_colormap_pub_ = create_publisher<sensor_msgs::msg::Image>("/eos/debug/depth_colormap", 10);
    points_3d_pub_      = create_publisher<sensor_msgs::msg::PointCloud2>("/eos/features/points_3d", 10);
    odom_path_pub_      = create_publisher<nav_msgs::msg::Path>("/eos/odom/path", 10);
    lidar_front_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>("/eos/lidar/front_filtered", 10);
    lidar_back_pub_     = create_publisher<sensor_msgs::msg::PointCloud2>("/eos/lidar/back_filtered", 10);

    odom_path_.header.frame_id = "map";

    RCLCPP_INFO(get_logger(), "Sensor processor node started");
  }

private:
  std::unique_ptr<RGBProcessor> rgb_proc_;
  std::unique_ptr<DepthProcessor> depth_proc_;
  std::unique_ptr<LidarProcessor> lidar_proc_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_back_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr features_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colormap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_3d_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_front_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_back_pub_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  nav_msgs::msg::Path odom_path_;

  void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    latest_info_ = msg;
    depth_proc_->setCameraInfo(msg);
  }

  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    auto result = rgb_proc_->process(msg);
    auto out_msg = ros::cvToImageMsg(result.debug_image, "bgr8", result.header);
    features_pub_->publish(*out_msg);
  }

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!latest_info_) return;
    auto result = depth_proc_->process(msg, latest_info_);
    auto cmap_msg = ros::cvToImageMsg(result.colormap_image, "bgr8", result.header);
    depth_colormap_pub_->publish(*cmap_msg);
    points_3d_pub_->publish(*result.points_3d);
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.header.frame_id = "map";
    ps.pose = msg->pose.pose;
    odom_path_.poses.push_back(ps);

    if (odom_path_.poses.size() > 10000) {
      odom_path_.poses.erase(odom_path_.poses.begin(), odom_path_.poses.begin() + 5000);
    }

    odom_path_.header.stamp = msg->header.stamp;
    odom_path_pub_->publish(odom_path_);
  }

  void lidar_front_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto filtered = lidar_proc_->process(msg);
    lidar_front_pub_->publish(*filtered);
  }

  void lidar_back_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto filtered = lidar_proc_->process(msg);
    lidar_back_pub_->publish(*filtered);
  }
};

}  // namespace eos_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eos_slam::SensorProcessorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
