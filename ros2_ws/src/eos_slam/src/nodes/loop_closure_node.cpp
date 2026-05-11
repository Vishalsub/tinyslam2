#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "eos_slam/loop_closure/loop_closure_detector.hpp"
#include "eos_slam/utils/math_utils.hpp"

namespace eos_slam {

class LoopClosureNode : public rclcpp::Node {
public:
  LoopClosureNode() : Node("eos_loop_closure") {
    detector_ = std::make_unique<LoopClosureDetector>(this);

    auto sensor_qos = rclcpp::SensorDataQoS();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/eos/vo/pose", sensor_qos,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
          latest_pose_ = math::poseMsgToIsometry(msg->pose);
          latest_stamp_ = msg->header.stamp;
          has_pose_ = true;
        });

    // Use the front LiDAR for loop closure scan matching.
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/eos/lidar/front_filtered", sensor_qos,
        std::bind(&LoopClosureNode::lidar_callback, this, std::placeholders::_1));

    // Corrected path published after loop closure.
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/eos/loop_closure/path", rclcpp::QoS(1).transient_local());

    // Status string for RViz / logging.
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/eos/loop_closure/status", 10);

    RCLCPP_INFO(get_logger(), "Loop closure node started");
  }

private:
  std::unique_ptr<LoopClosureDetector> detector_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              status_pub_;

  Eigen::Isometry3d latest_pose_{Eigen::Isometry3d::Identity()};
  rclcpp::Time      latest_stamp_;
  bool              has_pose_{false};

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    if (!has_pose_) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
        pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    auto result = detector_->addKeyframe(latest_pose_, cloud, latest_stamp_);

    auto status_msg = std_msgs::msg::String{};
    status_msg.data = "Keyframes: " + std::to_string(detector_->keyframeCount());

    if (result) {
      auto corrected_path = detector_->applyCorrection(*result);
      corrected_path.header.stamp = this->get_clock()->now();
      path_pub_->publish(corrected_path);

      status_msg.data += " | Loop closed KF " +
          std::to_string(result->from_id) + "↔" +
          std::to_string(result->to_id) +
          " fitness=" + std::to_string(result->fitness).substr(0, 6);

      RCLCPP_INFO(get_logger(), "%s", status_msg.data.c_str());
    }
    status_pub_->publish(status_msg);
  }
};

}  // namespace eos_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<eos_slam::LoopClosureNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
}
