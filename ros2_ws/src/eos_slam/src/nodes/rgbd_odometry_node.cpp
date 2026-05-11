#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "eos_slam/visual_odometry/rgbd_odometry.hpp"
#include "eos_slam/utils/ros_utils.hpp"

namespace eos_slam {

class RGBDOdometryNode : public rclcpp::Node {
public:
  RGBDOdometryNode() : Node("eos_rgbd_odometry") {
    vo_ = std::make_unique<RGBDOdometry>(this);

    // Use SensorDataQoS (BEST_EFFORT) to match the sensor bridge publisher.
    auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    rgb_sub_.subscribe(this, "/eos/input/rgb", sensor_qos);
    depth_sub_.subscribe(this, "/eos/input/depth", sensor_qos);
    info_sub_.subscribe(this, "/eos/input/camera_info", sensor_qos);

    sync_ = std::make_shared<Syncer>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
    sync_->registerCallback(
      std::bind(&RGBDOdometryNode::sync_callback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    vo_pose_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>("/eos/vo/pose", 10);
    vo_path_pub_    = create_publisher<nav_msgs::msg::Path>("/eos/vo/path", 10);
    vo_debug_pub_   = create_publisher<sensor_msgs::msg::Image>("/eos/vo/debug_matches", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(get_logger(), "RGB-D Odometry node started");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Syncer = message_filters::Synchronizer<SyncPolicy>;

  std::unique_ptr<RGBDOdometry> vo_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Syncer> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vo_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vo_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vo_debug_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
    auto result = vo_->process(rgb, depth, info);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = rgb->header.stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = math::isometryToPoseMsg(result.current_pose);
    vo_pose_pub_->publish(pose_msg);

    vo_path_pub_->publish(result.path);

    if (!result.debug_matches.empty()) {
      auto debug_msg = ros::cvToImageMsg(result.debug_matches, "bgr8", rgb->header);
      vo_debug_pub_->publish(*debug_msg);
    }

    if (result.valid) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = rgb->header.stamp;
      tf.header.frame_id = "map";
      tf.child_frame_id = "camera_odom";
      tf.transform.translation.x = pose_msg.pose.position.x;
      tf.transform.translation.y = pose_msg.pose.position.y;
      tf.transform.translation.z = pose_msg.pose.position.z;
      tf.transform.rotation = pose_msg.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }
};

}  // namespace eos_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eos_slam::RGBDOdometryNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
