#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "eos_slam/utils/math_utils.hpp"

namespace eos_slam {

// Publishes:
//   /map           (OccupancyGrid, transient_local) ← relay of /eos/map/grid
//   TF: map → odom ← derived from VO pose + existing odom → base_link TF
//
// With this node running, Nav2 can use EOS-SLAM's map and pose estimates.
class Nav2BridgeNode : public rclcpp::Node {
public:
  Nav2BridgeNode() : Node("eos_nav2_bridge") {
    declare_parameter("odom_frame",      std::string("odom"));
    declare_parameter("base_frame",      std::string("base_link"));
    declare_parameter("publish_rate_hz", 20.0);

    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    double rate = get_parameter("publish_rate_hz").as_double();

    tf_broadcaster_        = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_             = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_           = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Relay EOS map → /map (transient_local so late subscribers get it).
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/eos/map/grid",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
          auto relay = *msg;
          relay.header.stamp = get_clock()->now();
          map_pub_->publish(relay);
        });

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());

    // Subscribe to EOS VO pose to derive map→odom TF.
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/eos/vo/pose", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
          T_map_camera_ = math::poseMsgToIsometry(msg->pose);
          has_pose_     = true;
        });

    // Timer to broadcast map→odom TF at the configured rate.
    tf_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&Nav2BridgeNode::broadcastTf, this));

    RCLCPP_INFO(get_logger(),
        "Nav2 bridge started. Relaying /eos/map/grid → /map and publishing map→odom TF.");
  }

private:
  std::string odom_frame_;
  std::string base_frame_;

  bool              has_pose_{false};
  Eigen::Isometry3d T_map_camera_{Eigen::Isometry3d::Identity()};

  std::shared_ptr<tf2_ros::TransformBroadcaster>        tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>                      tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>           tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr                                   tf_timer_;

  void broadcastTf() {
    if (!has_pose_) return;

    // Look up odom → base_link from the robot's existing TF.
    geometry_msgs::msg::TransformStamped T_odom_base_msg;
    try {
      T_odom_base_msg = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "TF %s→%s unavailable: %s",
                           odom_frame_.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    // Reconstruct T_odom_base as Eigen.
    const auto& tr = T_odom_base_msg.transform.translation;
    const auto& rt = T_odom_base_msg.transform.rotation;
    Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
    T_odom_base.translation() << tr.x, tr.y, tr.z;
    T_odom_base.rotate(Eigen::Quaterniond(rt.w, rt.x, rt.y, rt.z).normalized());

    // VO gives us T_map_camera (map → camera frame).
    // We assume camera ≈ base for the TF derivation:
    //   T_map_odom = T_map_base * T_odom_base^-1
    //              ≈ T_map_camera * T_odom_base^-1
    Eigen::Isometry3d T_map_odom = T_map_camera_ * T_odom_base.inverse();

    geometry_msgs::msg::TransformStamped tf_out;
    tf_out.header.stamp    = get_clock()->now();
    tf_out.header.frame_id = "map";
    tf_out.child_frame_id  = odom_frame_;

    const Eigen::Vector3d& t = T_map_odom.translation();
    tf_out.transform.translation.x = t.x();
    tf_out.transform.translation.y = t.y();
    tf_out.transform.translation.z = t.z();

    Eigen::Quaterniond q(T_map_odom.rotation());
    tf_out.transform.rotation.w = q.w();
    tf_out.transform.rotation.x = q.x();
    tf_out.transform.rotation.y = q.y();
    tf_out.transform.rotation.z = q.z();

    tf_broadcaster_->sendTransform(tf_out);
  }
};

}  // namespace eos_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eos_slam::Nav2BridgeNode>());
  rclcpp::shutdown();
}
