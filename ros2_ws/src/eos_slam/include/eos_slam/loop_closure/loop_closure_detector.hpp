#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>
#include <optional>
#include <vector>
#include <mutex>

namespace eos_slam {

struct Keyframe {
  int                                         id;
  Eigen::Isometry3d                           pose;       // map frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr         cloud;      // LiDAR in sensor frame
  rclcpp::Time                                stamp;
};

struct LoopConstraint {
  int               from_id;
  int               to_id;
  Eigen::Isometry3d relative_pose;   // T_from_to computed by ICP
  double            fitness;
};

class LoopClosureDetector {
public:
  explicit LoopClosureDetector(rclcpp::Node* node);

  // Call with the latest VO pose and filtered LiDAR cloud.
  // Returns a LoopConstraint when a loop is found, otherwise nullopt.
  std::optional<LoopConstraint> addKeyframe(
      const Eigen::Isometry3d& pose,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
      const rclcpp::Time& stamp);

  // Apply the loop constraint and return the corrected full path.
  nav_msgs::msg::Path applyCorrection(const LoopConstraint& lc);

  // Access all keyframe poses (for map re-build after correction).
  const std::vector<Keyframe>& keyframes() const { return keyframes_; }

  int keyframeCount() const { return static_cast<int>(keyframes_.size()); }

private:
  rclcpp::Node* node_;

  // Parameters
  double min_keyframe_dist_;    // m — add new keyframe only if moved this far
  double candidate_radius_;     // m — search for loop candidates within this radius
  int    min_age_gap_;          // frames — avoid matching recent keyframes
  double icp_max_dist_;         // ICP correspondence distance threshold
  double icp_fitness_thresh_;   // ICP fitness score threshold (lower = better fit)
  int    icp_max_iter_;

  std::vector<Keyframe> keyframes_;
  mutable std::mutex    kf_mutex_;

  std::optional<LoopConstraint> tryICP(
      const Keyframe& source, const Keyframe& target);

  // Distribute correction linearly between keyframe indices [from, to].
  void distributeCorrection(int from_id, int to_id,
                             const Eigen::Isometry3d& T_error);
};

}  // namespace eos_slam
