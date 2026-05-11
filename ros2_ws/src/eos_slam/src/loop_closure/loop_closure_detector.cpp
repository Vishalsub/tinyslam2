#include "eos_slam/loop_closure/loop_closure_detector.hpp"
#include "eos_slam/utils/math_utils.hpp"

#include <pcl/filters/voxel_grid.h>

namespace eos_slam {

LoopClosureDetector::LoopClosureDetector(rclcpp::Node* node) : node_(node) {
  node->declare_parameter("loop.min_keyframe_dist",   0.5);
  node->declare_parameter("loop.candidate_radius",    3.0);
  node->declare_parameter("loop.min_age_gap",         20);
  node->declare_parameter("loop.icp_max_dist",        0.5);
  node->declare_parameter("loop.icp_fitness_thresh",  0.05);
  node->declare_parameter("loop.icp_max_iter",        50);

  min_keyframe_dist_  = node->get_parameter("loop.min_keyframe_dist").as_double();
  candidate_radius_   = node->get_parameter("loop.candidate_radius").as_double();
  min_age_gap_        = node->get_parameter("loop.min_age_gap").as_int();
  icp_max_dist_       = node->get_parameter("loop.icp_max_dist").as_double();
  icp_fitness_thresh_ = node->get_parameter("loop.icp_fitness_thresh").as_double();
  icp_max_iter_       = node->get_parameter("loop.icp_max_iter").as_int();
}

// Downsample a cloud for faster ICP.
static pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float leaf) {
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(leaf, leaf, leaf);
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  vg.filter(*out);
  return out;
}

std::optional<LoopConstraint> LoopClosureDetector::addKeyframe(
    const Eigen::Isometry3d& pose,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const rclcpp::Time& stamp) {
  std::lock_guard<std::mutex> lock(kf_mutex_);

  // Only add a new keyframe when the robot has moved enough.
  if (!keyframes_.empty()) {
    double dist = math::poseDistance(keyframes_.back().pose, pose);
    if (dist < min_keyframe_dist_) return std::nullopt;
  }

  Keyframe kf;
  kf.id    = static_cast<int>(keyframes_.size());
  kf.pose  = pose;
  kf.cloud = downsample(cloud, 0.1f);
  kf.stamp = stamp;
  keyframes_.push_back(kf);

  const int cur_id = kf.id;
  RCLCPP_DEBUG(node_->get_logger(), "Keyframe %d added (total: %d)", cur_id, cur_id + 1);

  // Search for loop closure candidates.
  for (int i = 0; i < cur_id - min_age_gap_; ++i) {
    double dist = math::poseDistance(keyframes_[i].pose, pose);
    if (dist > candidate_radius_) continue;

    RCLCPP_INFO(node_->get_logger(),
        "Loop candidate: KF %d ↔ KF %d  (dist=%.2f m)", i, cur_id, dist);

    auto result = tryICP(keyframes_[cur_id], keyframes_[i]);
    if (result) {
      RCLCPP_INFO(node_->get_logger(),
          "Loop closure confirmed KF %d ↔ KF %d  (fitness=%.4f)",
          cur_id, i, result->fitness);
      distributeCorrection(i, cur_id, result->relative_pose);
      return result;
    }
  }
  return std::nullopt;
}

std::optional<LoopConstraint> LoopClosureDetector::tryICP(
    const Keyframe& source, const Keyframe& target) {
  // Transform source cloud into map frame for ICP.
  auto src_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*source.cloud, *src_map,
                           source.pose.cast<float>());

  auto tgt_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*target.cloud, *tgt_map,
                           target.pose.cast<float>());

  if (src_map->empty() || tgt_map->empty()) return std::nullopt;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_map);
  icp.setInputTarget(tgt_map);
  icp.setMaxCorrespondenceDistance(icp_max_dist_);
  icp.setMaximumIterations(icp_max_iter_);
  icp.setTransformationEpsilon(1e-6);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned);

  if (!icp.hasConverged()) return std::nullopt;
  double fitness = icp.getFitnessScore();
  if (fitness > icp_fitness_thresh_) return std::nullopt;

  // ICP gives us T_icp that moves source closer to target.
  Eigen::Matrix4f T = icp.getFinalTransformation();
  Eigen::Isometry3d T_corr = Eigen::Isometry3d::Identity();
  T_corr.matrix() = T.cast<double>();

  LoopConstraint lc;
  lc.from_id       = source.id;
  lc.to_id         = target.id;
  lc.relative_pose = T_corr;
  lc.fitness       = fitness;
  return lc;
}

void LoopClosureDetector::distributeCorrection(
    int from_id, int to_id, const Eigen::Isometry3d& T_correction) {
  // Linearly interpolate the correction across keyframes [from_id+1 .. to_id].
  // At from_id: 0 correction.  At to_id: full correction.
  int span = to_id - from_id;
  if (span <= 0) return;

  // Extract translation and angle-axis for slerp.
  Eigen::Vector3d dt = T_correction.translation();
  Eigen::AngleAxisd aa(T_correction.rotation());

  for (int i = from_id + 1; i <= to_id; ++i) {
    double alpha = static_cast<double>(i - from_id) / static_cast<double>(span);

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = alpha * dt;
    delta.rotate(Eigen::AngleAxisd(alpha * aa.angle(), aa.axis()));

    keyframes_[i].pose = delta * keyframes_[i].pose;
  }
}

nav_msgs::msg::Path LoopClosureDetector::applyCorrection(const LoopConstraint& /*lc*/) {
  std::lock_guard<std::mutex> lock(kf_mutex_);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto& kf : keyframes_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp    = kf.stamp;
    ps.pose.position.x = kf.pose.translation().x();
    ps.pose.position.y = kf.pose.translation().y();
    ps.pose.position.z = kf.pose.translation().z();
    Eigen::Quaterniond q(kf.pose.rotation());
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace eos_slam
