#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace eos_slam::math {

inline Eigen::Isometry3d poseMsgToIsometry(const geometry_msgs::msg::Pose& msg) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << msg.position.x, msg.position.y, msg.position.z;
  Eigen::Quaterniond q(
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  T.rotate(q.normalized());
  return T;
}

inline geometry_msgs::msg::Pose isometryToPoseMsg(const Eigen::Isometry3d& T) {
  geometry_msgs::msg::Pose msg;
  msg.position.x    = T.translation().x();
  msg.position.y    = T.translation().y();
  msg.position.z    = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

inline double poseDistance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (b.translation() - a.translation()).norm();
}

inline double poseAngleDiff(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  Eigen::Quaterniond qa(a.rotation());
  Eigen::Quaterniond qb(b.rotation());
  double dot = qa.dot(qb);
  dot = std::max(-1.0, std::min(1.0, dot));
  return 2.0 * std::acos(std::abs(dot));
}

inline Eigen::Vector3d projectDepthTo3D(
    double u, double v, double depth,
    double fx, double fy, double cx, double cy) {
  double x = (u - cx) * depth / fx;
  double y = (v - cy) * depth / fy;
  return Eigen::Vector3d(x, y, depth);
}

}  // namespace eos_slam::math
