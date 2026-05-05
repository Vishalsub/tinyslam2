#include "eos_slam/visual_odometry/rgbd_odometry.hpp"

namespace eos_slam {

RGBDOdometry::RGBDOdometry(rclcpp::Node* node)
    : matcher_(cv::NORM_HAMMING), frame_count_(0), initialized_(false) {
  node->declare_parameter("vo.max_features", 500);
  node->declare_parameter("vo.min_depth", 0.1);
  node->declare_parameter("vo.max_depth", 10.0);
  node->declare_parameter("vo.keyframe_interval", 5);

  max_features_     = node->get_parameter("vo.max_features").as_int();
  min_depth_        = node->get_parameter("vo.min_depth").as_double();
  max_depth_        = node->get_parameter("vo.max_depth").as_double();
  keyframe_interval_ = node->get_parameter("vo.keyframe_interval").as_int();

  orb_ = cv::ORB::create(max_features_, 1.2f, 8, 31, 0, 2,
                         cv::ORB::HARRIS_SCORE, 31, 20);

  accumulated_pose_ = Eigen::Isometry3d::Identity();
  path_.header.frame_id = "map";
}

VOResult RGBDOdometry::process(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
  VOResult result;
  result.valid = false;
  result.current_pose = accumulated_pose_;
  result.path = path_;

  if (intrinsics_.fx == 0.0) {
    intrinsics_ = ros::CameraIntrinsics::fromMsg(*info_msg);
  }

  cv::Mat gray = cv_bridge::toCvShare(rgb_msg, "mono8")->image;
  cv::Mat depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;

  std::vector<cv::KeyPoint> kps;
  cv::Mat descs;
  orb_->detectAndCompute(gray, cv::noArray(), kps, descs);

  if (!initialized_) {
    prev_gray_ = gray.clone();
    prev_kps_ = kps;
    prev_descs_ = descs.clone();
    initialized_ = true;
    result.valid = true;
    result.debug_matches = gray.clone();
    cv::cvtColor(result.debug_matches, result.debug_matches, cv::COLOR_GRAY2BGR);
    cv::putText(result.debug_matches, "Initialized", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    return result;
  }

  result = computePose(gray, depth, kps, descs);

  if (result.valid) {
    prev_gray_ = gray.clone();
    prev_kps_ = kps;
    prev_descs_ = descs.clone();
    frame_count_++;
  }

  return result;
}

VOResult RGBDOdometry::computePose(
    const cv::Mat& gray, const cv::Mat& depth,
    const std::vector<cv::KeyPoint>& kps, const cv::Mat& descs) {
  VOResult result;
  result.valid = false;
  result.current_pose = accumulated_pose_;
  result.path = path_;

  auto matches = matchFeatures(prev_descs_, descs);
  if (matches.size() < 8) {
    result.debug_matches = drawMatchesDebug(prev_gray_, prev_kps_, gray, kps, matches);
    return result;
  }

  std::vector<cv::Point3f> pts_3d;
  auto corr = getCorrespondencesWithDepth(prev_kps_, kps, matches, depth, pts_3d);
  if (corr.size() < 6) {
    result.debug_matches = drawMatchesDebug(prev_gray_, prev_kps_, gray, kps, matches);
    return result;
  }

  std::vector<cv::Point2f> pts_2d;
  for (const auto& c : corr) { pts_2d.push_back(c); }

  cv::Mat rvec, tvec, inliers;
  cv::Mat K = (cv::Mat_<double>(3, 3) <<
    intrinsics_.fx, 0.0, intrinsics_.cx,
    0.0, intrinsics_.fy, intrinsics_.cy,
    0.0, 0.0, 1.0);

  cv::solvePnPRansac(pts_3d, pts_2d, K, cv::noArray(),
                     rvec, tvec, false, 100, 2.0, 0.99, inliers);

  cv::Mat R_3x3;
  cv::Rodrigues(rvec, R_3x3);
  Eigen::Matrix3d R_eigen;
  R_eigen << R_3x3.at<double>(0,0), R_3x3.at<double>(0,1), R_3x3.at<double>(0,2),
             R_3x3.at<double>(1,0), R_3x3.at<double>(1,1), R_3x3.at<double>(1,2),
             R_3x3.at<double>(2,0), R_3x3.at<double>(2,1), R_3x3.at<double>(2,2);

  Eigen::Vector3d t_eigen(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  delta.rotate(R_eigen);
  delta.translation() = t_eigen;

  accumulated_pose_ = accumulated_pose_ * delta;

  result.current_pose = accumulated_pose_;
  result.valid = true;

  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = rclcpp::Clock().now();
  ps.header.frame_id = "map";
  ps.pose = math::isometryToPoseMsg(accumulated_pose_);
  path_.poses.push_back(ps);
  result.path = path_;

  std::vector<cv::DMatch> inlier_matches;
  for (int i = 0; i < inliers.rows; ++i) {
    inlier_matches.push_back(matches[inliers.at<int>(i)]);
  }
  result.debug_matches = drawMatchesDebug(prev_gray_, prev_kps_, gray, kps, inlier_matches);

  return result;
}

std::vector<cv::DMatch> RGBDOdometry::matchFeatures(
    const cv::Mat& desc1, const cv::Mat& desc2) {
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher_.knnMatch(desc1, desc2, knn_matches, 2);

  std::vector<cv::DMatch> good;
  for (const auto& knn : knn_matches) {
    if (knn.size() == 2 && knn[0].distance < 0.75 * knn[1].distance) {
      good.push_back(knn[0]);
    }
  }
  return good;
}

std::vector<cv::Point2f> RGBDOdometry::getCorrespondencesWithDepth(
    const std::vector<cv::KeyPoint>& kps1,
    const std::vector<cv::KeyPoint>& kps2,
    const std::vector<cv::DMatch>& matches,
    const cv::Mat& depth,
    std::vector<cv::Point3f>& points_3d) {
  std::vector<cv::Point2f> pts_2d;
  points_3d.clear();

  for (const auto& m : matches) {
    const auto& kp1 = kps1[m.queryIdx];
    int u = cvRound(kp1.pt.x);
    int v = cvRound(kp1.pt.y);

    if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) continue;

    float d = depth.at<float>(v, u);
    if (!std::isfinite(d) || d < min_depth_ || d > max_depth_) continue;

    auto pt3 = math::projectDepthTo3D(u, v, d,
      intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy);
    points_3d.emplace_back(pt3.x(), pt3.y(), pt3.z());
    pts_2d.emplace_back(kps2[m.trainIdx].pt);
  }
  return pts_2d;
}

cv::Mat RGBDOdometry::drawMatchesDebug(
    const cv::Mat& img1, const std::vector<cv::KeyPoint>& kps1,
    const cv::Mat& img2, const std::vector<cv::KeyPoint>& kps2,
    const std::vector<cv::DMatch>& matches) {
  cv::Mat out;
  cv::Mat c1, c2;
  cv::cvtColor(img1, c1, cv::COLOR_GRAY2BGR);
  cv::cvtColor(img2, c2, cv::COLOR_GRAY2BGR);
  cv::drawMatches(c1, kps1, c2, kps2, matches, out,
                  cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255));
  return out;
}

Eigen::Vector3d RGBDOdometry::depthLookup(const cv::Mat& depth, double u, double v) {
  int ui = cvRound(u), vi = cvRound(v);
  float d = 0.0;
  int count = 0;
  for (int dv = -1; dv <= 1; ++dv) {
    for (int du = -1; du <= 1; ++du) {
      int uu = ui + du, vv = vi + dv;
      if (uu >= 0 && uu < depth.cols && vv >= 0 && vv < depth.rows) {
        float val = depth.at<float>(vv, uu);
        if (std::isfinite(val) && val > 0) { d += val; count++; }
      }
    }
  }
  if (count == 0) return Eigen::Vector3d::Zero();
  d /= count;
  return math::projectDepthTo3D(u, v, d,
    intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy);
}

}  // namespace eos_slam
