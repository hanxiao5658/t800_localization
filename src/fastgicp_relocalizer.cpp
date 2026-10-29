// T800 / FAST-LIO2 / fixed-map FastGICP initial relocalization, ROS1 Noetic.
// Input /cloud_registered is already a scan transformed into FAST-LIO's
// camera_init (= odometry) frame. Never treat it as an IMU/body-frame scan.
// Output: map -> camera_init. No changes to FAST-LIO or navigation controller.
// NOTE: first-stage alignment experiment, not a safety-rated localization source.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

namespace {
Eigen::Isometry3d poseFromMsg(const geometry_msgs::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                       p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-8) {
    throw std::runtime_error("invalid IMU quaternion");
  }
  q.normalize();
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return out;
}

Eigen::Isometry3d poseFromXyzRpy(const std::vector<double>& a) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = (Eigen::AngleAxisd(a[5], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(a[4], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(a[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  out.translation() = Eigen::Vector3d(a[0], a[1], a[2]);
  return out;
}

Cloud::Ptr downsample(const Cloud::ConstPtr& input, double leaf) {
  Cloud::Ptr output(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(static_cast<float>(leaf),
                    static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.filter(*output);
  return output;
}

double yawOf(const Eigen::Isometry3d& t) {
  return std::atan2(t.linear()(1, 0), t.linear()(0, 0));
}
double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

class Relocalizer {
 public:
  Relocalizer() : nh_(), pnh_("~") {
    pnh_.param<std::string>("map_pcd_file", map_path_, "");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("registered_cloud_topic", cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param("initial_pose_configured", initial_pose_configured_, false);
    pnh_.param("auto_relocalize", auto_relocalize_, false);
    pnh_.param("publish_tf", publish_tf_, false);
    pnh_.param("min_scans", min_scans_, 10);
    pnh_.param("max_scans", max_scans_, 25);
    pnh_.param("min_source_points", min_source_points_, 150);
    pnh_.param("min_target_points", min_target_points_, 300);
    pnh_.param("map_voxel_size", map_voxel_size_, 0.20);
    pnh_.param("scan_voxel_size", scan_voxel_size_, 0.15);
    pnh_.param("local_map_radius_xy", map_radius_xy_, 12.0);
    pnh_.param("local_map_half_height", map_half_z_, 4.0);
    pnh_.param("max_correspondence_distance", max_correspondence_, 1.2);
    pnh_.param("max_fitness_mse", max_fitness_mse_, 0.25);
    pnh_.param("overlap_distance", overlap_distance_, 0.50);
    pnh_.param("min_overlap_fraction", min_overlap_fraction_, 0.30);
    pnh_.param("max_translation_correction", max_translation_correction_, 1.5);
    pnh_.param("max_yaw_correction_deg", max_yaw_correction_deg_, 25.0);
    pnh_.param("max_data_age", max_data_age_, 1.0);
    pnh_.param("gicp_threads", gicp_threads_, 4);
    pnh_.param("gicp_iterations", gicp_iterations_, 50);

    if (map_path_.empty()) throw std::runtime_error("~map_pcd_file is empty");
    if (min_scans_ < 1 || max_scans_ < min_scans_ || map_voxel_size_ <= 0 ||
        scan_voxel_size_ <= 0 || map_radius_xy_ <= 0 ||
        map_half_z_ <= 0 || max_correspondence_ <= 0) {
      throw std::runtime_error("invalid scan count, voxel size, or map radius");
    }
    if (initial_pose_configured_) {
      if (!pnh_.getParam("initial_imu_pose_in_map", initial_pose_) ||
          initial_pose_.size() != 6) {
        throw std::runtime_error("set ~initial_imu_pose_in_map to [x,y,z,roll,pitch,yaw]");
      }
      for (double v : initial_pose_) if (!std::isfinite(v))
        throw std::runtime_error("non-finite initial pose");
      map_imu_reference_ = poseFromXyzRpy(initial_pose_);
    } else {
      ROS_WARN("Initial IMU pose not configured. Set initial_pose_configured: true before matching.");
    }

    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<Point>(map_path_, *raw) < 0 || raw->empty()) {
      throw std::runtime_error("failed to load PCD: " + map_path_);
    }
    Cloud::Ptr valid(new Cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*raw, *valid, indices);
    map_cloud_ = downsample(valid, map_voxel_size_);
    if (map_cloud_->size() < static_cast<size_t>(min_target_points_)) {
      throw std::runtime_error("map has too few points after downsampling");
    }
    ROS_INFO("Loaded prior map: %zu raw points, %zu downsampled", raw->size(), map_cloud_->size());

    status_pub_ = nh_.advertise<std_msgs::String>("/my_ros1/localization/status", 1, true);
    transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
        "/my_ros1/localization/map_to_odom", 1, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/my_ros1/localization/aligned_scan", 1, true);
    service_ = nh_.advertiseService("/my_ros1/localization/relocalize",
                                    &Relocalizer::serviceCb, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 50, &Relocalizer::odomCb, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 10, &Relocalizer::cloudCb, this);
    timer_ = nh_.createTimer(ros::Duration(1.0), &Relocalizer::timerCb, this);
    status_ = "UNINITIALIZED";
    publishStatus();
    ROS_INFO("FastGICP node ready. odom=%s cloud=%s auto=%s",
             odom_topic_.c_str(), cloud_topic_.c_str(), auto_relocalize_ ? "true" : "false");
  }

 private:
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.frame_id.empty() || msg->child_frame_id.empty()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_odom_ = *msg;
    have_odom_ = true;
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id.empty()) return;
    Cloud::Ptr cloud(new Cloud);
    try { pcl::fromROSMsg(*msg, *cloud); }
    catch (const std::exception& e) {
      ROS_WARN_THROTTLE(3.0, "Cannot decode /cloud_registered: %s", e.what());
      return;
    }
    if (cloud->empty()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (frame_of_scans_ != msg->header.frame_id) {
      scans_.clear();
      frame_of_scans_ = msg->header.frame_id;
    }
    scans_.push_back(cloud);
    while (static_cast<int>(scans_.size()) > max_scans_) scans_.pop_front();
    last_cloud_stamp_ = msg->header.stamp;
  }

  bool serviceCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::string detail;
    res.success = attemptAlignment(detail);
    res.message = detail;
    return true;
  }

  void timerCb(const ros::TimerEvent&) {
    if (auto_relocalize_ && !aligned_ && auto_attempt_count_ < 3) {
      bool ready = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ready = have_odom_ && static_cast<int>(scans_.size()) >= min_scans_;
      }
      if (ready && initial_pose_configured_) {
        ++auto_attempt_count_;
        std::string detail;
        bool ok = attemptAlignment(detail);
        ROS_INFO("Auto relocalization #%d: %s (%s)", auto_attempt_count_,
                 ok ? "OK" : "FAILED", detail.c_str());
      }
    }
    publishStatus();
    if (aligned_) publishTransform();
  }

  bool attemptAlignment(std::string& detail) {
    if (busy_.exchange(true)) { detail = "registration already in progress"; return false; }
    struct BusyGuard { std::atomic<bool>& a; ~BusyGuard() { a.store(false); } } guard{busy_};
    if (!initial_pose_configured_ && !aligned_) {
      detail = "initial_imu_pose_in_map not configured"; setStatus("REJECTED"); return false;
    }
    std::deque<Cloud::Ptr> scans;
    nav_msgs::Odometry odom;
    ros::Time cloud_stamp;
    std::string frame;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_odom_ || static_cast<int>(scans_.size()) < min_scans_) {
        detail = "waiting for /Odometry and sufficient /cloud_registered scans";
        setStatus("COLLECTING"); return false;
      }
      odom = last_odom_;
      scans = scans_;
      cloud_stamp = last_cloud_stamp_;
      frame = frame_of_scans_;
    }
    if (frame != odom.header.frame_id) {
      detail = "cloud frame is " + frame + ", odometry parent is " + odom.header.frame_id;
      setStatus("REJECTED_FRAME_MISMATCH"); return false;
    }
    const ros::Time now = ros::Time::now();
    if (!now.isZero() && ((now - cloud_stamp).toSec() > max_data_age_ ||
                           (now - odom.header.stamp).toSec() > max_data_age_)) {
      detail = "point cloud or odometry is stale";
      setStatus("REJECTED_STALE_DATA"); return false;
    }

    Eigen::Isometry3d odom_imu;
    try { odom_imu = poseFromMsg(odom.pose.pose); }
    catch (const std::exception& e) {
      detail = e.what(); setStatus("REJECTED"); return false;
    }
    const Eigen::Isometry3d guess = aligned_ ? map_odom_ :
                                           map_imu_reference_ * odom_imu.inverse();
    const Eigen::Vector3d center = (guess * odom_imu).translation();

    Cloud::Ptr aggregated(new Cloud);
    for (const auto& scan : scans) *aggregated += *scan;
    Cloud::Ptr valid(new Cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*aggregated, *valid, indices);
    Cloud::Ptr compact = downsample(valid, scan_voxel_size_);

    // Both clouds should represent the SAME local scene. Crop the map in map,
    // and the odom-frame scan using the current map<-odom initial guess.
    Cloud::Ptr source(new Cloud);
    const Eigen::Matrix4f guess_f = guess.matrix().cast<float>();
    for (const auto& p : compact->points) {
      Eigen::Vector4f v(p.x, p.y, p.z, 1.0f);
      Eigen::Vector4f m = guess_f * v;
      if (std::fabs(m.x() - center.x()) < map_radius_xy_ &&
          std::fabs(m.y() - center.y()) < map_radius_xy_ &&
          std::fabs(m.z() - center.z()) < map_half_z_) source->push_back(p);
    }
    if (source->size() < static_cast<size_t>(min_source_points_)) {
      detail = "source too sparse: " + std::to_string(source->size());
      setStatus("REJECTED_SPARSE_SCAN"); return false;
    }
    pcl::CropBox<Point> crop;
    crop.setInputCloud(map_cloud_);
    crop.setMin(Eigen::Vector4f(center.x()-map_radius_xy_, center.y()-map_radius_xy_,
                                center.z()-map_half_z_, 1.0));
    crop.setMax(Eigen::Vector4f(center.x()+map_radius_xy_, center.y()+map_radius_xy_,
                                center.z()+map_half_z_, 1.0));
    Cloud::Ptr target(new Cloud);
    crop.filter(*target);
    if (target->size() < static_cast<size_t>(min_target_points_)) {
      detail = "target too sparse: " + std::to_string(target->size());
      setStatus("REJECTED_SPARSE_MAP"); return false;
    }
    setStatus("MATCHING");
    fast_gicp::FastGICP<Point, Point> gicp;
    gicp.setNumThreads(std::max(1, gicp_threads_));
    gicp.setMaximumIterations(std::max(1, gicp_iterations_));
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaxCorrespondenceDistance(max_correspondence_);
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    Cloud aligned_scan;
    gicp.align(aligned_scan, guess_f);
    if (!gicp.hasConverged()) {
      detail = "GICP did not converge";
      setStatus("REJECTED_NOT_CONVERGED"); return false;
    }
    const Eigen::Matrix4f solution_f = gicp.getFinalTransformation();
    if (!solution_f.allFinite()) {
      detail = "non-finite GICP result";
      setStatus("REJECTED"); return false;
    }
    Eigen::Isometry3d solution = Eigen::Isometry3d::Identity();
    solution.matrix() = solution_f.cast<double>();
    const Eigen::Isometry3d delta = guess.inverse() * solution;
    const double translation_change = delta.translation().norm();
    const double yaw_change_deg = std::fabs(wrapAngle(yawOf(delta))) * 180.0 / M_PI;
    const double fitness_mse = gicp.getFitnessScore(max_correspondence_);

    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    int n_inliers = 0;
    const float dist2_threshold = static_cast<float>(overlap_distance_ * overlap_distance_);
    std::vector<int> nn_index(1);
    std::vector<float> nn_dist2(1);
    for (const auto& p : aligned_scan.points) {
      if (tree.nearestKSearch(p, 1, nn_index, nn_dist2) > 0 && nn_dist2[0] < dist2_threshold) ++n_inliers;
    }
    const double overlap = static_cast<double>(n_inliers) / source->size();
    std::ostringstream report;
    report.setf(std::ios::fixed);
    report.precision(3);
    report << "source=" << source->size() << " map=" << target->size()
           << " fitness_mse=" << fitness_mse << " overlap=" << overlap
           << " translation_correction=" << translation_change << "m"
           << " yaw_correction=" << yaw_change_deg << "deg";
    detail = report.str();
    if (!std::isfinite(fitness_mse) || fitness_mse > max_fitness_mse_ ||
        overlap < min_overlap_fraction_ ||
        translation_change > max_translation_correction_ ||
        yaw_change_deg > max_yaw_correction_deg_) {
      ROS_WARN("GICP rejected: %s", detail.c_str());
      setStatus("REJECTED_QUALITY");
      return false;
    }
    map_odom_ = solution;
    odom_frame_ = frame;
    aligned_ = true;
    setStatus("ALIGNED_INITIAL_ONLY");
    ROS_INFO("GICP accepted: %s; map<-odom translation=(%.3f, %.3f, %.3f) yaw=%.3f",
             detail.c_str(), solution.translation().x(), solution.translation().y(),
             solution.translation().z(), yawOf(solution));
    sensor_msgs::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned_scan, aligned_msg);
    aligned_msg.header.stamp = cloud_stamp;
    aligned_msg.header.frame_id = map_frame_;
    aligned_pub_.publish(aligned_msg);
    publishTransform();
    return true;
  }

  void setStatus(const std::string& text) { status_ = text; publishStatus(); }
  void publishStatus() {
    std_msgs::String msg;
    msg.data = status_;
    status_pub_.publish(msg);
  }
  void publishTransform() {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = odom_frame_;
    msg.transform.translation.x = map_odom_.translation().x();
    msg.transform.translation.y = map_odom_.translation().y();
    msg.transform.translation.z = map_odom_.translation().z();
    Eigen::Quaterniond q(map_odom_.linear());
    msg.transform.rotation.x = q.x(); msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z(); msg.transform.rotation.w = q.w();
    transform_pub_.publish(msg);
    if (publish_tf_) tf_broadcaster_.sendTransform(msg);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, cloud_sub_;
  ros::Publisher status_pub_, transform_pub_, aligned_pub_;
  ros::ServiceServer service_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::mutex data_mutex_;
  std::atomic<bool> busy_{false};
  nav_msgs::Odometry last_odom_;
  bool have_odom_ = false;
  std::deque<Cloud::Ptr> scans_;
  ros::Time last_cloud_stamp_;
  std::string frame_of_scans_, odom_frame_, status_;
  std::string map_path_, map_frame_, odom_topic_, cloud_topic_;
  Cloud::Ptr map_cloud_;
  bool initial_pose_configured_ = false;
  bool auto_relocalize_ = false;
  bool publish_tf_ = false;
  bool aligned_ = false;
  int auto_attempt_count_ = 0;
  int min_scans_ = 10, max_scans_ = 25, min_source_points_ = 150;
  int min_target_points_ = 300, gicp_threads_ = 4, gicp_iterations_ = 50;
  double map_voxel_size_ = 0.2, scan_voxel_size_ = 0.15;
  double map_radius_xy_ = 12.0, map_half_z_ = 4.0;
  double max_correspondence_ = 1.2, max_fitness_mse_ = 0.25;
  double overlap_distance_ = 0.5, min_overlap_fraction_ = 0.30;
  double max_translation_correction_ = 1.5, max_yaw_correction_deg_ = 25;
  double max_data_age_ = 1.0;
  std::vector<double> initial_pose_;
  Eigen::Isometry3d map_imu_reference_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_odom_ = Eigen::Isometry3d::Identity();
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastgicp_relocalizer");
  try {
    Relocalizer node;
    // First-stage verification: use one callback thread to keep state
    // changes deterministic during an occasional startup GICP match.
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("FastGICP node failed: %s", e.what());
    return 1;
  }
  return 0;
}
