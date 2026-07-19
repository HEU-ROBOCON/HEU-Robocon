// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <mutex>

namespace small_gicp_relocalization
{


namespace
{

constexpr double kPi = 3.14159265358979323846;

enum class GicpStage
{
  COARSE_RELOC,
  FINE_TRACKING
};

GicpStage g_stage = GicpStage::COARSE_RELOC;

bool g_two_stage_gicp = true;
bool g_update_xy_yaw_only = true;

int g_coarse_num_neighbors = 25;
double g_coarse_global_leaf_size = 0.15;
double g_coarse_registered_leaf_size = 0.12;
double g_coarse_max_dist_sq = 0.25;

int g_fine_num_neighbors = 30;
double g_fine_global_leaf_size = 0.10;
double g_fine_registered_leaf_size = 0.08;
double g_fine_max_dist_sq = 0.09;

int g_switch_to_fine_count = 5;
int g_switch_back_fail_count = 3;
int g_min_source_points = 150;

double g_coarse_accept_trans = 0.20;
double g_coarse_accept_yaw = 5.0 * kPi / 180.0;
double g_fine_accept_trans = 0.03;
double g_fine_accept_yaw = 0.8 * kPi / 180.0;

double g_coarse_apply_ratio = 0.50;
double g_fine_apply_ratio = 0.25;

// Fine tracking should not reject every correction larger than a few centimetres.
// A legitimate accumulated odometry drift may be larger than that. Instead, accept
// a reasonable candidate and clamp how much of it is applied per registration cycle.
double g_fine_max_step_trans = 0.015;
double g_fine_max_step_yaw = 0.25 * kPi / 180.0;

// Fine-stage scalar Kalman filter parameters. Small corrections are not discarded.
// Repeated corrections in the same direction are gradually absorbed, while
// zero-mean registration noise is attenuated.
bool g_fine_kalman_enable = true;
double g_fine_process_std_xy = 0.002;
double g_fine_measurement_std_xy = 0.006;
double g_fine_process_std_yaw_deg = 0.02;
double g_fine_measurement_std_yaw_deg = 0.10;

bool g_transform_input_cloud_to_odom = true;
std::mutex g_cloud_mutex;

// After a manual RViz /initialpose reset, the first few GICP iterations must be
// treated as a re-localization phase. Otherwise the fine-stage gates may reject
// the first correction, and stale accumulated scans may be matched against the
// new initial pose.
int g_after_initialpose_force_coarse_count = 6;
int g_after_initialpose_force_left = 0;
double g_initialpose_accept_trans = 1.00;
double g_initialpose_accept_yaw = 45.0 * kPi / 180.0;
double g_initialpose_apply_ratio = 1.00;

int g_good_match_count = 0;
int g_fail_match_count = 0;

double g_active_target_leaf_size = -1.0;
int g_active_target_num_neighbors = -1;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromIsometry(const Eigen::Isometry3d & t)
{
  return std::atan2(t.linear()(1, 0), t.linear()(0, 0));
}

Eigen::Isometry3d makePlanarTransform(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() << x, y, z;
  out.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return out;
}

Eigen::Isometry3d interpolateTransform(
  const Eigen::Isometry3d & old_t,
  const Eigen::Isometry3d & candidate_t,
  double ratio,
  bool xy_yaw_only)
{
  ratio = std::max(0.0, std::min(1.0, ratio));

  if (xy_yaw_only) {
    const double old_yaw = yawFromIsometry(old_t);
    const double candidate_yaw = yawFromIsometry(candidate_t);
    const double yaw = old_yaw + ratio * normalizeAngle(candidate_yaw - old_yaw);

    const double x = old_t.translation().x() +
      ratio * (candidate_t.translation().x() - old_t.translation().x());
    const double y = old_t.translation().y() +
      ratio * (candidate_t.translation().y() - old_t.translation().y());

    // Keep z from the previous TF. For a ground robot, z/roll/pitch GICP corrections
    // often introduce visible jitter and should not be used for 2D navigation.
    return makePlanarTransform(x, y, old_t.translation().z(), yaw);
  }

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = old_t.translation() +
    ratio * (candidate_t.translation() - old_t.translation());

  Eigen::Quaterniond q_old(old_t.rotation());
  Eigen::Quaterniond q_candidate(candidate_t.rotation());
  out.linear() = q_old.slerp(ratio, q_candidate).toRotationMatrix();
  return out;
}

Eigen::Isometry3d applyPlanarCorrectionClamped(
  const Eigen::Isometry3d & old_t,
  const Eigen::Isometry3d & candidate_t,
  double ratio,
  double max_trans_step,
  double max_yaw_step)
{
  ratio = std::max(0.0, std::min(1.0, ratio));

  const double dx = candidate_t.translation().x() - old_t.translation().x();
  const double dy = candidate_t.translation().y() - old_t.translation().y();
  const double distance = std::hypot(dx, dy);

  double trans_scale = ratio;
  if (distance > 1e-9 && max_trans_step > 0.0) {
    trans_scale = std::min(trans_scale, max_trans_step / distance);
  }

  const double old_yaw = yawFromIsometry(old_t);
  const double yaw_error = normalizeAngle(yawFromIsometry(candidate_t) - old_yaw);
  double yaw_step = ratio * yaw_error;
  if (max_yaw_step > 0.0) {
    yaw_step = std::max(-max_yaw_step, std::min(max_yaw_step, yaw_step));
  }

  return makePlanarTransform(
    old_t.translation().x() + trans_scale * dx,
    old_t.translation().y() + trans_scale * dy,
    old_t.translation().z(),
    old_yaw + yaw_step);
}

class ScalarKalmanFilter
{
public:
  void reset(double value, double initial_variance)
  {
    value_ = value;
    variance_ = std::max(initial_variance, 1e-12);
    initialized_ = true;
  }

  double updateLinear(
    double measurement,
    double process_variance,
    double measurement_variance)
  {
    process_variance = std::max(process_variance, 1e-12);
    measurement_variance = std::max(measurement_variance, 1e-12);

    if (!initialized_) {
      reset(measurement, measurement_variance);
      return value_;
    }

    variance_ += process_variance;
    const double gain = variance_ / (variance_ + measurement_variance);
    value_ += gain * (measurement - value_);
    variance_ = std::max((1.0 - gain) * variance_, 1e-12);
    return value_;
  }

  double updateAngle(
    double measurement,
    double process_variance,
    double measurement_variance)
  {
    process_variance = std::max(process_variance, 1e-12);
    measurement_variance = std::max(measurement_variance, 1e-12);

    if (!initialized_) {
      reset(normalizeAngle(measurement), measurement_variance);
      return value_;
    }

    variance_ += process_variance;
    const double gain = variance_ / (variance_ + measurement_variance);
    const double error = normalizeAngle(measurement - value_);
    value_ = normalizeAngle(value_ + gain * error);
    variance_ = std::max((1.0 - gain) * variance_, 1e-12);
    return value_;
  }

private:
  bool initialized_{false};
  double value_{0.0};
  double variance_{1.0};
};

ScalarKalmanFilter g_fine_filter_x;
ScalarKalmanFilter g_fine_filter_y;
ScalarKalmanFilter g_fine_filter_yaw;
bool g_fine_filter_ready = false;

void resetFineKalmanFilter(const Eigen::Isometry3d & transform)
{
  const double measurement_variance_xy =
    g_fine_measurement_std_xy * g_fine_measurement_std_xy;
  const double measurement_std_yaw =
    g_fine_measurement_std_yaw_deg * kPi / 180.0;
  const double measurement_variance_yaw =
    measurement_std_yaw * measurement_std_yaw;

  g_fine_filter_x.reset(
    transform.translation().x(), measurement_variance_xy);
  g_fine_filter_y.reset(
    transform.translation().y(), measurement_variance_xy);
  g_fine_filter_yaw.reset(
    yawFromIsometry(transform), measurement_variance_yaw);
  g_fine_filter_ready = true;
}

Eigen::Isometry3d filterFineTransform(
  const Eigen::Isometry3d & candidate,
  double fixed_z)
{
  if (!g_fine_filter_ready) {
    resetFineKalmanFilter(candidate);
  }

  const double process_variance_xy =
    g_fine_process_std_xy * g_fine_process_std_xy;
  const double measurement_variance_xy =
    g_fine_measurement_std_xy * g_fine_measurement_std_xy;

  const double process_std_yaw =
    g_fine_process_std_yaw_deg * kPi / 180.0;
  const double measurement_std_yaw =
    g_fine_measurement_std_yaw_deg * kPi / 180.0;

  const double filtered_x = g_fine_filter_x.updateLinear(
    candidate.translation().x(),
    process_variance_xy,
    measurement_variance_xy);
  const double filtered_y = g_fine_filter_y.updateLinear(
    candidate.translation().y(),
    process_variance_xy,
    measurement_variance_xy);
  const double filtered_yaw = g_fine_filter_yaw.updateAngle(
    yawFromIsometry(candidate),
    process_std_yaw * process_std_yaw,
    measurement_std_yaw * measurement_std_yaw);

  return makePlanarTransform(filtered_x, filtered_y, fixed_z, filtered_yaw);
}

const char * stageName()
{
  return g_stage == GicpStage::COARSE_RELOC ? "COARSE_RELOC" : "FINE_TRACKING";
}

}  // namespace

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->declare_parameter("two_stage_gicp", true);
  this->declare_parameter("update_xy_yaw_only", true);

  this->declare_parameter("coarse_num_neighbors", 25);
  this->declare_parameter("coarse_global_leaf_size", 0.15);
  this->declare_parameter("coarse_registered_leaf_size", 0.12);
  this->declare_parameter("coarse_max_dist_sq", 0.25);

  this->declare_parameter("fine_num_neighbors", 30);
  this->declare_parameter("fine_global_leaf_size", 0.10);
  this->declare_parameter("fine_registered_leaf_size", 0.08);
  this->declare_parameter("fine_max_dist_sq", 0.09);

  this->declare_parameter("switch_to_fine_count", 5);
  this->declare_parameter("switch_back_fail_count", 3);
  this->declare_parameter("min_source_points", 150);

  this->declare_parameter("coarse_accept_trans", 0.20);
  this->declare_parameter("coarse_accept_yaw_deg", 5.0);
  // These are hard rejection gates, not per-cycle output steps.
  // They must be wider than the expected accumulated odometry drift.
  this->declare_parameter("fine_accept_trans", 0.20);
  this->declare_parameter("fine_accept_yaw_deg", 5.0);

  this->declare_parameter("coarse_apply_ratio", 0.50);
  this->declare_parameter("fine_apply_ratio", 0.25);
  this->declare_parameter("fine_max_step_trans", 0.015);
  this->declare_parameter("fine_max_step_yaw_deg", 0.25);

  // Fine tracking Kalman filter. These are standard deviations, not variances.
  this->declare_parameter("fine_kalman_enable", true);
  this->declare_parameter("fine_process_std_xy", 0.002);
  this->declare_parameter("fine_measurement_std_xy", 0.006);
  this->declare_parameter("fine_process_std_yaw_deg", 0.02);
  this->declare_parameter("fine_measurement_std_yaw_deg", 0.10);

  this->declare_parameter("transform_input_cloud_to_odom", true);

  // Manual initialpose recovery parameters.
  this->declare_parameter("after_initialpose_force_coarse_count", 6);
  this->declare_parameter("initialpose_accept_trans", 1.00);
  this->declare_parameter("initialpose_accept_yaw_deg", 45.0);
  this->declare_parameter("initialpose_apply_ratio", 1.00);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  double coarse_accept_yaw_deg = 5.0;
  double fine_accept_yaw_deg = 0.8;

  this->get_parameter("two_stage_gicp", g_two_stage_gicp);
  this->get_parameter("update_xy_yaw_only", g_update_xy_yaw_only);

  this->get_parameter("coarse_num_neighbors", g_coarse_num_neighbors);
  this->get_parameter("coarse_global_leaf_size", g_coarse_global_leaf_size);
  this->get_parameter("coarse_registered_leaf_size", g_coarse_registered_leaf_size);
  this->get_parameter("coarse_max_dist_sq", g_coarse_max_dist_sq);

  this->get_parameter("fine_num_neighbors", g_fine_num_neighbors);
  this->get_parameter("fine_global_leaf_size", g_fine_global_leaf_size);
  this->get_parameter("fine_registered_leaf_size", g_fine_registered_leaf_size);
  this->get_parameter("fine_max_dist_sq", g_fine_max_dist_sq);

  this->get_parameter("switch_to_fine_count", g_switch_to_fine_count);
  this->get_parameter("switch_back_fail_count", g_switch_back_fail_count);
  this->get_parameter("min_source_points", g_min_source_points);

  this->get_parameter("coarse_accept_trans", g_coarse_accept_trans);
  this->get_parameter("coarse_accept_yaw_deg", coarse_accept_yaw_deg);
  this->get_parameter("fine_accept_trans", g_fine_accept_trans);
  this->get_parameter("fine_accept_yaw_deg", fine_accept_yaw_deg);

  this->get_parameter("coarse_apply_ratio", g_coarse_apply_ratio);
  this->get_parameter("fine_apply_ratio", g_fine_apply_ratio);
  this->get_parameter("fine_max_step_trans", g_fine_max_step_trans);
  double fine_max_step_yaw_deg = 0.25;
  this->get_parameter("fine_max_step_yaw_deg", fine_max_step_yaw_deg);

  this->get_parameter("fine_kalman_enable", g_fine_kalman_enable);
  this->get_parameter("fine_process_std_xy", g_fine_process_std_xy);
  this->get_parameter("fine_measurement_std_xy", g_fine_measurement_std_xy);
  this->get_parameter("fine_process_std_yaw_deg", g_fine_process_std_yaw_deg);
  this->get_parameter("fine_measurement_std_yaw_deg", g_fine_measurement_std_yaw_deg);

  this->get_parameter("transform_input_cloud_to_odom", g_transform_input_cloud_to_odom);
  g_fine_max_step_yaw = fine_max_step_yaw_deg * kPi / 180.0;

  g_fine_process_std_xy = std::max(0.0001, g_fine_process_std_xy);
  g_fine_measurement_std_xy = std::max(0.0001, g_fine_measurement_std_xy);
  g_fine_process_std_yaw_deg = std::max(0.001, g_fine_process_std_yaw_deg);
  g_fine_measurement_std_yaw_deg = std::max(0.001, g_fine_measurement_std_yaw_deg);

  double initialpose_accept_yaw_deg = 45.0;
  this->get_parameter("after_initialpose_force_coarse_count", g_after_initialpose_force_coarse_count);
  this->get_parameter("initialpose_accept_trans", g_initialpose_accept_trans);
  this->get_parameter("initialpose_accept_yaw_deg", initialpose_accept_yaw_deg);
  this->get_parameter("initialpose_apply_ratio", g_initialpose_apply_ratio);

  g_coarse_accept_yaw = coarse_accept_yaw_deg * kPi / 180.0;
  g_fine_accept_yaw = fine_accept_yaw_deg * kPi / 180.0;
  g_initialpose_accept_yaw = initialpose_accept_yaw_deg * kPi / 180.0;

  if (g_two_stage_gicp) {
    g_stage = GicpStage::COARSE_RELOC;
    g_good_match_count = 0;
    g_fail_match_count = 0;

    num_neighbors_ = g_coarse_num_neighbors;
    global_leaf_size_ = g_coarse_global_leaf_size;
    registered_leaf_size_ = g_coarse_registered_leaf_size;
    max_dist_sq_ = g_coarse_max_dist_sq;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "two_stage_gicp=%s, initial_stage=%s, coarse(max_dist_sq=%.3f, leaf=%.3f/%.3f), "
    "fine(max_dist_sq=%.3f, leaf=%.3f/%.3f), update_xy_yaw_only=%s, "
    "fine_kalman=%s, kalman_xy(process=%.4f, measurement=%.4f)",
    g_two_stage_gicp ? "true" : "false", stageName(),
    g_coarse_max_dist_sq, g_coarse_global_leaf_size, g_coarse_registered_leaf_size,
    g_fine_max_dist_sq, g_fine_global_leaf_size, g_fine_registered_leaf_size,
    g_update_xy_yaw_only ? "true" : "false",
    g_fine_kalman_enable ? "true" : "false",
    g_fine_process_std_xy, g_fine_measurement_std_xy);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  g_active_target_leaf_size = global_leaf_size_;
  g_active_target_num_neighbors = num_neighbors_;

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  // The GICP result is published as map -> odom. Therefore every source cloud
  // must be expressed in odom_frame_. If the input cloud is in camera_init or
  // another frame and is used directly, the estimated transform is not map -> odom
  // and the global correction cannot consistently remove Point-LIO drift.
  if (
    g_transform_input_cloud_to_odom &&
    !odom_frame_.empty() &&
    !msg->header.frame_id.empty() &&
    msg->header.frame_id != odom_frame_)
  {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        odom_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.10));
      const Eigen::Isometry3d odom_from_cloud = tf2::transformToEigen(tf_stamped.transform);
      pcl::PointCloud<pcl::PointXYZ> transformed;
      pcl::transformPointCloud(*scan, transformed, odom_from_cloud.matrix().cast<float>());
      *scan = std::move(transformed);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform input cloud from %s to %s: %s",
        msg->header.frame_id.c_str(), odom_frame_.c_str(), ex.what());
      return;
    }
  }

  std::lock_guard<std::mutex> lock(g_cloud_mutex);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud(
    new pcl::PointCloud<pcl::PointXYZ>());
  {
    std::lock_guard<std::mutex> lock(g_cloud_mutex);
    if (accumulated_cloud_->empty()) {
      RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
      return;
    }
    *registration_cloud = *accumulated_cloud_;
    accumulated_cloud_->clear();
  }

  if (g_two_stage_gicp) {
    if (g_stage == GicpStage::COARSE_RELOC) {
      num_neighbors_ = g_coarse_num_neighbors;
      global_leaf_size_ = g_coarse_global_leaf_size;
      registered_leaf_size_ = g_coarse_registered_leaf_size;
      max_dist_sq_ = g_coarse_max_dist_sq;
    } else {
      num_neighbors_ = g_fine_num_neighbors;
      global_leaf_size_ = g_fine_global_leaf_size;
      registered_leaf_size_ = g_fine_registered_leaf_size;
      max_dist_sq_ = g_fine_max_dist_sq;
    }
  }

  // Rebuild target map only when the current stage requires a different target resolution.
  if (
    std::abs(g_active_target_leaf_size - global_leaf_size_) > 1e-6 ||
    g_active_target_num_neighbors != num_neighbors_)
  {
    target_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, global_leaf_size_);

    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      target_, small_gicp::KdTreeBuilderOMP(num_threads_));

    g_active_target_leaf_size = global_leaf_size_;
    g_active_target_num_neighbors = num_neighbors_;

    RCLCPP_INFO(
      this->get_logger(),
      "Rebuilt target map for stage=%s: target_points=%zu, global_leaf_size=%.3f, neighbors=%d",
      stageName(), target_->size(), global_leaf_size_, num_neighbors_);
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *registration_cloud, registered_leaf_size_);

  if (!source_ || static_cast<int>(source_->size()) < g_min_source_points) {
    RCLCPP_WARN(
      this->get_logger(), "Too few source points after downsampling: %zu < %d",
      source_ ? source_->size() : 0, g_min_source_points);
    return;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations =
    (g_two_stage_gicp && g_stage == GicpStage::FINE_TRACKING) ? 30 : 15;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (!result.converged) {
    g_fail_match_count++;
    g_good_match_count = 0;

    RCLCPP_WARN(
      this->get_logger(), "GICP did not converge. stage=%s, fail_count=%d",
      stageName(), g_fail_match_count);

    if (
      g_two_stage_gicp &&
      g_stage == GicpStage::FINE_TRACKING &&
      g_fail_match_count >= g_switch_back_fail_count)
    {
      g_stage = GicpStage::COARSE_RELOC;
      g_fail_match_count = 0;
      g_fine_filter_ready = false;
      RCLCPP_WARN(this->get_logger(), "GICP switch back to COARSE_RELOC.");
    }

    return;
  }

  const Eigen::Isometry3d candidate_t = result.T_target_source;
  const bool initialpose_recovery = g_after_initialpose_force_left > 0;

  const double dx = candidate_t.translation().x() - previous_result_t_.translation().x();
  const double dy = candidate_t.translation().y() - previous_result_t_.translation().y();
  const double dtrans = std::hypot(dx, dy);
  const double dyaw = normalizeAngle(yawFromIsometry(candidate_t) - yawFromIsometry(previous_result_t_));

  const double accept_trans =
    initialpose_recovery ? g_initialpose_accept_trans :
    ((!g_two_stage_gicp || g_stage == GicpStage::COARSE_RELOC) ?
    g_coarse_accept_trans : g_fine_accept_trans);
  const double accept_yaw =
    initialpose_recovery ? g_initialpose_accept_yaw :
    ((!g_two_stage_gicp || g_stage == GicpStage::COARSE_RELOC) ?
    g_coarse_accept_yaw : g_fine_accept_yaw);

  if (dtrans > accept_trans || std::abs(dyaw) > accept_yaw) {
    g_fail_match_count++;
    g_good_match_count = 0;

    RCLCPP_WARN(
      this->get_logger(),
      "Reject GICP correction. stage=%s, dtrans=%.3f m, dyaw=%.2f deg, "
      "limit=(%.3f m, %.2f deg), fail_count=%d",
      stageName(), dtrans, dyaw * 180.0 / kPi,
      accept_trans, accept_yaw * 180.0 / kPi, g_fail_match_count);

    if (
      g_two_stage_gicp &&
      g_stage == GicpStage::FINE_TRACKING &&
      g_fail_match_count >= g_switch_back_fail_count)
    {
      g_stage = GicpStage::COARSE_RELOC;
      g_fail_match_count = 0;
      g_fine_filter_ready = false;
      RCLCPP_WARN(this->get_logger(), "GICP correction rejected repeatedly, switch back to COARSE_RELOC.");
    }

    return;
  }

  g_fail_match_count = 0;
  g_good_match_count++;

  const double apply_ratio =
    initialpose_recovery ? g_initialpose_apply_ratio :
    ((!g_two_stage_gicp || g_stage == GicpStage::COARSE_RELOC) ?
    g_coarse_apply_ratio : g_fine_apply_ratio);

  if (
    g_stage == GicpStage::FINE_TRACKING &&
    g_update_xy_yaw_only &&
    g_fine_kalman_enable)
  {
    // The Kalman filter keeps every small correction. It suppresses alternating
    // registration noise but still converges when millimetre-level corrections
    // persist in the same direction. The final clamp only limits one published
    // output step; it does not discard the underlying measurement.
    const Eigen::Isometry3d filtered_candidate = filterFineTransform(
      candidate_t, previous_result_t_.translation().z());

    result_t_ = applyPlanarCorrectionClamped(
      previous_result_t_, filtered_candidate, 1.0,
      g_fine_max_step_trans, g_fine_max_step_yaw);
  } else if (g_stage == GicpStage::FINE_TRACKING && g_update_xy_yaw_only) {
    result_t_ = applyPlanarCorrectionClamped(
      previous_result_t_, candidate_t, apply_ratio,
      g_fine_max_step_trans, g_fine_max_step_yaw);
  } else {
    result_t_ = interpolateTransform(
      previous_result_t_, candidate_t, apply_ratio, g_update_xy_yaw_only);
  }
  previous_result_t_ = result_t_;

  RCLCPP_INFO(
    this->get_logger(),
    "Accept GICP. stage=%s, initialpose_recovery=%s, force_left=%d, good_count=%d, "
    "dtrans=%.3f m, dyaw=%.2f deg, apply_ratio=%.2f, max_dist_sq=%.3f, source_points=%zu",
    stageName(), initialpose_recovery ? "true" : "false", g_after_initialpose_force_left,
    g_good_match_count, dtrans, dyaw * 180.0 / kPi,
    apply_ratio, max_dist_sq_, source_->size());

  if (g_after_initialpose_force_left > 0) {
    g_after_initialpose_force_left--;
  }

  if (
    g_two_stage_gicp &&
    g_stage == GicpStage::COARSE_RELOC &&
    g_after_initialpose_force_left <= 0 &&
    g_good_match_count >= g_switch_to_fine_count)
  {
    g_stage = GicpStage::FINE_TRACKING;
    g_good_match_count = 0;
    g_fail_match_count = 0;
    resetFineKalmanFilter(result_t_);
    RCLCPP_INFO(this->get_logger(), "GICP switch to FINE_TRACKING. Fine Kalman filter reset.");
  }

}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]",
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  // The TF published by this node is map_frame_ -> odom_frame_. Therefore the
  // initial pose must also reset map -> odom. The old implementation used
  // current_scan_frame_id_, which may become "map" or another filtered cloud
  // frame after field_cloud_filter, causing the first manual RViz reset to
  // compute the wrong transform.
  std::vector<std::string> source_frame_candidates;
  if (!odom_frame_.empty()) {
    source_frame_candidates.push_back(odom_frame_);
  }
  if (!current_scan_frame_id_.empty() && current_scan_frame_id_ != odom_frame_) {
    source_frame_candidates.push_back(current_scan_frame_id_);
  }

  for (const auto & source_frame : source_frame_candidates) {
    try {
      auto transform =
        tf_buffer_->lookupTransform(robot_base_frame_, source_frame, tf2::TimePointZero);
      Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
      Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

      previous_result_t_ = result_t_ = map_to_odom;
      g_fine_filter_ready = false;

      // Discard scans accumulated before the manual reset. Those scans were
      // accumulated under the old pose and can poison the first GICP match.
      if (accumulated_cloud_) {
        std::lock_guard<std::mutex> lock(g_cloud_mutex);
        accumulated_cloud_->clear();
      }

      if (g_two_stage_gicp) {
        g_stage = GicpStage::COARSE_RELOC;
        g_good_match_count = 0;
        g_fail_match_count = 0;
        g_after_initialpose_force_left = g_after_initialpose_force_coarse_count;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Initial pose applied with source_frame=%s. Reset stage=%s, force_coarse_left=%d.",
        source_frame.c_str(), stageName(), g_after_initialpose_force_left);
      return;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not transform initial pose from %s to %s: %s",
        robot_base_frame_.c_str(), source_frame.c_str(), ex.what());
    }
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "Initial pose was ignored because no valid transform was found. robot_base_frame=%s, "
    "odom_frame=%s, current_scan_frame_id=%s",
    robot_base_frame_.c_str(), odom_frame_.c_str(), current_scan_frame_id_.c_str());
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
