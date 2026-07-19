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

#include "pcd2pgm/pcd2pgm.hpp"

#include "pcl/common/transforms.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/passthrough.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcd2pgm
{
Pcd2PgmNode::Pcd2PgmNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

  applyTransform();

  passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
  radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
  setMapTopicMsg(cloud_after_radius_, map_topic_msg_);

  timer_ =
    create_wall_timer(std::chrono::seconds(1), std::bind(&Pcd2PgmNode::publishCallback, this));
}

void Pcd2PgmNode::publishCallback()
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_after_radius_, output);
  output.header.frame_id = "map";
  pcd_publisher_->publish(output);
  map_publisher_->publish(map_topic_msg_);
}

void Pcd2PgmNode::declareParameters()
{
  declare_parameter("pcd_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("map_topic_name", "map");

  // 斜坡/可行区识别参数
  // enable_slope_detection = false 时保持原来的逻辑：滤波后的点全部写成障碍物。
  declare_parameter("enable_slope_detection", true);
  declare_parameter("traversable_slope_angle_deg", 18.0);     // 小于等于该角度的坡面认为可通行
  declare_parameter("max_step_height", 0.15);                  // 相邻栅格高度突变大于该值认为是台阶/障碍
  declare_parameter("obstacle_height_threshold", 0.20);        // 单个栅格内高度跨度过大认为有立体障碍
  declare_parameter("slope_neighbor_radius", 2);               // 用几圈邻居估计局部坡度
  declare_parameter("min_points_per_cell", 2);                 // 单栅格最少点数，过少认为无效/噪点
  declare_parameter("unknown_as_free", true);                  // true: 未观测区域按自由区；false: 未观测区域按未知区

  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // 新增的参数
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("pcd_file", pcd_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);  // 获取新的参数
}

void Pcd2PgmNode::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering: %lu points",
    cloud_after_pass_through_->points.size());
}

void Pcd2PgmNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}

void Pcd2PgmNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  bool enable_slope_detection = true;
  double traversable_slope_angle_deg = 18.0;
  double max_step_height = 0.15;
  double obstacle_height_threshold = 0.20;
  int slope_neighbor_radius = 2;
  int min_points_per_cell = 2;
  bool unknown_as_free = true;

  get_parameter("enable_slope_detection", enable_slope_detection);
  get_parameter("traversable_slope_angle_deg", traversable_slope_angle_deg);
  get_parameter("max_step_height", max_step_height);
  get_parameter("obstacle_height_threshold", obstacle_height_threshold);
  get_parameter("slope_neighbor_radius", slope_neighbor_radius);
  get_parameter("min_points_per_cell", min_points_per_cell);
  get_parameter("unknown_as_free", unknown_as_free);

  slope_neighbor_radius = std::max(1, slope_neighbor_radius);
  min_points_per_cell = std::max(1, min_points_per_cell);
  traversable_slope_angle_deg = std::max(0.0, traversable_slope_angle_deg);
  max_step_height = std::max(0.0, max_step_height);
  obstacle_height_threshold = std::max(0.0, obstacle_height_threshold);

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (!cloud || cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  if (x_min > x_max || y_min > y_max) {
    RCLCPP_WARN(get_logger(), "Point cloud has no finite XYZ points!");
    return;
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = static_cast<unsigned int>(std::ceil((x_max - x_min) / map_resolution_)) + 1U;
  msg.info.height = static_cast<unsigned int>(std::ceil((y_max - y_min) / map_resolution_)) + 1U;
  msg.data.assign(msg.info.width * msg.info.height, unknown_as_free ? 0 : -1);

  // 兼容原始模式：滤波后的点云全部认为是障碍物。
  if (!enable_slope_detection) {
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const int i = static_cast<int>(std::floor((point.x - x_min) / map_resolution_));
      const int j = static_cast<int>(std::floor((point.y - y_min) / map_resolution_));

      if (i >= 0 && i < static_cast<int>(msg.info.width) &&
          j >= 0 && j < static_cast<int>(msg.info.height)) {
        msg.data[i + j * msg.info.width] = 100;
      }
    }

    RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
    return;
  }

  struct CellStats
  {
    int count = 0;
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();
  };

  std::vector<CellStats> cells(msg.info.width * msg.info.height);

  auto cellIndex = [&msg](int i, int j) -> std::size_t {
    return static_cast<std::size_t>(i + j * static_cast<int>(msg.info.width));
  };

  for (const auto & point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const int i = static_cast<int>(std::floor((point.x - x_min) / map_resolution_));
    const int j = static_cast<int>(std::floor((point.y - y_min) / map_resolution_));

    if (i < 0 || i >= static_cast<int>(msg.info.width) ||
        j < 0 || j >= static_cast<int>(msg.info.height)) {
      continue;
    }

    auto & cell = cells[cellIndex(i, j)];
    cell.count++;
    cell.z_min = std::min(cell.z_min, static_cast<double>(point.z));
    cell.z_max = std::max(cell.z_max, static_cast<double>(point.z));
  }

  constexpr double kPi = 3.14159265358979323846;
  const double slope_limit_rad = traversable_slope_angle_deg * kPi / 180.0;
  std::size_t free_count = 0;
  std::size_t occupied_count = 0;

  for (int j = 0; j < static_cast<int>(msg.info.height); ++j) {
    for (int i = 0; i < static_cast<int>(msg.info.width); ++i) {
      const auto & cell = cells[cellIndex(i, j)];
      if (cell.count < min_points_per_cell) {
        continue;
      }

      const double vertical_span = cell.z_max - cell.z_min;
      double max_neighbor_slope_rad = 0.0;
      double max_neighbor_step = 0.0;
      bool has_valid_neighbor = false;

      for (int dy = -slope_neighbor_radius; dy <= slope_neighbor_radius; ++dy) {
        for (int dx = -slope_neighbor_radius; dx <= slope_neighbor_radius; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }

          const int ni = i + dx;
          const int nj = j + dy;
          if (ni < 0 || ni >= static_cast<int>(msg.info.width) ||
              nj < 0 || nj >= static_cast<int>(msg.info.height)) {
            continue;
          }

          const auto & nb = cells[cellIndex(ni, nj)];
          if (nb.count < min_points_per_cell) {
            continue;
          }

          has_valid_neighbor = true;
          const double horizontal_dist =
            map_resolution_ * std::sqrt(static_cast<double>(dx * dx + dy * dy));
          const double dz = std::abs(cell.z_min - nb.z_min);
          max_neighbor_step = std::max(max_neighbor_step, dz);
          max_neighbor_slope_rad = std::max(max_neighbor_slope_rad, std::atan2(dz, horizontal_dist));
        }
      }

      // 判断逻辑：
      // 1. 单个栅格内高度跨度过大，说明可能是墙、柱、障碍物侧面，标为障碍；
      // 2. 与邻居高度突变过大，说明是台阶/断崖，标为障碍；
      // 3. 局部坡度角小于阈值，认为是可通行坡面，标为自由区。
      const bool looks_like_vertical_obstacle = vertical_span > obstacle_height_threshold;
      const bool has_too_high_step = has_valid_neighbor && (max_neighbor_step > max_step_height);
      const bool slope_is_traversable = !has_valid_neighbor || (max_neighbor_slope_rad <= slope_limit_rad);

      if (!looks_like_vertical_obstacle && !has_too_high_step && slope_is_traversable) {
        msg.data[cellIndex(i, j)] = 0;
        free_count++;
      } else {
        msg.data[cellIndex(i, j)] = 100;
        occupied_count++;
      }
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Slope map generated: size=%lu, free_cells=%lu, occupied_cells=%lu, slope_limit=%.2f deg, "
    "max_step=%.3f m, obstacle_height=%.3f m",
    msg.data.size(), free_count, occupied_count, traversable_slope_angle_deg, max_step_height,
    obstacle_height_threshold);
}

void Pcd2PgmNode::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmNode)