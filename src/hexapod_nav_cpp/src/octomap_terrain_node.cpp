/*
 * octomap_terrain_node.cpp
 *
 * Converts OctoMap occupied-voxel point cloud into a dual-layer grid_map:
 *   floor, ceiling, clearance, unknown_above
 * C++ port of hexapod_nav/octomap_terrain_node.py
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hexapod_nav_cpp/grid_map_utils.hpp"

using namespace hexapod_nav_cpp;

class OctomapTerrainNode : public rclcpp::Node {
public:
  OctomapTerrainNode() : Node("octomap_terrain_node") {
    declare_parameter("resolution", 0.05);
    declare_parameter("grid_radius", 3.0);
    declare_parameter("robot_body_height", 0.15);
    declare_parameter("floor_search_height", 0.3);
    declare_parameter("voxel_size", 0.05);
    declare_parameter("base_frame", "spooder/base_footprint");
    declare_parameter("world_frame", "map");

    res_ = get_parameter("resolution").as_double();
    radius_ = get_parameter("grid_radius").as_double();
    body_h_ = get_parameter("robot_body_height").as_double();
    floor_search_ = get_parameter("floor_search_height").as_double();
    voxel_size_ = get_parameter("voxel_size").as_double();
    base_frame_ = get_parameter("base_frame").as_string();
    world_frame_ = get_parameter("world_frame").as_string();

    grid_size_ = static_cast<int>(2 * radius_ / res_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/octomap_point_cloud_centers", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          pointcloud_callback(msg);
        });

    pub_terrain_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "/terrain_grid_map", 10);

    RCLCPP_INFO(get_logger(), "OctoMap terrain node started");
  }

private:
  void update_robot_position() {
    try {
      auto tf = tf_buffer_->lookup_transform(world_frame_, base_frame_,
                                              tf2::TimePointZero);
      robot_pos_[0] = tf.transform.translation.x;
      robot_pos_[1] = tf.transform.translation.y;
      robot_pos_[2] = tf.transform.translation.z;
    } catch (...) {
      // keep last known position
    }
  }

  void pointcloud_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    update_robot_position();

    // Parse point cloud
    int num_points = msg->width * msg->height;
    if (num_points == 0) return;

    sensor_msgs::PointCloud2Iterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(*msg, "z");

    double cx = robot_pos_[0], cy = robot_pos_[1];
    int n = grid_size_;
    double half_r = radius_;

    // Bin points into grid cells: (ix, iy) -> vector of z values
    std::unordered_map<int, std::vector<double>> cell_points;
    cell_points.reserve(num_points);

    for (int i = 0; i < num_points; ++i, ++it_x, ++it_y, ++it_z) {
      double x = *it_x, y = *it_y, z = *it_z;
      int ix = static_cast<int>((x - (cx - half_r)) / res_);
      int iy = static_cast<int>((y - (cy - half_r)) / res_);
      if (ix >= 0 && ix < n && iy >= 0 && iy < n) {
        cell_points[ix * n + iy].push_back(z);
      }
    }

    std::vector<float> floor_grid(n * n, std::numeric_limits<float>::quiet_NaN());
    std::vector<float> ceiling_grid(n * n,
                                     std::numeric_limits<float>::quiet_NaN());
    std::vector<float> unknown_grid(n * n, 1.0f);

    int column_voxels =
        static_cast<int>((body_h_ + 0.1) / voxel_size_);

    for (auto &[key, zvals] : cell_points) {
      std::sort(zvals.begin(), zvals.end());
      double z_min = zvals[0];

      // Floor: cluster near lowest point
      std::vector<double> floor_pts;
      for (double z : zvals) {
        if (z <= z_min + floor_search_) floor_pts.push_back(z);
      }
      double floor_h = z_min;
      if (!floor_pts.empty()) {
        floor_h = 0.0;
        for (double z : floor_pts) floor_h += z;
        floor_h /= floor_pts.size();
      }
      floor_grid[key] = static_cast<float>(floor_h);

      // Ceiling: lowest point above robot body + margin
      double overhead_threshold = floor_h + body_h_ + 0.05;
      double ceiling = std::numeric_limits<float>::quiet_NaN();
      for (double z : zvals) {
        if (z > overhead_threshold) {
          ceiling = static_cast<float>(z);
          break;
        }
      }
      ceiling_grid[key] = ceiling;

      // Unknown above fraction
      double col_top = floor_h + body_h_ + 0.1;
      double col_bot = floor_h + body_h_ + 0.05;
      int overhead_count = 0;
      for (double z : zvals) {
        if (z > col_bot && z <= col_top) ++overhead_count;
      }
      unknown_grid[key] = static_cast<float>(
          std::max(0.0, 1.0 - static_cast<double>(overhead_count) /
                                   std::max(column_voxels, 1)));
    }

    // Clearance = ceiling - floor
    std::vector<float> clearance_grid(n * n,
                                       std::numeric_limits<float>::quiet_NaN());
    for (int i = 0; i < n * n; ++i) {
      if (std::isfinite(floor_grid[i]) && std::isfinite(ceiling_grid[i])) {
        clearance_grid[i] = ceiling_grid[i] - floor_grid[i];
      }
    }

    // Publish GridMap
    grid_map_msgs::msg::GridMap gm;
    gm.header = msg->header;
    gm.info.resolution = res_;
    gm.info.length_x = 2.0 * radius_;
    gm.info.length_y = 2.0 * radius_;
    gm.info.pose.position.x = robot_pos_[0];
    gm.info.pose.position.y = robot_pos_[1];
    gm.info.pose.orientation.w = 1.0;

    gm.layers = {"floor", "ceiling", "clearance", "unknown_above"};
    gm.data.push_back(make_layer(floor_grid.data(), n));
    gm.data.push_back(make_layer(ceiling_grid.data(), n));
    gm.data.push_back(make_layer(clearance_grid.data(), n));
    gm.data.push_back(make_layer(unknown_grid.data(), n));

    pub_terrain_->publish(gm);
  }

  // Parameters
  double res_, radius_, body_h_, floor_search_, voxel_size_;
  std::string base_frame_, world_frame_;
  int grid_size_;
  std::array<double, 3> robot_pos_{0, 0, 0};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions / publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_terrain_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapTerrainNode>());
  rclcpp::shutdown();
  return 0;
}
