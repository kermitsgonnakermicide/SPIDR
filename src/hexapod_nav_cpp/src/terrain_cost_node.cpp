/*
 * terrain_cost_node.cpp
 *
 * Computes foothold cost per grid cell from terrain properties:
 * slope, roughness, clearance, unknown overhead, friction.
 * C++ port of hexapod_nav/terrain_cost_node.py
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hexapod_nav_cpp/grid_map_utils.hpp"

using namespace hexapod_nav_cpp;

class TerrainCostNode : public rclcpp::Node {
public:
  TerrainCostNode() : Node("terrain_cost_node") {
    declare_parameter("w_slope", 1.0);
    declare_parameter("w_roughness", 0.8);
    declare_parameter("w_unknown", 0.5);
    declare_parameter("w_friction", 0.6);
    declare_parameter("surface_friction_coefficient", 0.8);
    declare_parameter("min_swing_height", 0.12);
    declare_parameter("strict_unknown", true);

    w_s_ = get_parameter("w_slope").as_double();
    w_r_ = get_parameter("w_roughness").as_double();
    w_u_ = get_parameter("w_unknown").as_double();
    w_f_ = get_parameter("w_friction").as_double();
    mu_ = get_parameter("surface_friction_coefficient").as_double();
    min_swing_ = get_parameter("min_swing_height").as_double();
    strict_unknown_ = get_parameter("strict_unknown").as_bool();

    sub_grid_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain_grid_map", 10,
        [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
          grid_map_callback(msg);
        });

    pub_cost_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "/terrain_costmap", 10);

    RCLCPP_INFO(get_logger(), "Terrain cost node started");
  }

private:
  void grid_map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    // Extract layers
    std::vector<float> floor_data, ceiling_data, clearance_data, unknown_data;
    int n = 0;
    if (!extract_layer(*msg, "floor", floor_data, n) || n == 0) return;
    extract_layer(*msg, "ceiling", ceiling_data, n);
    extract_layer(*msg, "clearance", clearance_data, n);
    extract_layer(*msg, "unknown_above", unknown_data, n);

    double res = msg->info.resolution;
    std::vector<float> cost(n * n, 0.0f);

    // Compute gradient-based slope using central differences
    std::vector<float> slope(n * n, 0.0f);
    float max_slope = 1e-6f;
    for (int ix = 1; ix < n - 1; ++ix) {
      for (int iy = 1; iy < n - 1; ++iy) {
        float fl = std::isfinite(floor_data[(ix - 1) * n + iy])
                       ? floor_data[(ix - 1) * n + iy]
                       : 0.0f;
        float fr = std::isfinite(floor_data[(ix + 1) * n + iy])
                       ? floor_data[(ix + 1) * n + iy]
                       : 0.0f;
        float fb = std::isfinite(floor_data[ix * n + (iy - 1)])
                       ? floor_data[ix * n + (iy - 1)]
                       : 0.0f;
        float ft = std::isfinite(floor_data[ix * n + (iy + 1)])
                       ? floor_data[ix * n + (iy + 1)]
                       : 0.0f;
        float gx = (fr - fl) / (2.0f * static_cast<float>(res));
        float gy = (ft - fb) / (2.0f * static_cast<float>(res));
        float s = std::sqrt(gx * gx + gy * gy);
        slope[ix * n + iy] = s;
        if (s > max_slope) max_slope = s;
      }
    }
    // Normalize and apply slope cost
    for (int i = 0; i < n * n; ++i) {
      cost[i] += static_cast<float>(w_s_) * (slope[i] / max_slope);
    }

    // Local variance (roughness) using 3x3 kernel
    std::vector<float> roughness(n * n, 0.0f);
    float max_rough = 1e-6f;
    for (int ix = 1; ix < n - 1; ++ix) {
      for (int iy = 1; iy < n - 1; ++iy) {
        float vals[9];
        int vi = 0;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            float v = floor_data[(ix + dx) * n + (iy + dy)];
            vals[vi++] = std::isfinite(v) ? v : 0.0f;
          }
        }
        float mean = std::accumulate(vals, vals + 9, 0.0f) / 9.0f;
        float var = 0;
        for (int k = 0; k < 9; ++k) {
          var += (vals[k] - mean) * (vals[k] - mean);
        }
        var /= 9.0f;
        roughness[ix * n + iy] = var;
        if (var > max_rough) max_rough = var;
      }
    }
    for (int i = 0; i < n * n; ++i) {
      cost[i] += static_cast<float>(w_r_) * (roughness[i] / max_rough);
    }

    // Friction cost: 1 - mu * cos(slope_angle)
    for (int i = 0; i < n * n; ++i) {
      double slope_angle = std::atan(static_cast<double>(slope[i]));
      double friction_factor = mu_ * std::cos(slope_angle);
      cost[i] += static_cast<float>(w_f_ * (1.0 - friction_factor));
    }

    // Clearance constraint
    if (!clearance_data.empty()) {
      for (int i = 0; i < n * n; ++i) {
        if (clearance_data[i] < static_cast<float>(min_swing_)) {
          cost[i] = std::numeric_limits<float>::infinity();
        }
      }
    }

    // Unknown overhead penalty
    if (!unknown_data.empty()) {
      if (strict_unknown_) {
        for (int i = 0; i < n * n; ++i) {
          if (unknown_data[i] > 0.0f) {
            cost[i] = std::numeric_limits<float>::infinity();
          }
        }
      } else {
        for (int i = 0; i < n * n; ++i) {
          cost[i] += static_cast<float>(w_u_) * unknown_data[i];
        }
      }
    }

    // NaN floor = infinite cost
    for (int i = 0; i < n * n; ++i) {
      if (!std::isfinite(floor_data[i])) {
        cost[i] = std::numeric_limits<float>::infinity();
      }
    }

    // Publish
    grid_map_msgs::msg::GridMap out;
    out.header = msg->header;
    out.info = msg->info;
    out.layers = {"cost"};
    out.data.push_back(make_layer(cost.data(), n));
    pub_cost_->publish(out);
  }

  // Parameters
  double w_s_, w_r_, w_u_, w_f_;
  double mu_, min_swing_;
  bool strict_unknown_;

  // Subscriptions / publishers
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_grid_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_cost_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainCostNode>());
  rclcpp::shutdown();
  return 0;
}
