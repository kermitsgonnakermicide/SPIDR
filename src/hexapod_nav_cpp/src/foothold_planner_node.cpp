/*
 * foothold_planner_node.cpp
 *
 * Per-leg foothold selection + mid-swing replanning.
 * C++ port of hexapod_nav/foothold_planner_node.py
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hexapod_nav_cpp/grid_map_utils.hpp"
#include "hexapod_nav_cpp/kinematics.hpp"

using namespace hexapod_nav_cpp;

static constexpr int NUM_LEGS = 6;
static constexpr int STANCE = 0;
static constexpr int SWING = 1;

static constexpr std::array<std::array<float, 4>, NUM_LEGS> LEG_COLORS = {{
    {1.0f, 0.2f, 0.2f, 0.95f},
    {1.0f, 0.6f, 0.1f, 0.95f},
    {1.0f, 1.0f, 0.2f, 0.95f},
    {0.2f, 1.0f, 0.3f, 0.95f},
    {0.2f, 0.6f, 1.0f, 0.95f},
    {0.7f, 0.3f, 1.0f, 0.95f},
}};
static constexpr std::array<const char *, 6> LEG_NAMES = {
    "RF", "RM", "RR", "LF", "LM", "LR"};

class FootholdPlannerNode : public rclcpp::Node {
public:
  FootholdPlannerNode() : Node("foothold_planner_node") {
    declare_parameter("replan_cost_threshold", 0.3);
    declare_parameter("nominal_stance_height", -0.12);
    declare_parameter("aep_forward_offset", 0.05);
    declare_parameter("pep_backward_offset", 0.05);

    replan_thresh_ = get_parameter("replan_cost_threshold").as_double();
    stance_z_ = get_parameter("nominal_stance_height").as_double();
    aep_offset_ = get_parameter("aep_forward_offset").as_double();
    pep_offset_ = get_parameter("pep_backward_offset").as_double();

    body_pose_ = {0.0, 0.0, 0.12, 0.0};

    for (int i = 0; i < NUM_LEGS; ++i) {
      pub_target_[i] = create_publisher<geometry_msgs::msg::PointStamped>(
          "/leg_" + std::to_string(i) + "/foothold_target", 10);
      pub_replan_[i] = create_publisher<geometry_msgs::msg::PointStamped>(
          "/leg_" + std::to_string(i) + "/foothold_replan", 10);
    }
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/foothold_markers", 10);

    sub_costmap_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain_costmap", 10,
        [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
          costmap_callback(msg);
        });

    sub_terrain_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain_grid_map", 10,
        [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
          terrain_grid_ = msg;
        });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          auto &p = msg->pose.pose.position;
          auto &q = msg->pose.pose.orientation;
          body_pose_ = {p.x, p.y, p.z, 2.0 * std::atan2(q.z, q.w)};
        });

    sub_phase_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/leg_phase", 10,
        [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
          if (msg->data.size() != NUM_LEGS) return;
          for (int i = 0; i < NUM_LEGS; ++i) {
            int new_phase = msg->data[i];
            if (leg_phase_[i] == STANCE && new_phase == SWING) {
              begin_swing(i);
            } else if (leg_phase_[i] == SWING && new_phase == STANCE) {
              end_swing(i);
            }
          }
        });

    timer_markers_ =
        create_timer(std::chrono::milliseconds(100),
                     [this]() { publish_markers(); });

    RCLCPP_INFO(get_logger(), "Foothold planner node started");
  }

private:
  void begin_swing(int leg) {
    leg_phase_[leg] = SWING;
    select_and_publish(leg, false);
  }

  void end_swing(int leg) {
    leg_phase_[leg] = STANCE;
    committed_[leg] = std::nullopt;
    target_points_[leg] = std::nullopt;
    target_is_replan_[leg] = false;
  }

  void costmap_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    std::vector<float> cost_data;
    int n = 0;
    if (!extract_cost_array(*msg, cost_data, n)) return;

    costmap_meta_ = msg->info;
    costmap_ = cost_data;
    costmap_n_ = n;

    // Check committed targets for cost changes
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
      if (leg_phase_[leg] == SWING && committed_[leg].has_value()) {
        auto &[ix, iy, committed_cost] = *committed_[leg];
        float current_cost = get_cost_at(ix, iy, n, cost_data);

        if (current_cost == std::numeric_limits<float>::infinity() ||
            (current_cost - committed_cost) > replan_thresh_) {
          RCLCPP_INFO(get_logger(),
                       "Leg %d: cost delta %.2f > threshold. Replanning.",
                       leg, current_cost - committed_cost);
          select_and_publish(leg, true);
        }
      }
    }
  }

  void select_and_publish(int leg, bool replan) {
    if (costmap_.empty() || costmap_n_ == 0) return;

    int n = costmap_n_;
    double res = costmap_meta_.resolution;
    double cx = costmap_meta_.pose.position.x;
    double cy = costmap_meta_.pose.position.y;
    double half_len_x = costmap_meta_.length_x / 2.0;
    double half_len_y = costmap_meta_.length_y / 2.0;

    // Build candidate positions and floor heights
    std::vector<float> floor_data;
    int floor_n = 0;
    bool has_floor = terrain_grid_ &&
                     extract_layer(*terrain_grid_, "floor", floor_data, floor_n);

    double min_cost = std::numeric_limits<double>::infinity();
    int best_ix = 0, best_iy = 0;
    Vec3 best_pos = {0, 0, stance_z_};

    for (int ix = 0; ix < n; ++ix) {
      for (int iy = 0; iy < n; ++iy) {
        double wx = cx - half_len_x + (ix + 0.5) * res;
        double wy = cy - half_len_y + (iy + 0.5) * res;
        double wz = stance_z_;

        if (has_floor && floor_n == n) {
          float fv = floor_data[ix * n + iy];
          if (std::isfinite(fv)) wz = fv;
        }

        // Reachability check
        bool reachable = true;
        {
          double max_reach = COXA_LEN + FEMUR_LEN + TIBIA_LEN;
          double min_reach =
              std::abs(FEMUR_LEN - TIBIA_LEN) + COXA_LEN * 0.5;

          double cos_yaw = std::cos(-body_pose_[3]);
          double sin_yaw = std::sin(-body_pose_[3]);
          double cos_ang = std::cos(-LEG_ANGLES[leg]);
          double sin_ang = std::sin(-LEG_ANGLES[leg]);

          double dx = wx - body_pose_[0];
          double dy = wy - body_pose_[1];
          double bx = dx * cos_yaw - dy * sin_yaw;
          double by = dx * sin_yaw + dy * cos_yaw;
          double tx = bx - LEG_ORIGINS[leg].x;
          double ty = by - LEG_ORIGINS[leg].y;
          double lx = tx * cos_ang - ty * sin_ang;
          double ly = tx * sin_ang + ty * cos_ang;
          double lz = wz - body_pose_[2];
          double h_dist = std::sqrt(lx * lx + ly * ly);
          if (h_dist < min_reach || h_dist > max_reach) reachable = false;
        }

        if (!reachable) continue;

        // AEP/PEP filter
        {
          double cos_yaw = std::cos(-body_pose_[3]);
          double sin_yaw = std::sin(-body_pose_[3]);
          double cos_ang = std::cos(-LEG_ANGLES[leg]);
          double sin_ang = std::sin(-LEG_ANGLES[leg]);

          double dx = wx - body_pose_[0];
          double dy = wy - body_pose_[1];
          double bx = dx * cos_yaw - dy * sin_yaw;
          double by = dx * sin_yaw + dy * cos_yaw;
          double tx = bx - LEG_ORIGINS[leg].x;
          double ty = by - LEG_ORIGINS[leg].y;
          double coxa_x = tx * cos_ang - ty * sin_ang;

          double pep_limit = COXA_LEN + pep_offset_;
          double aep_limit = COXA_LEN + FEMUR_LEN + TIBIA_LEN - aep_offset_;
          if (coxa_x < pep_limit || coxa_x > aep_limit) continue;
        }

        // Cost check
        float cost = costmap_[ix * n + iy];
        if (cost < min_cost) {
          min_cost = cost;
          best_ix = ix;
          best_iy = iy;
          best_pos = {wx, wy, wz};
        }
      }
    }

    if (min_cost == std::numeric_limits<double>::infinity()) {
      RCLCPP_WARN(get_logger(), "Leg %d: no valid foothold found", leg);
      return;
    }

    committed_[leg] = {best_ix, best_iy, static_cast<float>(min_cost)};

    // Publish target
    auto pt = std::make_unique<geometry_msgs::msg::PointStamped>();
    pt->header.stamp = now();
    pt->header.frame_id = "map";
    pt->point.x = best_pos.x;
    pt->point.y = best_pos.y;
    pt->point.z = best_pos.z;

    target_points_[leg] = best_pos;
    target_is_replan_[leg] = replan;

    if (replan) {
      pub_replan_[leg]->publish(std::move(pt));
    } else {
      pub_target_[leg]->publish(std::move(pt));
    }
    publish_markers();
  }

  float get_cost_at(int ix, int iy, int n, const std::vector<float> &data) {
    if (ix >= 0 && ix < n && iy >= 0 && iy < n) {
      return data[ix * n + iy];
    }
    return std::numeric_limits<float>::infinity();
  }

  void publish_markers() {
    auto ma = std::make_shared<visualization_msgs::msg::MarkerArray>();
    auto stamp = now();

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = stamp;
    clear.ns = "footholds";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma->markers.push_back(clear);

    for (int i = 0; i < NUM_LEGS; ++i) {
      if (!target_points_[i].has_value()) continue;
      auto &pt = *target_points_[i];
      auto &c = LEG_COLORS[i];

      double scale = target_is_replan_[i] ? 0.035 : 0.025;

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = stamp;
      sphere.ns = "footholds";
      sphere.id = i + 1;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = pt.x;
      sphere.pose.position.y = pt.y;
      sphere.pose.position.z = pt.z;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = scale;
      sphere.color.r = c[0];
      sphere.color.g = c[1];
      sphere.color.b = c[2];
      sphere.color.a = c[3];
      ma->markers.push_back(sphere);

      // Line from body to target
      visualization_msgs::msg::Marker line;
      line.header.frame_id = "map";
      line.header.stamp = stamp;
      line.ns = "foothold_rays";
      line.id = i + 1;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.008;
      line.color.r = c[0];
      line.color.g = c[1];
      line.color.b = c[2];
      line.color.a = 0.6f;
      geometry_msgs::msg::Point p1, p2;
      p1.x = body_pose_[0];
      p1.y = body_pose_[1];
      p1.z = body_pose_[2];
      p2.x = pt.x;
      p2.y = pt.y;
      p2.z = pt.z;
      line.points = {p1, p2};
      ma->markers.push_back(line);

      // Text label
      visualization_msgs::msg::Marker text;
      text.header.frame_id = "map";
      text.header.stamp = stamp;
      text.ns = "foothold_labels";
      text.id = i + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = pt.x;
      text.pose.position.y = pt.y;
      text.pose.position.z = pt.z + 0.04;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.03;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 0.9f;
      text.text = std::string(LEG_NAMES[i]) +
                  (target_is_replan_[i] ? " REPLAN" : " TGT");
      ma->markers.push_back(text);
    }
    pub_markers_->publish(ma);
  }

  // Parameters
  double replan_thresh_, stance_z_, aep_offset_, pep_offset_;

  // State
  std::array<int, NUM_LEGS> leg_phase_{};
  std::array<std::optional<std::tuple<int, int, float>>, NUM_LEGS> committed_;
  std::array<std::optional<Vec3>, NUM_LEGS> target_points_;
  std::array<bool, NUM_LEGS> target_is_replan_{};
  std::array<double, 4> body_pose_;
  std::vector<float> costmap_;
  int costmap_n_ = 0;
  grid_map_msgs::msg::Info costmap_meta_;
  grid_map_msgs::msg::GridMap::SharedPtr terrain_grid_;

  // Publishers
  std::array<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr,
             NUM_LEGS>
      pub_target_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr,
             NUM_LEGS>
      pub_replan_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // Subscriptions
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_costmap_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_terrain_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_phase_;
  rclcpp::TimerBase::SharedPtr timer_markers_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootholdPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
