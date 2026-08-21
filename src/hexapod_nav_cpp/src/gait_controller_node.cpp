/*
 * gait_controller_node.cpp
 *
 * Tripod gait state machine + cubic Bezier swing arc generator.
 * C++ port of hexapod_nav/gait_controller_node.py
 * Publishes Float64MultiArray to /spooder_controller/commands
 */

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hexapod_nav_cpp/kinematics.hpp"

using namespace hexapod_nav_cpp;

static constexpr int NUM_LEGS = 6;
static constexpr double CMD_VEL_TIMEOUT = 0.5;

// Tripod groups
static constexpr std::array<int, NUM_LEGS> TRIPOD_A = {0, 2, 4};
static constexpr std::array<int, NUM_LEGS> TRIPOD_B = {1, 3, 5};

// Per-leg visualization colors (RGBA)
static constexpr std::array<std::array<float, 4>, NUM_LEGS> LEG_COLORS = {{
    {1.0f, 0.2f, 0.2f, 0.85f},  // RF red
    {1.0f, 0.6f, 0.1f, 0.85f},  // RM orange
    {1.0f, 1.0f, 0.2f, 0.85f},  // RR yellow
    {0.2f, 1.0f, 0.3f, 0.85f},  // LF green
    {0.2f, 0.6f, 1.0f, 0.85f},  // LM blue
    {0.7f, 0.3f, 1.0f, 0.85f},  // LR purple
}};

// Cubic Bezier swing trajectory: t in [0, 1]
inline Vec3 bezier_arc(const Vec3 &p_start, const Vec3 &p_end, double height,
                       double t) {
  double u = 1.0 - t;
  Vec3 p1 = p_start + Vec3{0, 0, height};
  Vec3 p2 = p_end + Vec3{0, 0, height};

  Vec3 result;
  result.x = u * u * u * p_start.x + 3 * u * u * t * p1.x +
             3 * u * t * t * p2.x + t * t * t * p_end.x;
  result.y = u * u * u * p_start.y + 3 * u * u * t * p1.y +
             3 * u * t * t * p2.y + t * t * t * p_end.y;
  result.z = u * u * u * p_start.z + 3 * u * u * t * p1.z +
             3 * u * t * t * p2.z + t * t * t * p_end.z;
  return result;
}

class GaitControllerNode : public rclcpp::Node {
public:
  GaitControllerNode() : Node("gait_controller_node") {
    declare_parameter("swing_duration", 0.5);
    declare_parameter("max_swing_height", 0.06);
    declare_parameter("step_frequency", 20.0);
    declare_parameter("nominal_stance_height", -0.12);
    declare_parameter("nominal_stance_forward", 0.12);

    swing_dur_ = get_parameter("swing_duration").as_double();
    max_height_ = get_parameter("max_swing_height").as_double();
    freq_ = get_parameter("step_frequency").as_double();
    default_z_ = get_parameter("nominal_stance_height").as_double();
    default_x_ = get_parameter("nominal_stance_forward").as_double();

    for (auto &fp : foot_positions_) {
      fp = {default_x_, 0.0, default_z_};
    }

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_vel_ = {msg->linear.x, msg->linear.y, msg->angular.z};
          last_cmd_stamp_ = now();
        });

    sub_terrain_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "/terrain_grid_map", 10,
        [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
          terrain_grid_ = msg;
        });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          body_x_ = msg->pose.pose.position.x;
          body_y_ = msg->pose.pose.position.y;
          body_z_ = msg->pose.pose.position.z;
          auto &q = msg->pose.pose.orientation;
          body_yaw_ = 2.0 * std::atan2(q.z, q.w);
        });

    // Per-leg foothold target + replan subscriptions
    for (int i = 0; i < NUM_LEGS; ++i) {
      auto target_cb = [this, i](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        Vec3 world = {msg->point.x, msg->point.y, msg->point.z};
        std::array<double, 4> bp = {body_x_, body_y_, body_z_, body_yaw_};
        swing_targets_[i] = world_to_coxa(world, i, bp);
      };
      auto replan_cb = [this, i](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        Vec3 world = {msg->point.x, msg->point.y, msg->point.z};
        std::array<double, 4> bp = {body_x_, body_y_, body_z_, body_yaw_};
        swing_targets_[i] = world_to_coxa(world, i, bp);
        RCLCPP_INFO(get_logger(), "Leg %d: mid-swing replan", i);
      };
      sub_target_[i] = create_subscription<geometry_msgs::msg::PointStamped>(
          "/leg_" + std::to_string(i) + "/foothold_target", 10, target_cb);
      sub_replan_[i] = create_subscription<geometry_msgs::msg::PointStamped>(
          "/leg_" + std::to_string(i) + "/foothold_replan", 10, replan_cb);
    }

    pub_joint_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/spooder_controller/commands", 10);
    pub_phase_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/leg_phase",
                                                                    10);
    pub_markers_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/gait_foot_markers", 10);

    timer_ = create_timer(std::chrono::duration<double>(1.0 / freq_),
                          [this]() { control_loop(); });

    RCLCPP_INFO(get_logger(), "Gait controller node started");
  }

private:
  void control_loop() {
    double dt = 1.0 / freq_;
    double vel_age =
        (now() - last_cmd_stamp_).seconds();
    if (vel_age > CMD_VEL_TIMEOUT) {
      cmd_vel_ = {0.0, 0.0, 0.0};
    }

    double vx = cmd_vel_[0], vy = cmd_vel_[1], yaw_rate = cmd_vel_[2];
    bool moving = std::abs(vx) >= 0.001 || std::abs(vy) >= 0.001 ||
                  std::abs(yaw_rate) >= 0.01;

    if (!moving) {
      solve_and_publish();
      publish_phase(false);
      publish_markers();
      return;
    }

    auto &swinging =
        (active_tripod_ == 0) ? TRIPOD_A : TRIPOD_B;
    auto &stance =
        (active_tripod_ == 0) ? TRIPOD_B : TRIPOD_A;

    // Advance swing progress
    bool all_done = true;
    for (int leg : swinging) {
      swing_progress_[leg] =
          std::min(1.0, swing_progress_[leg] + dt / swing_dur_);
      if (swing_progress_[leg] < 1.0) all_done = false;
    }

    // Advance tripod phase
    if (all_done) {
      for (int leg : swinging) {
        swing_progress_[leg] = 0.0;
        swing_targets_[leg] = std::nullopt;
      }
      active_tripod_ = 1 - active_tripod_;
    }

    auto &new_swinging =
        (active_tripod_ == 0) ? TRIPOD_A : TRIPOD_B;
    auto &new_stance =
        (active_tripod_ == 0) ? TRIPOD_B : TRIPOD_A;

    // Swing: Bezier arc
    for (int leg : new_swinging) {
      double t = swing_progress_[leg];
      Vec3 target;
      if (swing_targets_[leg].has_value()) {
        target = *swing_targets_[leg];
      } else {
        target = foot_positions_[leg];
        target[0] += vx * swing_dur_;
        target[1] += vy * swing_dur_;
        if (std::abs(yaw_rate) > 0.01) {
          double dtheta = yaw_rate * swing_dur_;
          double c = std::cos(dtheta), s = std::sin(dtheta);
          double x = target.x, y = target.y;
          target.x = c * x - s * y;
          target.y = s * x + c * y;
        }
      }
      std::array<double, 4> bp = {body_x_, body_y_, body_z_, body_yaw_};
      Vec3 world_guess = coxa_to_world(target, leg, bp);
      double arc_h = std::min(max_height_,
                              get_ceiling_clearance(world_guess) * 0.6);
      foot_positions_[leg] =
          bezier_arc(foot_positions_[leg], target, arc_h, t);
    }

    // Stance: push feet opposite to body motion
    for (int leg : new_stance) {
      foot_positions_[leg].x -= vx * dt;
      foot_positions_[leg].y -= vy * dt;
      if (std::abs(yaw_rate) > 0.01) {
        double dtheta = -yaw_rate * dt;
        double c = std::cos(dtheta), s = std::sin(dtheta);
        double x = foot_positions_[leg].x, y = foot_positions_[leg].y;
        foot_positions_[leg].x = c * x - s * y;
        foot_positions_[leg].y = s * x + c * y;
      }
    }

    solve_and_publish();
    publish_phase(true);
    publish_markers();
  }

  double get_ceiling_clearance(const Vec3 &world_pos) {
    if (!terrain_grid_) return max_height_;

    int idx = -1;
    for (size_t i = 0; i < terrain_grid_->layers.size(); ++i) {
      if (terrain_grid_->layers[i] == "clearance") {
        idx = static_cast<int>(i);
        break;
      }
    }
    if (idx < 0) return max_height_;

    const auto &data = terrain_grid_->data[idx];
    if (data.layout.dim.empty()) return max_height_;
    int n = data.layout.dim[0].size;
    double res = terrain_grid_->info.resolution;
    double cx = terrain_grid_->info.pose.position.x;
    double cy = terrain_grid_->info.pose.position.y;
    double half_len = terrain_grid_->info.length_x / 2.0;

    int ix = static_cast<int>((world_pos.x - (cx - half_len)) / res);
    int iy = static_cast<int>((world_pos.y - (cy - half_len)) / res);

    if (ix >= 0 && ix < n && iy >= 0 && iy < n) {
      float val = data.data[ix * n + iy];
      if (std::isfinite(val) && val > 0) return static_cast<double>(val);
    }
    return max_height_;
  }

  void solve_and_publish() {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(NUM_LEGS * 3);

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
      auto ik = ik_coxa(foot_positions_[leg].x, foot_positions_[leg].y,
                        foot_positions_[leg].z);
      if (!ik.valid) {
        ik = ik_coxa(default_x_, 0.0, default_z_);
      }
      auto j = clamp_joints(ik.q1, ik.q2, ik.q3);
      msg.data.push_back(j.q_coxa);
      msg.data.push_back(j.q_femur);
      msg.data.push_back(j.q_tibia);
    }
    pub_joint_->publish(msg);
  }

  void publish_phase(bool moving) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(NUM_LEGS, 0);
    if (moving) {
      auto &swinging =
          (active_tripod_ == 0) ? TRIPOD_A : TRIPOD_B;
      for (int leg : swinging) msg.data[leg] = 1;
    }
    pub_phase_->publish(msg);
  }

  void publish_markers() {
    auto ma = std::make_shared<visualization_msgs::msg::MarkerArray>();
    auto stamp = now();

    // Delete-all
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = stamp;
    clear.ns = "gait_feet";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma->markers.push_back(clear);

    std::array<double, 4> bp = {body_x_, body_y_, body_z_, body_yaw_};
    auto &swinging =
        (active_tripod_ == 0) ? TRIPOD_A : TRIPOD_B;
    bool is_moving = std::abs(cmd_vel_[0]) > 0.001 ||
                     std::abs(cmd_vel_[1]) > 0.001 ||
                     std::abs(cmd_vel_[2]) > 0.01;
    std::set<int> swing_set(swinging.begin(), swinging.end());

    for (int i = 0; i < NUM_LEGS; ++i) {
      Vec3 world = coxa_to_world(foot_positions_[i], i, bp);
      auto &c = LEG_COLORS[i];

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = "gait_feet";
      m.id = i + 1;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = world.x;
      m.pose.position.y = world.y;
      m.pose.position.z = world.z + 0.01;
      m.pose.orientation.w = 1.0;
      bool tall = swing_set.count(i) && is_moving;
      m.scale.x = m.scale.y = 0.018;
      m.scale.z = tall ? 0.04 : 0.02;
      m.color.r = c[0];
      m.color.g = c[1];
      m.color.b = c[2];
      m.color.a = c[3];
      ma->markers.push_back(m);
    }
    pub_markers_->publish(ma);
  }

  // Parameters
  double swing_dur_, max_height_, freq_, default_z_, default_x_;

  // State
  std::array<Vec3, NUM_LEGS> foot_positions_;
  std::array<std::optional<Vec3>, NUM_LEGS> swing_targets_;
  std::array<double, NUM_LEGS> swing_progress_{};
  int active_tripod_ = 0;
  std::array<double, 3> cmd_vel_{0, 0, 0};
  rclcpp::Time last_cmd_stamp_{0};
  double body_x_ = 0, body_y_ = 0, body_z_ = 0, body_yaw_ = 0;
  grid_map_msgs::msg::GridMap::SharedPtr terrain_grid_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_terrain_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr,
             NUM_LEGS>
      sub_target_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr,
             NUM_LEGS>
      sub_replan_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joint_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_phase_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitControllerNode>());
  rclcpp::shutdown();
  return 0;
}
