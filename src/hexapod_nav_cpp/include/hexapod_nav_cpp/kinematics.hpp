#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace hexapod_nav_cpp {

// Diddler link lengths (meters)
constexpr double COXA_LEN = 0.043;
constexpr double FEMUR_LEN = 0.060;
constexpr double TIBIA_LEN = 0.104;

// Joint limits (radians)
constexpr double COXA_MIN = -0.7;
constexpr double COXA_MAX = 0.7;
constexpr double FEMUR_MIN = -1.5;
constexpr double FEMUR_MAX = 1.5;
constexpr double TIBIA_MIN = -2.5;
constexpr double TIBIA_MAX = 0.5;

// Leg mount positions relative to base_link (x, y, z)
struct Vec3 {
  double x, y, z;
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

// Leg origins in body frame (from URDF xacro)
constexpr std::array<Vec3, 6> LEG_ORIGINS = {{
    {0.0835, -0.063, 0.0},   // 0: rf
    {0.0, -0.063, 0.0},      // 1: rm
    {-0.0835, -0.063, 0.0},  // 2: rr
    {0.0835, 0.063, 0.0},    // 3: lf
    {0.0, 0.063, 0.0},       // 4: lm
    {-0.0835, 0.063, 0.0},   // 5: lr
}};

// Leg mount yaw angles (radians, from URDF)
constexpr std::array<double, 6> LEG_ANGLES = {{
    -0.7853,  // rf: -45 deg
    -1.5708,  // rm: -90 deg
    -2.3561,  // rr: -135 deg
     0.7853,  // lf: +45 deg
     1.5708,  // lm: +90 deg
     2.3561,  // lr: +135 deg
}};

struct IKResult {
  double q1, q2, q3;
  bool valid;
};

struct JointAngles {
  double q_coxa, q_femur, q_tibia;
};

// 3-DOF inverse kinematics in coxa frame (matches Python ik_coxa exactly)
// x = forward along coxa, y = lateral, z = up
// Returns joint angles (coxa, femur, tibia) or (0,0,0) if unreachable
inline IKResult ik_coxa(double x, double y, double z) {
  double q1 = std::atan2(y, x);

  double h_dist = std::sqrt(x * x + y * y) - COXA_LEN;
  double L = std::sqrt(h_dist * h_dist + z * z);

  double L_sum = FEMUR_LEN + TIBIA_LEN;
  double L_diff = std::abs(FEMUR_LEN - TIBIA_LEN);
  if (L > L_sum || L < L_diff) {
    return {0.0, 0.0, 0.0, false};
  }

  double cos_alpha = (FEMUR_LEN * FEMUR_LEN + L * L - TIBIA_LEN * TIBIA_LEN) /
                     (2.0 * FEMUR_LEN * L);
  cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
  double alpha = std::acos(cos_alpha);

  double beta = std::atan2(z, h_dist);
  double q2 = beta + alpha;

  double cos_gamma = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - L * L) /
                     (2.0 * FEMUR_LEN * TIBIA_LEN);
  cos_gamma = std::clamp(cos_gamma, -1.0, 1.0);
  double gamma = std::acos(cos_gamma);
  double q3 = gamma - M_PI;

  return {q1, q2, q3, true};
}

// Clamp joint angles to limits
inline JointAngles clamp_joints(double q1, double q2, double q3) {
  return {
      std::clamp(q1, COXA_MIN, COXA_MAX),
      std::clamp(q2, FEMUR_MIN, FEMUR_MAX),
      std::clamp(q3, TIBIA_MIN, TIBIA_MAX),
  };
}

// 2D rotation
inline void rotate2d(double &x, double &y, double angle) {
  double c = std::cos(angle), s = std::sin(angle);
  double nx = c * x - s * y;
  double ny = s * x + c * y;
  x = nx;
  y = ny;
}

// Transform world point to coxa frame
// body_pose = {x, y, z, yaw}
inline Vec3 world_to_coxa(const Vec3 &world, int leg,
                          const std::array<double, 4> &body_pose) {
  double dx = world.x - body_pose[0];
  double dy = world.y - body_pose[1];

  // World -> body frame (rotate by -yaw)
  double bx = dx * std::cos(-body_pose[3]) - dy * std::sin(-body_pose[3]);
  double by = dx * std::sin(-body_pose[3]) + dy * std::cos(-body_pose[3]);
  double bz = world.z - body_pose[2];

  // Body -> leg origin offset
  double tx = bx - LEG_ORIGINS[leg].x;
  double ty = by - LEG_ORIGINS[leg].y;

  // Leg yaw rotation
  double cx = tx * std::cos(-LEG_ANGLES[leg]) - ty * std::sin(-LEG_ANGLES[leg]);
  double cy = tx * std::sin(-LEG_ANGLES[leg]) + ty * std::cos(-LEG_ANGLES[leg]);

  return {cx, cy, bz};
}

// Transform coxa frame point to world frame
inline Vec3 coxa_to_world(const Vec3 &coxa, int leg,
                          const std::array<double, 4> &body_pose) {
  // Leg yaw rotation (inverse)
  double tx = coxa.x * std::cos(LEG_ANGLES[leg]) - coxa.y * std::sin(LEG_ANGLES[leg]);
  double ty = coxa.x * std::sin(LEG_ANGLES[leg]) + coxa.y * std::cos(LEG_ANGLES[leg]);

  // Add leg origin
  double bx = tx + LEG_ORIGINS[leg].x;
  double by = ty + LEG_ORIGINS[leg].y;

  // Body frame -> world (rotate by yaw)
  double wx = bx * std::cos(body_pose[3]) - by * std::sin(body_pose[3]);
  double wy = bx * std::sin(body_pose[3]) + by * std::cos(body_pose[3]);

  return {wx + body_pose[0], wy + body_pose[1], coxa.z + body_pose[2]};
}

// Check if a set of world-frame candidate positions is reachable by a leg
// body_pose = {x, y, z, yaw}
inline void get_reachable_zone(int leg,
                               const std::array<double, 4> &body_pose,
                               const double *cx, const double *cy,
                               const double *cz, size_t count,
                               bool *reachable) {
  double max_reach = COXA_LEN + FEMUR_LEN + TIBIA_LEN;
  double min_reach = std::abs(FEMUR_LEN - TIBIA_LEN) + COXA_LEN * 0.5;

  double cos_yaw = std::cos(-body_pose[3]);
  double sin_yaw = std::sin(-body_pose[3]);
  double cos_ang = std::cos(-LEG_ANGLES[leg]);
  double sin_ang = std::sin(-LEG_ANGLES[leg]);

  for (size_t i = 0; i < count; ++i) {
    double dx = cx[i] - body_pose[0];
    double dy = cy[i] - body_pose[1];

    // World -> body
    double bx = dx * cos_yaw - dy * sin_yaw;
    double by = dx * sin_yaw + dy * cos_yaw;

    // Body -> leg
    double tx = bx - LEG_ORIGINS[leg].x;
    double ty = by - LEG_ORIGINS[leg].y;

    double lx = tx * cos_ang - ty * sin_ang;
    double ly = tx * sin_ang + ty * cos_ang;
    double lz = cz[i] - body_pose[2];

    // Horizontal reach from coxa joint
    double h_dist = std::sqrt(lx * lx + ly * ly);
    reachable[i] = (h_dist >= min_reach && h_dist <= max_reach);
  }
}

}  // namespace hexapod_nav_cpp
