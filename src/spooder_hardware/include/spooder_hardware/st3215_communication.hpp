#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

namespace spooder_hardware {

class ST3215Communication {
public:
  explicit ST3215Communication(const std::string &port, int baudrate = 1000000);
  ~ST3215Communication();

  ST3215Communication(const ST3215Communication &) = delete;
  ST3215Communication &operator=(const ST3215Communication &) = delete;

  bool connect();
  void disconnect();
  bool is_connected() const { return connected_; }

  // Single-servo read/write
  std::optional<double> read_position(uint8_t id);
  std::optional<double> read_velocity(uint8_t id);
  std::optional<double> read_load(uint8_t id);
  std::optional<double> read_temperature(uint8_t id);
  std::optional<double> read_voltage(uint8_t id);

  bool write_position(uint8_t id, double position_rad);
  bool write_torque_enable(uint8_t id, bool enable);

  // Bulk operations via SYNC_WRITE / SYNC_READ
  bool sync_write_positions(const std::vector<uint8_t> &ids,
                            const std::vector<double> &positions_rad);
  bool sync_read_positions(const std::vector<uint8_t> &ids,
                           std::vector<double> &positions_rad);

  bool ping(uint8_t id);
  bool set_torque(uint8_t id, bool enable);

private:
  // ST3215 instruction opcodes
  enum class Cmd : uint8_t {
    PING = 0x01,
    READ_DATA = 0x02,
    WRITE_DATA = 0x03,
    REG_WRITE = 0x04,
    REG_ACTION = 0x05,
    RESET = 0x06,
    SYNC_WRITE = 0x83,
    SYNC_READ = 0x82,
  };

  // ST3215 SRAM register addresses (model SMS_STS)
  enum class Reg : uint8_t {
    TORQUE_ENABLE = 40,
    ACC = 41,
    GOAL_POSITION_L = 42,
    GOAL_POSITION_H = 43,
    GOAL_TIME_L = 44,
    GOAL_TIME_H = 45,
    GOAL_SPEED_L = 46,
    GOAL_SPEED_H = 47,
    PRESENT_POSITION_L = 56,
    PRESENT_POSITION_H = 57,
    PRESENT_SPEED_L = 58,
    PRESENT_SPEED_H = 59,
    PRESENT_LOAD_L = 60,
    PRESENT_LOAD_H = 61,
    PRESENT_VOLTAGE = 62,
    PRESENT_TEMPERATURE = 63,
    MOVING = 66,
    PRESENT_CURRENT_L = 69,
    PRESENT_CURRENT_H = 70,
    TORQUE_LIMIT_L = 48,
    TORQUE_LIMIT_H = 49,
  };

  struct Packet {
    uint8_t id = 0;
    uint8_t instruction = 0;
    std::vector<uint8_t> params;
  };

  // Low-level serial I/O
  bool send_raw(const uint8_t *data, size_t len);
  bool read_raw(uint8_t *data, size_t len, int timeout_ms);

  // Packet framing
  bool send_packet(uint8_t id, uint8_t instruction, const uint8_t *params, size_t param_len);
  bool read_response(uint8_t expected_id, std::vector<uint8_t> &params, int timeout_ms = 50);
  static uint8_t calc_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                               const uint8_t *params, size_t param_len);
  bool wait_header();

  // Register-level I/O
  bool write_reg(uint8_t id, uint8_t addr, const uint8_t *data, size_t len);
  bool read_reg(uint8_t id, uint8_t addr, uint8_t *data, size_t len);

  // Unit conversion (ST3215 15-bit signed position)
  // Position: signed 16-bit, range -30720..+30720, full 360 degrees
  // Negative stored as |val| with bit 15 set
  static double position_to_rad(uint16_t raw);
  static uint16_t rad_to_position(double rad);
  static double speed_to_rad_s(uint16_t raw);
  static double load_to_nm(uint16_t raw);

  std::string port_;
  int baudrate_;
  std::unique_ptr<LibSerial::SerialPort> serial_;
  mutable std::mutex mutex_;
  std::atomic<bool> connected_{false};
};

}  // namespace spooder_hardware
