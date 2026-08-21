#include "spooder_hardware/st3215_communication.hpp"

#include <cmath>
#include <cstring>

namespace spooder_hardware {

ST3215Communication::ST3215Communication(const std::string &port, int baudrate)
    : port_(port), baudrate_(baudrate), serial_(std::make_unique<LibSerial::SerialPort>()) {}

ST3215Communication::~ST3215Communication() { disconnect(); }

bool ST3215Communication::connect() {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (serial_->IsOpen()) serial_->Close();
    serial_->Open(port_);
    serial_->SetBaudRate(static_cast<LibSerial::BaudRate>(baudrate_));
    serial_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    connected_ = true;
    return true;
  } catch (...) {
    connected_ = false;
    return false;
  }
}

void ST3215Communication::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (serial_->IsOpen()) serial_->Close();
  connected_ = false;
}

// ---------------------------------------------------------------------------
// Unit conversion
// ---------------------------------------------------------------------------

double ST3215Communication::position_to_rad(uint16_t raw) {
  // ST3215 position is signed 16-bit: bit 15 = sign, bits 0-14 = magnitude
  // Range: -30720 to +30720 maps to one full rotation
  // Convention: 0 = center, positive = CCW (when viewed from output shaft)
  int16_t signed_val = static_cast<int16_t>(raw);
  return static_cast<double>(signed_val) * (2.0 * M_PI / 30720.0);
}

uint16_t ST3215Communication::rad_to_position(double rad) {
  double counts = rad * (30720.0 / (2.0 * M_PI));
  // Clamp to ST3215 range
  if (counts > 30720.0) counts = 30720.0;
  if (counts < -30720.0) counts = -30720.0;
  return static_cast<uint16_t>(static_cast<int16_t>(std::round(counts)));
}

double ST3215Communication::speed_to_rad_s(uint16_t raw) {
  int16_t signed_val = static_cast<int16_t>(raw);
  return static_cast<double>(signed_val) * (10.0 / 1023.0);
}

double ST3215Communication::load_to_nm(uint16_t raw) {
  // Load: bit 10 = direction, bits 0-9 = magnitude (0-1000 = 0-100%)
  // Map 0-1000 to 0-20 Nm max torque of ST3215
  uint16_t magnitude = raw & 0x3FF;
  return static_cast<double>(magnitude) * (20.0 / 1000.0);
}

// ---------------------------------------------------------------------------
// Raw serial I/O
// ---------------------------------------------------------------------------

bool ST3215Communication::send_raw(const uint8_t *data, size_t len) {
  try {
    serial_->Write(data, len);
    serial_->DrainWriteBuffer();
    return true;
  } catch (...) {
    return false;
  }
}

bool ST3215Communication::read_raw(uint8_t *data, size_t len, int timeout_ms) {
  try {
    size_t read = 0;
    while (read < len) {
      serial_->ReadByte(data[read], timeout_ms);
      ++read;
    }
    return true;
  } catch (...) {
    return false;
  }
}

// ---------------------------------------------------------------------------
// Packet framing
// ---------------------------------------------------------------------------

uint8_t ST3215Communication::calc_checksum(uint8_t id, uint8_t length,
                                            uint8_t instruction,
                                            const uint8_t *params,
                                            size_t param_len) {
  uint16_t sum = id + length + instruction;
  for (size_t i = 0; i < param_len; ++i) sum += params[i];
  return static_cast<uint8_t>(~sum & 0xFF);
}

bool ST3215Communication::send_packet(uint8_t id, uint8_t instruction,
                                       const uint8_t *params, size_t param_len) {
  uint8_t length = static_cast<uint8_t>(param_len + 2);
  uint8_t buf[256];
  size_t idx = 0;

  buf[idx++] = 0xFF;
  buf[idx++] = 0xFF;
  buf[idx++] = id;
  buf[idx++] = length;
  buf[idx++] = instruction;

  if (params && param_len > 0) {
    std::memcpy(buf + idx, params, param_len);
    idx += param_len;
  }

  buf[idx++] = calc_checksum(id, length, instruction, params, param_len);

  return send_raw(buf, idx);
}

bool ST3215Communication::wait_header() {
  uint8_t byte;
  uint8_t prev = 0;
  for (int i = 0; i < 20; ++i) {
    if (!read_raw(&byte, 1, 10)) return false;
    if (byte == 0xFF && prev == 0xFF) return true;
    prev = byte;
  }
  return false;
}

bool ST3215Communication::read_response(uint8_t expected_id,
                                         std::vector<uint8_t> &params,
                                         int timeout_ms) {
  (void)timeout_ms;
  if (!wait_header()) return false;

  uint8_t id, length, instruction;
  if (!read_raw(&id, 1, 50)) return false;
  if (!read_raw(&length, 1, 50)) return false;

  if (length < 2) return false;

  uint8_t err_or_status;
  if (!read_raw(&err_or_status, 1, 50)) return false;

  size_t data_len = length - 2;
  params.resize(data_len);
  if (data_len > 0) {
    if (!read_raw(params.data(), data_len, 50)) return false;
  }

  uint8_t checksum;
  if (!read_raw(&checksum, 1, 50)) return false;

  // Validate checksum
  uint16_t sum = id + length + err_or_status;
  for (size_t i = 0; i < data_len; ++i) sum += params[i];
  uint8_t expected = static_cast<uint8_t>(~sum & 0xFF);
  if (checksum != expected) return false;

  if (expected_id != 0xFE && id != expected_id) return false;

  return true;
}

// ---------------------------------------------------------------------------
// Register I/O
// ---------------------------------------------------------------------------

bool ST3215Communication::write_reg(uint8_t id, uint8_t addr,
                                     const uint8_t *data, size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> params;
  params.push_back(addr);
  params.insert(params.end(), data, data + len);
  if (!send_packet(id, static_cast<uint8_t>(Cmd::WRITE_DATA), params.data(),
                   params.size())) {
    return false;
  }
  std::vector<uint8_t> resp;
  return read_response(id, resp, 50);
}

bool ST3215Communication::read_reg(uint8_t id, uint8_t addr, uint8_t *data,
                                    size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t params[2] = {addr, static_cast<uint8_t>(len)};
  if (!send_packet(id, static_cast<uint8_t>(Cmd::READ_DATA), params, 2)) {
    return false;
  }
  std::vector<uint8_t> resp;
  if (!read_response(id, resp, 50)) return false;
  if (resp.size() < len) return false;
  std::memcpy(data, resp.data(), len);
  return true;
}

// ---------------------------------------------------------------------------
// High-level read/write
// ---------------------------------------------------------------------------

std::optional<double> ST3215Communication::read_position(uint8_t id) {
  uint8_t buf[2];
  if (!read_reg(id, static_cast<uint8_t>(Reg::PRESENT_POSITION_L), buf, 2))
    return std::nullopt;
  uint16_t raw = buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
  return position_to_rad(raw);
}

std::optional<double> ST3215Communication::read_velocity(uint8_t id) {
  uint8_t buf[2];
  if (!read_reg(id, static_cast<uint8_t>(Reg::PRESENT_SPEED_L), buf, 2))
    return std::nullopt;
  uint16_t raw = buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
  return speed_to_rad_s(raw);
}

std::optional<double> ST3215Communication::read_load(uint8_t id) {
  uint8_t buf[2];
  if (!read_reg(id, static_cast<uint8_t>(Reg::PRESENT_LOAD_L), buf, 2))
    return std::nullopt;
  uint16_t raw = buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
  return load_to_nm(raw);
}

std::optional<double> ST3215Communication::read_temperature(uint8_t id) {
  uint8_t buf;
  if (!read_reg(id, static_cast<uint8_t>(Reg::PRESENT_TEMPERATURE), &buf, 1))
    return std::nullopt;
  return static_cast<double>(buf);
}

std::optional<double> ST3215Communication::read_voltage(uint8_t id) {
  uint8_t buf;
  if (!read_reg(id, static_cast<uint8_t>(Reg::PRESENT_VOLTAGE), &buf, 1))
    return std::nullopt;
  return static_cast<double>(buf) / 10.0;
}

bool ST3215Communication::write_position(uint8_t id, double position_rad) {
  uint16_t pos = rad_to_position(position_rad);
  uint8_t buf[2] = {static_cast<uint8_t>(pos & 0xFF),
                     static_cast<uint8_t>((pos >> 8) & 0xFF)};
  return write_reg(id, static_cast<uint8_t>(Reg::GOAL_POSITION_L), buf, 2);
}

bool ST3215Communication::write_torque_enable(uint8_t id, bool enable) {
  uint8_t val = enable ? 1 : 0;
  return write_reg(id, static_cast<uint8_t>(Reg::TORQUE_ENABLE), &val, 1);
}

// ---------------------------------------------------------------------------
// SYNC_WRITE: write same register to multiple servos in one bus transaction
// ---------------------------------------------------------------------------

bool ST3215Communication::sync_write_positions(
    const std::vector<uint8_t> &ids, const std::vector<double> &positions_rad) {
  if (ids.empty() || ids.size() != positions_rad.size()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  // SYNC_WRITE: header + ID(0xFE) + length + instruction + start_addr +
  //            data_len + [id + data]...
  // Each servo: 2 bytes (goal position L/H)
  size_t servo_count = ids.size();
  size_t per_servo = 2;
  uint8_t msg_len =
      static_cast<uint8_t>(4 + servo_count * (1 + per_servo));

  std::vector<uint8_t> packet;
  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(0xFE);  // broadcast
  packet.push_back(msg_len);
  packet.push_back(static_cast<uint8_t>(Cmd::SYNC_WRITE));
  packet.push_back(static_cast<uint8_t>(Reg::GOAL_POSITION_L));
  packet.push_back(static_cast<uint8_t>(per_servo));

  uint16_t checksum = 0xFE + msg_len +
                       static_cast<uint8_t>(Cmd::SYNC_WRITE) +
                       static_cast<uint8_t>(Reg::GOAL_POSITION_L) + per_servo;

  for (size_t i = 0; i < servo_count; ++i) {
    uint16_t pos = rad_to_position(positions_rad[i]);
    uint8_t lo = static_cast<uint8_t>(pos & 0xFF);
    uint8_t hi = static_cast<uint8_t>((pos >> 8) & 0xFF);

    packet.push_back(ids[i]);
    packet.push_back(lo);
    packet.push_back(hi);

    checksum += ids[i] + lo + hi;
  }

  packet.push_back(static_cast<uint8_t>(~checksum & 0xFF));

  return send_raw(packet.data(), packet.size());
}

// ---------------------------------------------------------------------------
// SYNC_READ: read same register block from multiple servos
// ---------------------------------------------------------------------------

bool ST3215Communication::sync_read_positions(
    const std::vector<uint8_t> &ids, std::vector<double> &positions_rad) {
  if (ids.empty()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  // SYNC_READ: read 2 bytes from PRESENT_POSITION_L for each servo
  uint8_t per_servo = 2;
  uint8_t msg_len = static_cast<uint8_t>(4 + ids.size());

  std::vector<uint8_t> packet;
  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(0xFE);  // broadcast
  packet.push_back(msg_len);
  packet.push_back(static_cast<uint8_t>(Cmd::SYNC_READ));
  packet.push_back(static_cast<uint8_t>(Reg::PRESENT_POSITION_L));
  packet.push_back(per_servo);

  uint16_t checksum = 0xFE + msg_len +
                       static_cast<uint8_t>(Cmd::SYNC_READ) +
                       static_cast<uint8_t>(Reg::PRESENT_POSITION_L) + per_servo;

  for (uint8_t id : ids) {
    packet.push_back(id);
    checksum += id;
  }

  packet.push_back(static_cast<uint8_t>(~checksum & 0xFF));

  if (!send_raw(packet.data(), packet.size())) return false;

  // Read responses from each servo
  positions_rad.resize(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    std::vector<uint8_t> resp;
    if (!read_response(ids[i], resp, 20) || resp.size() < 2) {
      positions_rad[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    uint16_t raw = resp[0] | (static_cast<uint16_t>(resp[1]) << 8);
    positions_rad[i] = position_to_rad(raw);
  }

  return true;
}

bool ST3215Communication::ping(uint8_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!send_packet(id, static_cast<uint8_t>(Cmd::PING), nullptr, 0))
    return false;
  std::vector<uint8_t> resp;
  return read_response(id, resp, 50);
}

bool ST3215Communication::set_torque(uint8_t id, bool enable) {
  return write_torque_enable(id, enable);
}

}  // namespace spooder_hardware
