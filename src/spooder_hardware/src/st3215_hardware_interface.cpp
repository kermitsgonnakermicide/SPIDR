#include "spooder_hardware/st3215_hardware_interface.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spooder_hardware::ST3215System,
                       hardware_interface::SystemInterface)

namespace spooder_hardware {

hardware_interface::CallbackReturn ST3215System::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse hardware parameters
  auto get_param = [&](const std::string &key, const std::string &fallback) {
    auto it = info.hardware_parameters.find(key);
    return it != info.hardware_parameters.end() ? it->second : fallback;
  };

  port_ = get_param("port", "/dev/ttyUSB0");
  baudrate_ = std::stoi(get_param("baudrate", "1000000"));

  // Parse joint-to-servo mapping
  joints_.reserve(info.joints.size());
  for (const auto &joint : info.joints) {
    ServoJoint sj;
    sj.name = joint.name;

    auto it = joint.parameters.find("servo_id");
    if (it == joint.parameters.end()) {
      RCLCPP_ERROR(logger_, "Joint '%s' missing 'servo_id' parameter",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    sj.servo_id = static_cast<uint8_t>(std::stoi(it->second));
    joints_.push_back(sj);
  }

  // Prepare bulk I/O vectors
  servo_ids_.reserve(joints_.size());
  target_positions_.resize(joints_.size(), 0.0);
  current_positions_.resize(joints_.size(), 0.0);
  for (auto &sj : joints_) {
    servo_ids_.push_back(sj.servo_id);
  }

  RCLCPP_INFO(logger_, "ST3215System initialized with %zu joints on %s",
              joints_.size(), port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215System::on_configure(
    const rclcpp_lifecycle::State & /*prev*/) {
  RCLCPP_INFO(logger_, "Configuring ST3215System...");

  comm_ = std::make_unique<ST3215Communication>(port_, baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215System::on_activate(
    const rclcpp_lifecycle::State & /*prev*/) {
  RCLCPP_INFO(logger_, "Activating ST3215System...");

  if (!comm_->connect()) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Enable torque on all servos
  bool all_ok = true;
  for (auto &sj : joints_) {
    if (!comm_->ping(sj.servo_id)) {
      RCLCPP_WARN(logger_, "No response from servo %d (joint '%s')",
                  sj.servo_id, sj.name.c_str());
    }
    if (!comm_->set_torque(sj.servo_id, true)) {
      RCLCPP_WARN(logger_, "Failed to enable torque on servo %d", sj.servo_id);
    }
  }

  // Read initial positions via SYNC_READ
  if (!comm_->sync_read_positions(servo_ids_, current_positions_)) {
    RCLCPP_WARN(logger_, "SYNC_READ failed at activation; using zeros");
    std::fill(current_positions_.begin(), current_positions_.end(), 0.0);
  }

  // Seed command from current state
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].position_state = current_positions_[i];
    joints_[i].position_command = current_positions_[i];
    target_positions_[i] = current_positions_[i];
  }

  all_commands_changed_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215System::on_deactivate(
    const rclcpp_lifecycle::State & /*prev*/) {
  RCLCPP_INFO(logger_, "Deactivating ST3215System...");

  // Disable torque on all servos
  for (auto &sj : joints_) {
    comm_->set_torque(sj.servo_id, false);
  }

  comm_->disconnect();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ST3215System::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> states;
  for (auto &sj : joints_) {
    states.emplace_back(sj.name, "position", &sj.position_state);
    states.emplace_back(sj.name, "velocity", &sj.velocity_state);
    states.emplace_back(sj.name, "effort", &sj.effort_state);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface>
ST3215System::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> cmds;
  for (auto &sj : joints_) {
    cmds.emplace_back(sj.name, "position", &sj.position_command);
  }
  return cmds;
}

hardware_interface::return_type ST3215System::read(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {
  // SYNC_READ all servo positions in one bus transaction
  if (!comm_->sync_read_positions(servo_ids_, current_positions_)) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].position_state = current_positions_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ST3215System::write(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  // Check if any command changed
  static constexpr double kThreshold = 0.001;
  bool any_changed = all_commands_changed_;
  all_commands_changed_ = false;

  for (size_t i = 0; i < joints_.size(); ++i) {
    double diff = joints_[i].position_command - target_positions_[i];
    if (std::abs(diff) > kThreshold) {
      any_changed = true;
      target_positions_[i] = joints_[i].position_command;
    }
  }

  if (!any_changed) return hardware_interface::return_type::OK;

  // SYNC_WRITE all target positions in one bus transaction
  if (!comm_->sync_write_positions(servo_ids_, target_positions_)) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace spooder_hardware
