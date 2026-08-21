#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "spooder_hardware/st3215_communication.hpp"

namespace spooder_hardware {

struct ServoJoint {
  std::string name;
  uint8_t servo_id;
  double position_state = 0.0;
  double velocity_state = 0.0;
  double effort_state = 0.0;
  double position_command = 0.0;
};

class ST3215System : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ST3215System)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

private:
  std::string port_;
  int baudrate_ = 1000000;
  std::unique_ptr<ST3215Communication> comm_;
  std::vector<ServoJoint> joints_;

  // Batch read/write vectors for SYNC operations
  std::vector<uint8_t> servo_ids_;
  std::vector<double> target_positions_;
  std::vector<double> current_positions_;

  bool all_commands_changed_ = true;
  rclcpp::Logger logger_{rclcpp::get_logger("ST3215System")};
};

}  // namespace spooder_hardware
