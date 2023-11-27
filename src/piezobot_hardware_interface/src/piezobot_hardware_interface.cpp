// Copyright (c) 2023, Nils-Christian Iseke
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template)
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

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "piezobot_hardware_interface/piezobot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace piezobot_hardware_interface {

/// @brief Initializes the necessary variables with the values specified in the
/// urdf of the robot via the
/// <plugin>piezobot_hardware_interface/PiezobotHardwareInterface</plugin>
/// @param info Parameters defined by the plugin
/// @return
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joint_pos_commands_.resize(info_.joints.size(),
                             std::numeric_limits<double>::quiet_NaN());
  joint_pos_states_.resize(info_.joints.size(),
                           std::numeric_limits<double>::quiet_NaN());

  piezobot_commander_rs_ =
      std::make_unique<piezobot_commander_rs::PiezobotCommanderRS>(
          info_.hardware_parameters["serial_port_name"]);
  return CallbackReturn::SUCCESS;
}

/// @brief Enables the transition between the uncofigured and the
/// configured state of the hardware, this is done by connecting to the serial
/// port
/// @param
/// @return
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  piezobot_commander_rs_->connect_to_serial_port();
  return CallbackReturn::SUCCESS;
}

/// @brief Ativates the hardware by engaging the motors
/// @param
/// @return
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  piezobot_commander_rs_->activate_motors();
  // joint_pos_commands_ = {0, 0, 0, 0};

  return CallbackReturn::SUCCESS;
}

/// @brief Deactivates the hardware by disengaging the motors
/// @param
/// @return

hardware_interface::CallbackReturn PiezobotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  piezobot_commander_rs_->deactivate_motors();
  return CallbackReturn::SUCCESS;
}

/// @brief Unconfigures the hardware by closing the connection to the serial
/// port
/// @param
/// @return
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  piezobot_commander_rs_->disconnect_from_serial_port();
  return CallbackReturn::SUCCESS;
}

/// @brief Shutdowns the hardware, no implementation yet.
/// @param
/// @return
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PiezobotHardwareInterface::read(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {
  piezobot_commander_rs_->read_joint_states(joint_pos_states_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PiezobotHardwareInterface::write(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  piezobot_commander_rs_->write_joint_commands(joint_pos_commands_);
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
PiezobotHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_pos_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PiezobotHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  position_command_interface_names_.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_pos_commands_[i]));
    position_command_interface_names_.push_back(
        command_interfaces.back().get_name());
  }

  return command_interfaces;
}

} // namespace piezobot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(piezobot_hardware_interface::PiezobotHardwareInterface,
                       hardware_interface::SystemInterface)
