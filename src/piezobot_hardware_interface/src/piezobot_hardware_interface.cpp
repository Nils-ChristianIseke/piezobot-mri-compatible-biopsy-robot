// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software distributed under the License
// is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
// or implied. See the License for the specific language governing permissions and limitations under
// the License.


#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "piezobot_hardware_interface/piezobot_hardware_interface.hpp"

namespace piezobot_hardware_interface
{
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string serial_port_name = info_.hardware_parameters["serial_port_name"];
  int encoder_count_linear = std::stoi(info_.hardware_parameters["encoder_count_per_meter_heds_9200"]);
  int encoder_count_prismatic =
    std::stoi(info_.hardware_parameters["encoder_count_per_revolution_piezomotor_rt23"]);
 
  piezobot_commander_rs_ =
    std::make_unique<piezobot_commander_rs::PiezobotCommanderRS>(
    serial_port_name,
    encoder_count_linear, \
    encoder_count_prismatic);

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Piezobot"),
        "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Piezobot"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn PiezobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }
  piezobot_commander_rs_->connect_to_serial_port();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiezobotHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  piezobot_commander_rs_->close_serial_port();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiezobotHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  piezobot_commander_rs_->disable_movement();
  piezobot_commander_rs_->deactivate_motors();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PiezobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PiezobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PiezobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }
  piezobot_commander_rs_->activate_motors();
  piezobot_commander_rs_->enable_movement();
  RCLCPP_INFO(rclcpp::get_logger("Piezobot"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiezobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  piezobot_commander_rs_->disable_movement();
  piezobot_commander_rs_->deactivate_motors();

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type PiezobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw-states are mimicking the hardware-command
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiezobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  piezobot_commander_rs_->write_joint_commands(this->hw_commands_);
  return hardware_interface::return_type::OK;
}

}  // namespace piezobot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  piezobot_hardware_interface::PiezobotHardwareInterface, hardware_interface::SystemInterface)
