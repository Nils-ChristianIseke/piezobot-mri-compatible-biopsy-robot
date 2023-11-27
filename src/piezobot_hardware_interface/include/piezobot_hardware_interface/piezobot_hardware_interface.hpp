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

#ifndef PIEZOBOT_HARDWARE_INTERFACE__PIEZOBOT_HARDWARE_INTERFACE_HPP_
#define PIEZOBOT_HARDWARE_INTERFACE__PIEZOBOT_HARDWARE_INTERFACE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "piezobot_hardware_interface/piezobot_commander_rs.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "piezobot_hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace piezobot_hardware_interface {
class PiezobotHardwareInterface : public hardware_interface::SystemInterface {

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PiezobotHardwareInterface)
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::vector<double> joint_pos_commands_;
  std::vector<double> joint_pos_states_;
  std::vector<std::string> position_command_interface_names_;

  std::unique_ptr<piezobot_commander_rs::PiezobotCommanderRS>
      piezobot_commander_rs_;

  rclcpp::Logger logger_ = rclcpp::get_logger("PiezobotHardwareInterface");
};

} // namespace piezobot_hardware_interface

#endif // PIEZOBOT_HARDWARE_INTERFACE__PIEZOBOT_HARDWARE_INTERFACE_HPP_
