// Copyright (c) 2023, Nils-Christian Iseke
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

#include <fstream>

class TestPiezobotHardwareInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    piezobot_hardware_interface_2dof_ =
      R"(
        <ros2_control name="PiezobotHardwareInterface" type="system">
          <hardware>
            <plugin>piezobot_hardware_interface/PiezobotHardwareInterface</plugin>
            <param name="joint1_name">"joint1"</param>
            <param name="loop_rate">100</param>
            <param name="encoder_count_heds9000">100</param>
            <param name="encoder_count_RT23">100</param>
            <param name="port_name">"/dev/"</param>
            <param name="baud_rate">10000</param>
            <param name="timeout_ms">100</param>
            </hardware>
        <joint name="joint1">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>
        </ros2_control>)";
  }

  std::string piezobot_hardware_interface_2dof_;
};

TEST_F(TestPiezobotHardwareInterface, load_piezobot_hardware_interface_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + piezobot_hardware_interface_2dof_ +
    ros2_control_test_assets::urdf_tail;

  std::ofstream outfile("/workspaces/piezobot_pc_workspace/output.txt");
  outfile << urdf << std::endl;
  outfile.close();
  try {
    hardware_interface::ResourceManager rm(urdf);
  } catch (const std::exception & e) {
    std::ofstream errorfile("/workspaces/piezobot_pc_workspace/error.txt");
    errorfile << e.what() << std::endl;
    errorfile.close();
  }

  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
