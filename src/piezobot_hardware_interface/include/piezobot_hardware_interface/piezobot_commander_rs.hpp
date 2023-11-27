#ifndef PIEZO_COMMANDER_RS_HPP
#define PIEZO_COMMANDER_RS_HPP

/**
 * @file piezobot_commander_rs.hpp
 *
 *  PiezobotCommanderRS
 *
 */
// Filename: piezobot_commander.hpp

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include <piezobot_hardware_interface/piezobot_hardware_interface.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unistd.h>

namespace piezobot_commander_rs {

class PiezobotCommanderRS {

public:
  PiezobotCommanderRS(std::string serial_port_name)
      : serial_port_name_(serial_port_name){};
  void activate_motors();
  void connect_to_serial_port();
  void close_serial_port();
  void setup();
  void disconnect_from_serial_port();
  std::string send_command(const std::string &message);
  void deactivate_motors();
  bool read_joint_states(std::vector<double> &position_state);
  bool write_joint_commands(const std::vector<double> &position_command);
  void calibration_run();
  void jog_joints(const std::vector<int> &jog_command);
  void shutdown();

private:
  LibSerial::SerialPort serial_port_;
  std::string serial_port_name_;

  int timeout_ms_;
  LibSerial::BaudRate baudrate_ = LibSerial::BaudRate::BAUD_115200;
  std::vector<double> encoder_resolutions_ = {0.000035275, 0.000035275, 0.0004,
                                              0.0004 / 2};
  rclcpp::Logger logger_ = rclcpp::get_logger("PiezobotCommanderRS");
  bool movement_enabled_ = true;
  std::vector<double> joint_positions_ = {0, 0, 0, 0};
};

void PiezobotCommanderRS::connect_to_serial_port() {
  try {
    serial_port_.Open(serial_port_name_);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    // serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    // serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    // serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    // serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    if (serial_port_.IsOpen()) {
      RCLCPP_INFO(logger_, "Connected successfully to %s",
                  serial_port_name_.c_str());
    }
  } catch (const LibSerial::OpenFailed &e) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", e.what());
  }
  return;
}

void PiezobotCommanderRS::disconnect_from_serial_port() {
  if (serial_port_.IsOpen()) {
    try {
      serial_port_.Close();
      std::cout << "Serial port closed successfully." << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Failed to close serial port: " << e.what() << std::endl;
    }
  }
}

void PiezobotCommanderRS::activate_motors() {
  std::string request = "X1M2;X2M2;X3M2;X4M2";
  this->send_command(request);
  // last_time_stamp_ = std::chrono::steady_clock::now();
  movement_enabled_ = true;
  rclcpp::sleep_for(std::chrono::seconds(1));
  this->send_command("Y25,1");
  rclcpp::sleep_for(std::chrono::seconds(1));
}

void PiezobotCommanderRS::deactivate_motors() {
  this->send_command("X1M4\n");
  sleep(0.3);
  this->send_command("X2M4\n");
  sleep(0.3);
  this->send_command("X3M4\n");
  sleep(0.3);
  this->send_command("X4M4\n");
  sleep(0.3);
}

void PiezobotCommanderRS::shutdown() {}

bool PiezobotCommanderRS::write_joint_commands(
    const std::vector<double> &position_command) {

  std::vector<int> position_commands_motor;

  // Iterate over the original vector and divide each element by the divisor
  for (size_t i = 0;
       i < position_command.size() && i < encoder_resolutions_.size(); ++i) {
    position_commands_motor.push_back(static_cast<double>(position_command[i]) /
                                      encoder_resolutions_[i]);
  }

  if (movement_enabled_) {
    std::stringstream commands;
    for (int i = 0; i < 4; i++) {
      commands << "X" << i + 1 << "T-" << position_commands_motor[i] << ",1000"
               << ";";
    }

    std::string commandsStr = commands.str() + "\n";
    RCLCPP_INFO(logger_, "Jointcommands: %s", commandsStr.c_str());
    std::string response = send_command(commandsStr);
  }
  // last_time_stamp_ = std::chrono::steady_clock::now();

  return true;
}

bool PiezobotCommanderRS::read_joint_states(
    std::vector<double> &position_state) {
  std::vector<double> position_state_motor = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 4; i++) {
    // if statement cause controller board 2 is defective.
    if (i != 1) {
      std::string request = "X" + std::to_string(i + 1) + "E\n";
      std::string response = send_command(request);
      position_state_motor[i] = std::stoi(response.substr(4));
      //   RCLCPP_INFO(logger, "Position_response %s",response.c_str());
      //   RCLCPP_INFO(logger,
      //    "Position %f",position_state_motor[i-1]); RCLCPP_INFO(logger,
      //    "encoder_response %f",encoder_resolutions_[i-1]); double result =
      //    position_state_motor[i] * encoder_resolutions_[i];
      //    RCLCPP_INFO(logger, "Result %f",result);
    }
  }
  for (size_t i = 0; i < position_state_motor.size(); ++i) {
    double result = -position_state_motor[i] * encoder_resolutions_[i];
    position_state[i] = result;
  }

  // RCLCPP_INFO(logger, "joint5 %f",position_state[4]); RCLCPP_INFO(logger,
  // "joint2 %f",position_state[1]); RCLCPP_INFO(logger, "joint3
  // %f",position_state[2]); RCLCPP_INFO(logger, "Encoder of joint4
  // %f",position_state[3]); RCLCPP_INFO(logger, "Position
  // %f",position_state_motor[3]);

  // TODO: Iterate over position_state to

  return true;
}

std::string PiezobotCommanderRS::send_command(const std::string &message) {
  serial_port_.FlushIOBuffers();

  serial_port_.Write(message);

  std::string response = "";
  try {
    serial_port_.ReadLine(response, '\n', 300);
  } catch (const LibSerial::ReadTimeout &) {
    RCLCPP_INFO(logger_, "SerialPort is empty");
  }

  // RCLCPP_INFO(logger, "Logging test %s",response.c_str());
  return response;
}
void PiezobotCommanderRS::jog_joints(const std::vector<int> &jog_command) {

  std::vector<int> position_commands_motor;

  // Iterate over the original vector and divide each element by the divisor
  for (size_t i = 0; i < jog_command.size() && i < encoder_resolutions_.size();
       ++i) {
    position_commands_motor.push_back(jog_command[i]);
  }

  if (movement_enabled_) {
    std::stringstream commands;
    for (int i = 0; i < 4; i++) {
      commands << "X" << i + 1 << "J" << position_commands_motor[i] << ",0,1000"
               << ";";
    }

    std::string commandsStr = commands.str() + "\n";
    RCLCPP_INFO(logger_, "Jointcommands: %s", commandsStr.c_str());
    std::string response = send_command(commandsStr);
  }
  // last_time_stamp_ = std::chrono::steady_clock::now();
}

void PiezobotCommanderRS::calibration_run() {
  double previous_encoder_value = 0;

  read_joint_states(joint_positions_);
  do {
    previous_encoder_value = joint_positions_[0];
    jog_joints({-1000, 0, 0, 0});
    sleep(0.3);
    read_joint_states(joint_positions_);
    RCLCPP_INFO(logger_, "joint1 %f", joint_positions_[0]);
    RCLCPP_INFO(logger_, "joint1 previous%f", previous_encoder_value);
  } while (previous_encoder_value != joint_positions_[0]);

  do {
    previous_encoder_value = joint_positions_[1];
    jog_joints({0, -1000, 0, 0});
    sleep(0.3);
    read_joint_states(joint_positions_);
    RCLCPP_INFO(logger_, "joint2 %f", joint_positions_[1]);
    RCLCPP_INFO(logger_, "joint2 previous %f", previous_encoder_value);
  } while (previous_encoder_value != joint_positions_[1]);

  this->send_command("X1E0\n");
  sleep(0.3);
  this->send_command("X2E0\n");
  sleep(0.3);
  this->send_command("X1Y25,1");
  sleep(0.3);
  this->send_command("X2Y25,1");
  sleep(0.3);
  this->send_command("X1Y11,3550000");
  sleep(0.3);
  this->send_command("X2Y11,3550000");
  sleep(0.3); //
  // Set software limit, this may cause promblems if an overshoot in target
  // mode occurs, motor is stuck.
  this->send_command("X1Y4,0");
  sleep(0.3);
  this->send_command("X1Y5,2");
  sleep(0.3);
  this->send_command("X2Y4,0");
  sleep(0.3);
  this->send_command("X2Y5,2");
  sleep(0.3);
  this->send_command("X1Y3,-3400");
  sleep(0.3);
  this->send_command("X2Y3,-3400");
  sleep(0.3);
  this->send_command("X3N4\n");
  sleep(1);
  this->send_command("X3I-100000\n");
  sleep(1);
  this->send_command("X4N4\n");
  sleep(1);

  this->send_command("X4I100000\n");
  sleep(10);

  this->send_command("X3T-3460,200\n");
  sleep(0.3);
  this->send_command("X4T4450\n");
  sleep(15);
  this->send_command("X3J0\n");
  sleep(0.3);
  this->send_command("X4J0\n");
  sleep(0.3);
  this->send_command("X3E0\n");
  sleep(0.3);
  this->send_command("X4E0\n");
  RCLCPP_INFO(logger_, "Calibration run done");
}

} // namespace piezobot_commander_rs

#endif // PIEZO_COMMANDER_RS_HPP
