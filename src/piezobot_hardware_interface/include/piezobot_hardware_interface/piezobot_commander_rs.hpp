#ifndef PIEZO_COMMANDER_RS_HPP
#define PIEZO_COMMANDER_RS_HPP
// HEADER ONLY LIBRARY OF PIEZOBOT COMMANDER RS

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <libserial/SerialStream.h>


#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unistd.h>


#include <boost/regex.hpp>
#include <string>

#include <iostream>
#include <string>
#include <vector>
#include <regex>


namespace piezobot_commander_rs
{

class PiezobotCommanderRS
{
public:
  PiezobotCommanderRS(
    std::string serial_port_name,
    int encoder_count_linear, int encoder_count_prismatic) \
  : serial_port_name_(serial_port_name), encoder_count_linear_(encoder_count_linear),
    encoder_count_prismatic_(encoder_count_prismatic)
  {
    encoder_counts_ =
  {encoder_count_linear_, encoder_count_linear_,
    encoder_count_prismatic_*2/(2*M_PI),
    encoder_count_prismatic_/(2*M_PI), 1, 1};
  }
  void activate_motors();
  void connect_to_serial_port();
  void close_serial_port();
  std::string send_command(const std::string &, bool awnser);
  void deactivate_motors();
  void write_joint_commands(const std::vector<double> & position_command);
  void calibration_run();
  void jog_joints(const std::vector<int> & jog_command);
  void go_home();
  void initiate();
  void extractEncoderValues(const std::string & input, std::vector<int> & number);
  void read_joint_states(std::vector<double> & position_state);
  void enable_movement();
  void disable_movement();

private:
  bool movement_enabled_ = false;
  bool calibrate_robot_ = true;
  const std::string serial_port_name_;
  int encoder_count_linear_;
  int encoder_count_prismatic_;
  LibSerial::SerialPort serial_port_;
  static constexpr int MILLIMETER_IN_METER = 1000;
  std::vector<double> encoder_counts_;
  rclcpp::Logger logger_ = rclcpp::get_logger("PiezobotCommander");
  std::string last_target_command;
};

void PiezobotCommanderRS::enable_movement()
{
  this->movement_enabled_ = true;
  RCLCPP_INFO(logger_, "Movement of Piezobot enabled!");
}

void PiezobotCommanderRS::disable_movement()
{
  this->movement_enabled_ = false;
  RCLCPP_INFO(logger_, "Movement of Piezobot disabled!");

}
void PiezobotCommanderRS::calibration_run(){
  
};

void PiezobotCommanderRS::connect_to_serial_port()
{
  serial_port_.Open(serial_port_name_);
  serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
  serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  if (serial_port_.IsOpen()) {
    RCLCPP_INFO(logger_, "Connected successfully to %s", serial_port_name_.c_str());
  }
}

void PiezobotCommanderRS::close_serial_port()
{
  if (serial_port_.IsOpen()) {
    try {
      serial_port_.Close();
      RCLCPP_INFO(logger_,"Serial port closed successfully.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to close serial port: %s", e.what());
    }
  }
}

void PiezobotCommanderRS::activate_motors()
{
  std::string request = "X0~M2\n";
  send_command(request, false);
  this->initiate();
}


void PiezobotCommanderRS::deactivate_motors()
{
  std::string request = "X0~M4\n";
  send_command(request, false);
}

void PiezobotCommanderRS::write_joint_commands(const std::vector<double> & position_command)
{
  if (movement_enabled_) {
    std::vector<int> position_commands_motor{0,0,0,0};

    for (size_t i = 0; i < position_command.size(); ++i) {
      position_commands_motor[i]=
        static_cast<__float128>(position_command[i]) * encoder_counts_[i];
    }

    std::stringstream commands;
    for (int i = 0; i < 4; i++) {
      commands << "X" << i + 1 << "T" << -position_commands_motor[i] << ",1000"
               << ";";
    }
    std::string commandsStr = commands.str();
    RCLCPP_INFO(logger_, "Jointcommands: %s", commandsStr.c_str());
    
    if (commandsStr != last_target_command) {
      send_command(commandsStr, false);
      last_target_command = commandsStr;
      RCLCPP_INFO(logger_, "Sended Position Target Command.");
    }
    else{
      RCLCPP_INFO(logger_, "Position Target command did not change, hence it wont be send again.");
    }  }
}


void PiezobotCommanderRS::read_joint_states(std::vector<double> & position_state)
{
  // Currently not working, because the encoder values are not read correctly.
  std::vector<int> position_state_motor;
  position_state_motor.reserve(6);
  std::string request = "X0~E\n";
  std::string joint_values = send_command(request, true);
  
  extractEncoderValues(joint_values, position_state_motor);
  if (position_state_motor.size() != 4) {
    RCLCPP_ERROR(
      logger_, "Got incorrect number of encoder values %ld",
      position_state_motor.size());
  }

  position_state_motor.push_back(0);
  position_state_motor.push_back(0);

  for (size_t i = 0; i < position_state_motor.size(); ++i) {
    double result = position_state_motor[i] / encoder_counts_[i];
    position_state[i] = result;
  }
}

void PiezobotCommanderRS::extractEncoderValues(const std::string & input, std::vector<int> & number)
{
  boost::regex r("(?<![X])\\d+");
  boost::sregex_iterator it(input.begin(), input.end(), r);
  boost::sregex_iterator end;

  while (it != end) {

    number.push_back(std::stoi(it->str()));
    ++it;
  }
}

std::string PiezobotCommanderRS::send_command(const std::string & message, bool anwser)
{
  serial_port_.FlushIOBuffers();
  serial_port_.Write(message);
  unsigned char buffer=' ';
  try{

    serial_port_.ReadByte(buffer, 300);
    } 
  catch (LibSerial::ReadTimeout & e) {
      e.what();
      }
  std::string response ="";
  if (anwser) {
    try {
      int number_of_bytes = serial_port_.GetNumberOfBytesAvailable();
      
      serial_port_.Read(response, number_of_bytes, 300);
    } catch (LibSerial::ReadTimeout & e) {
      e.what();
    }
  }
  return response;
}


void PiezobotCommanderRS::jog_joints(const std::vector<int> & jog_command)
{
  if (movement_enabled_) {
    std::stringstream commands;
    for (int i = 0; i < 4; i++) {
      commands << "X" << i + 1 << "J" << 10 << ",0," << jog_command[i] << ";";
    }
    std::string commandsStr = commands.str() + ";";
    send_command(commandsStr, false);
  }
}
void PiezobotCommanderRS::initiate()
{
  //CHECK PMWDO4 Manual for more information on how to set up the motors.
  //This could be optimized in future, eg by using an object oriented approach
  send_command("X1Y11,97090;X1Y3,-4262;2Y4,0;X1Y10,5;X1Y9,5;X1Y12,3;X1Y5,2;", false);
  send_command(
    "X2Y11,97090;X2Y3,-4262;X2Y4,0;X2Y10,5;X2Y9,5;X2Y12,3;X2Y5,2;X2Y6,1;",
    false);
  send_command("X3Y10,10;X3Y12,3;X3Y5,2;X3Y3,-8000;X3Y4,4000;X4Y25,1;", false);
  send_command("X4Y10,10;X4Y12,3;X4Y5,2;X4Y3,-100000;X3Y4,100000;X4Y25,1;", false);
}

}  // namespace piezobot_commander_rs

#endif  // PIEZO_COMMANDER_RS_HPP



  // constexpr const char * TARGET_LIMIT_LOW = "Y3";
  // constexpr const char * TARGET_LIMIT_HIGH = "Y4";
  // constexpr const char * TARGET_MODE_STOP_RANGE = "Y5";
  // constexpr const char * TARGET_MODE_STOP_RANGE_VALUE = "Y6";
  // constexpr const char * SPEED_RAMP_UP = "Y9";
  // constexpr const char * SPEED_RAMP_DOWN = "Y10";
  // constexpr const char * STEPS_PER_COUNT = "Y11";
  // constexpr const char * TARGET_MODE_MODEL = "Y12";
  // constexpr const char * AUTOCALIBRATION = "Y25,1";