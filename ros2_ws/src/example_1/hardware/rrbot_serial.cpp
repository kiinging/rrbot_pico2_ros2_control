// File: rrbot_serial.cpp
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "ros2_control_demo_example_1/rrbot_serial.hpp"

namespace ros2_control_demo_example_1
{

hardware_interface::CallbackReturn RealRRBotSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = info.hardware_parameters.at("serial_port");
  baudrate_ = std::stoi(info.hardware_parameters.at("baud_rate"));

  hw_positions_.resize(info.joints.size(), 0.0);
  hw_commands_.resize(info.joints.size(), 0.0);

  try {
    serial_port_.setPort(port_);
    serial_port_.setBaudrate(baudrate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_port_.setTimeout(timeout);
    serial_port_.open();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("RealRRBotSerialHardware"), "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealRRBotSerialHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RealRRBotSerialHardware"), "Hardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealRRBotSerialHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RealRRBotSerialHardware"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealRRBotSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RealRRBotSerialHardware"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealRRBotSerialHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // (Optional) Query angles from Pico using "e <i>" commands
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealRRBotSerialHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    std::stringstream ss;
    ss << "m " << i << " " << hw_commands_[i] << "\n";
    serial_port_.write(ss.str());
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RealRRBotSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RealRRBotSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_1::RealRRBotSerialHardware,
  hardware_interface::SystemInterface)
