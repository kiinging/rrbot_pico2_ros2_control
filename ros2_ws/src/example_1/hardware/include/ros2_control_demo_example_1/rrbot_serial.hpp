#ifndef ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_SERIAL_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_SERIAL_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/macros.hpp"
#include "serial/serial.h"

namespace ros2_control_demo_example_1
{

class RealRRBotSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealRRBotSerialHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::string port_;
  int baudrate_;
  serial::Serial serial_port_;
};

}  // namespace ros2_control_demo_example_1

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_1_RRBOT_SERIAL_HPP_
