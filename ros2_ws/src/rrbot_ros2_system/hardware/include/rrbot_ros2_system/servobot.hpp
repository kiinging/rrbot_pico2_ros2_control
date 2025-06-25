// Copyright 2020 ros2_control Development Team
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

#ifndef RRBOT_ROS2_SYSTEM__HPP_
#define RRBOT_ROS2_SYSTEM__HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rrbot_ros2_system/visibility_control.h"

#include "rrbot_ros2_system/serial_driver.h"

namespace rrbot_ros2_system
{
class ServoBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ServoBotSystemPositionOnlyHardware);

  RRBOT_ROS2_SYSTEM_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RRBOT_ROS2_SYSTEM_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // Serial communication
  SerialDriver _serial_driver;
};

}  // namespace ros2_control_demo_example_1

#endif  // RRBOT_ROS2_SYSTEM__SERVOBOT_HPP_
