cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_1 LANGUAGES CXX)

# Basic setup
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Ensure required version of ros2_control
find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required.")
endif()

# Ament and dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ✅ Add libserial via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

## COMPILE
add_library(
  ros2_control_demo_example_1
  SHARED
  hardware/rrbot.cpp
  hardware/serial_driver.cpp
)

target_compile_features(ros2_control_demo_example_1 PUBLIC cxx_std_17)

# Includes
target_include_directories(ros2_control_demo_example_1 PUBLIC
  ${LIBSERIAL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/ros2_control_demo_example_1>
)

# ✅ FIXED: Plain-style (no "PUBLIC") to avoid conflict with ament
target_link_libraries(ros2_control_demo_example_1
  ${LIBSERIAL_LIBRARIES}
)

# ROS 2 dependencies
ament_target_dependencies(ros2_control_demo_example_1
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_1.xml)

## INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_1
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_1
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_1
)

install(TARGETS ros2_control_demo_example_1
  EXPORT export_ros2_control_demo_example_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_1_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_1_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_1_launch test/test_rrbot_launch.py)
  ament_add_pytest_test(run_example_1_launch_cli_direct test/test_rrbot_launch_cli_direct.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_1 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
