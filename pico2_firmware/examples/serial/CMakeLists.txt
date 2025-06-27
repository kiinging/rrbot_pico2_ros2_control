# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.14)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(PICO_BOARD pico2)

# Set name of project (as PROJECT_NAME) and C/CXX standards
project(servo_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# --- Executable 1: serial_echo ---
add_executable(serial_echo
    serial_echo.cpp
)

target_link_libraries(serial_echo pico_stdlib)

pico_enable_stdio_usb(serial_echo 1)
pico_enable_stdio_uart(serial_echo 0)

pico_add_extra_outputs(serial_echo)


# --- Executable 2: serial_servo ---
add_executable(serial_servo
    serial_servo.cpp  # Your new alignment code
)

target_link_libraries(serial_servo pico_stdlib)

pico_enable_stdio_usb(serial_servo 1)
pico_enable_stdio_uart(serial_servo 0)
pico_add_extra_outputs(serial_servo)