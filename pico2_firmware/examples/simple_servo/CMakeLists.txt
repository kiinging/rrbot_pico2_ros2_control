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

# Add the library from the parent directory
add_subdirectory(../../lib/pca9685 AdafruitServoLib)


# --- Executable 1: simple_servo ---
add_executable(simple_servo
    simple_servo.cpp  # Original file
)

target_link_libraries(simple_servo
    pico_stdlib
    pca9685  # Your PCA9685 driver library
)

pico_enable_stdio_usb(simple_servo 0)
pico_enable_stdio_uart(simple_servo 1)
pico_add_extra_outputs(simple_servo)

# --- Executable 2: servo_align ---
add_executable(servo_align
    servo_align.cpp  # Your new alignment code
)

target_link_libraries(servo_align
    pico_stdlib
    pca9685  # Reuse the same library
)

pico_enable_stdio_usb(servo_align 0)
pico_enable_stdio_uart(servo_align 1)
pico_add_extra_outputs(servo_align)