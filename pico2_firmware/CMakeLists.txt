cmake_minimum_required(VERSION 3.12)

# Pull in Pico SDK
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

# Set name of project and C/CXX standards
project(servo_freertos C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Enable C++ exceptions if needed (optional)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.1.0")
    message(FATAL_ERROR "Requires Pico SDK version 2.1.0 or newer.")
endif()

# Optional: Wait for USB for printf()
if (NOT DEFINED PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS)
    set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 3000)
endif()

pico_sdk_init()


# Updated compiler flags for C++
add_compile_options(
    -Wall
    -Wextra
    -Wno-format          # Keep if you need to suppress format warnings
    -Wno-unused-function # Keep if needed
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-register>  # Suppress C++ specific warnings if needed
)


# C++ specific options
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>          # Disable RTTI if not needed
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>    # Only if you disabled exceptions
    )
endif()


if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
    return()
endif()

include(FreeRTOS_Kernel_import.cmake)

# Add subdirectories
add_subdirectory(include/pca9685)  # ✅ Only here
add_subdirectory(src)
