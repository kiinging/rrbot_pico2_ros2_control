## Adafruit Servo Driver Library for Pi Pico

A lightweight C/C++ SDK–compatible fork of the [Adafruit-Servo-Driver-Library-Pi-Pico](https://github.com/grzesiek2201/Adafruit-Servo-Driver-Library-Pi-Pico) by **grzesiek2201**.  
Originally based on Adafruit’s PCA9685 drivers, this version is tailored for the Raspberry Pi Pico (RP2040) using the official C/C++ SDK.

---

## 🚀 Simple Servo Test Example

A “hello world”–style demo that sweeps a single servo channel back and forth via the PCA9685 PWM driver. Perfect for validating your wiring, firmware build, and Pico 2 hardware setup.

### 📂 Directory Layout

examples/simple_servo/
├── CMakeLists.txt # Build script for the test executable
├── simple_servo.cpp # Example code: servo sweep logic
└── build/ # (Out-of-source build directory)


---

### 🔧 Building

1. **Create & enter** the build directory:
   ```bash
   mkdir -p build
   cd build

    Configure with CMake (targeting Pico 2):

cmake -DPICO_BOARD=pico2 ..

Compile:

    make

    This produces:

        servo_driver.elf

        servo_driver.bin

        servo_driver.uf2

🔥 Flashing & Debugging
USB UF2 (Quickest)

    Hold BOOTSEL on the Pico 2 and plug it into USB.

    Copy servo_driver.uf2 onto the RPI-RP2 device.

    The Pico will reboot and start running the test.

OpenOCD + CMSIS-DAP Probe

If you have a CMSIS-DAP (or Picoprobe) attached:

sudo openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program servo_driver.elf verify reset exit"

This command will flash, verify, and reset your Pico 2 in one step.

🎉 Now watch your servo sweep smoothly between its endpoints! Use this example as a quick hardware check whenever you need.