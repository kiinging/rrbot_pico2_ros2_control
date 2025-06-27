# rrbot_pico2_ros2_control

This repository contains all the software components needed to control a 2-joint robotic arm using ROS 2 and a Raspberry Pi Pico 2 (RP2350) as the hardware interface. The main controller is an Orange Pi 5 Max (or any compatible SBC), which runs the ROS 2 control stack.

---

## 📁 Repository Structure

```bash
rrbot_pico2_ros2_control/
├── pico2_firmware/        # Firmware for the Raspberry Pi Pico 2 (RP2350)
│   ├── build/             # Build directory (can be ignored in version control)
│   ├── examples/          # Example usage (e.g., simple_servo)
│   ├── include/           # Header files, including PCA9685 driver
│   ├── src/               # Main application and driver sources
│   ├── CMakeLists.txt     # Build configuration
│   └── README.md          # Details about firmware
├── ros2_ws/               # ROS 2 workspace
│   └── src/
│       └── rrbot_ros2_system/  # ROS 2 control packages for the 2-joint robot
└── README.md              # Root README (this file)
```

## Sources 
git clone from https://github.com/grzesiek2201/servobot_pico_driver

