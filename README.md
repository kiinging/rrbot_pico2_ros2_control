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

🧠 Key Takeaway (Today’s Progress)
The biggest breakthrough today was finding and integrating a ROS 2-compatible serial library that allows seamless communication with the Pico microcontroller. This replaces manual serial hacks with a clean, buildable, ROS-native solution.

🔌 Reliable Serial Library for ROS 2
We are using a ROS 2 port of the popular wjwwood/serial library, maintained here:

🔗 https://github.com/tylerjw/serial/tree/ros2

This version is ROS 2-ready (ament_cmake), compiles cleanly in the workspace, and installs correctly with headers such as serial/serial.h and serial/impl/unix.h.

✅ How to Install It
bash
Copy
Edit
# Navigate to your ROS 2 workspace
cd ~/ros2_projects/ros2_ws/src

# Clone the ROS 2-compatible serial library
git clone -b ros2 https://github.com/tylerjw/serial.git

# Build your workspace
cd ..
colcon build

# Source the overlay
source install/setup.bash
Now your ROS 2 packages can cleanly #include <serial/serial.h> and interface with /dev/ttyACM0 (or whatever device your Pico is on).

📡 Communication Protocol with the Pico
The firmware running on the Pico uses a simple text-based serial protocol:

m <servo_index> <radians> — move servo to position

e <servo_index> — echo current servo position

r — reset all servos to 0

j <index> <radians> — (auto-published) joint feedback, sent periodically

These messages are parsed and executed onboard the Pico. You can listen to the feedback and publish it to /joint_states in your ROS 2 node.

📚 Sources & References
📦 Firmware adapted from: https://github.com/grzesiek2201/servobot_pico_driver

📡 ROS 2 serial library: https://github.com/tylerjw/serial/tree/ros2
