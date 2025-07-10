# 🤖 rrbot_pico2_ros2_control

This repository enables control of a **3-joint robotic arm** using:

- **Raspberry Pi Pico 2 (RP2350)** for low-level firmware
- **ROS 2** (e.g., Jazzy) running on an SBC like Orange Pi 5 Max

---

## 🧩 Key Components

### 🔧 `pico2_firmware/`
Firmware written in C++ for the RP2350 to drive servos using a PCA9685 over I2C. The microcontroller communicates with the ROS 2 stack over serial using a lightweight text-based protocol.

### 🌐 `ros2_ws/src/example_1/`
ROS 2 packages using `ros2_control` framework. Includes hardware interface implementations and launch files for both simulation and real hardware.

---

## 🔌 Serial Communication Protocol

The Pico firmware uses a human-readable serial protocol:
```
m <index> <radians>  → Move servo
e <index>            → Echo servo angle
r                   → Reset all
j <index> <radians>  → Feedback (auto-sent)
```

---

## 📦 Serial Library for ROS 2

We use a ROS 2-compatible version of [wjwwood/serial](https://github.com/tylerjw/serial/tree/ros2). To install:

```bash
cd ~/ros2_projects/ros2_ws/src
git clone -b ros2 https://github.com/tylerjw/serial.git
cd ..
colcon build
source install/setup.bash
```

Now your ROS 2 code can easily communicate with the Pico via `/dev/ttyACM0`.

---

## 🔗 Resources

- Servo firmware base: [grzesiek2201/servobot_pico_driver](https://github.com/grzesiek2201/servobot_pico_driver)
- ROS 2 serial library: [tylerjw/serial/tree/ros2](https://github.com/tylerjw/serial/tree/ros2)

---

🍕 Built with microcontrollers and mechatronics love.