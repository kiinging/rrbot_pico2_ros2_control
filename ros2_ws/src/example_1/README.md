
# 🤖 RRBot Serial Hardware Interface – Example 1

This example demonstrates the **RRBot** (Revolute-Revolute Robot) controlled via a real hardware interface using ROS 2 and the `ros2_control` framework.

---

## 🛠️ Overview

This demo shows how to connect a 3-joint robotic arm to ROS 2 using serial communication with a **Raspberry Pi Pico 2 (RP2040)**. Two main controller configurations are shown:

- `forward_position_controller`
- `joint_trajectory_controller`

---

## ⚙️ Hardware Interface Options

### Simulation (Fake Hardware)
- File: `hardware/rrbot.cpp`
- Use this when testing with simulated feedback (no hardware required)

### Real Hardware (Serial Interface)
- File: `hardware/rrbot_serial.cpp`
- Uses a serial protocol to communicate with the Pico firmware

---

## 🚀 Launching Controllers

### 1. Forward Position Controller

Start the robot with the position controller:

```bash
ros2 launch ros2_control_demo_example_1 rrbot.serial.launch.py
```

Then send joint commands via:

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 1.0]"
```

---

### 2. Joint Trajectory Controller

Launch the trajectory controller version:

```bash
ros2 launch ros2_control_demo_example_1 rrbot.serial.trajectory.launch.py
```

Then run a test trajectory plan:

```bash
ros2 launch ros2_control_demo_example_1 test_joint_trajectory_controller.launch.py
```

---

## 🔌 Serial Protocol Summary

The hardware interface expects the Pico to respond to and send messages like:

- `m <index> <radians>` – Move servo
- `j <index> <radians>` – Feedback from Pico
- `e <index>` – Echo current position
- `r` – Reset all servos to 0

Make sure your firmware implements this protocol and echoes joint angles regularly.

---

## 📚 Resources

- ROS 2 Control Demos:  
  [https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html](https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html)

- PCA9685 Servo Driver Firmware:  
  [https://github.com/grzesiek2201/servobot_pico_driver](https://github.com/grzesiek2201/servobot_pico_driver)

---

🧪 Whether you're testing in simulation or running real hardware — this example is a solid launchpad for embedded robotics with ROS 2. Enjoy building! 🔧🤖✨