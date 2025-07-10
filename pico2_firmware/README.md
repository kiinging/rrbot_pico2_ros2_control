# 🔧 pico2_firmware — RP2040 Servo Control

This firmware enables the **Raspberry Pi Pico 2 (RP2040)** to control servo motors using a **PCA9685 PWM driver**, communicating via serial UART with a host running ROS 2.

---

## 🧠 Features

- UART command parser for ROS 2 integration
- Joint position feedback via serial
- Lightweight driver for Adafruit PCA9685 PWM chip
- Built with the official RP2040 C/C++ SDK

---

## 📡 Serial Protocol

The firmware listens for simple commands like:

```
m 0 1.57     → Move joint 0 to 1.57 rad
e 0          → Echo current position of joint 0
r            → Reset all joints to 0
j 1 0.78     → Feedback (auto-sent to host)
```

---

## 📂 Structure

```
pico2_firmware/
├── examples/            # e.g. simple_servo test
├── include/             # Headers, drivers
├── src/                 # Main firmware source
├── build/               # (build artifacts)
└── CMakeLists.txt
```

---

## 🚀 Simple Servo Example

This test toggles one servo smoothly back and forth.

### 🧪 Build it

```bash
mkdir -p build
cd build
cmake -DPICO_BOARD=pico2 ..
make
```

This generates:

- `servo_driver.elf`
- `servo_driver.uf2`

### ⚡ Flash via USB

1. Hold **BOOTSEL**
2. Plug in Pico 2 via USB
3. Drag `servo_driver.uf2` onto `RPI-RP2`

### 🔍 Flash via OpenOCD (optional)

```bash
sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 5000" \
  -c "program servo_driver.elf verify reset exit"
```

---

## 🔗 Credits

- Forked from: [Adafruit-Servo-Driver-Library-Pi-Pico](https://github.com/grzesiek2201/Adafruit-Servo-Driver-Library-Pi-Pico)