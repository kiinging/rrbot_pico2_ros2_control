
  
#include "PCA9685_servo_driver.h"
#include "PCA9685_servo.h"
#include <pico/stdlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#define LED_PIN 25

// Create the controller and servos
PCA9685_servo_driver myController(i2c0, 8, 9, 0x40);

// 3 servos on channels 0–2
std::vector<PCA9685_servo> myServo = {
    PCA9685_servo(&myController, 0, 100, 540),
    PCA9685_servo(&myController, 1, 100, 540),
    PCA9685_servo(&myController, 2, 100, 540)
};

// Time tracking
uint64_t TNow = 0;
uint64_t TPrevious = 0;
uint64_t TEllapsed = 0;

// Input buffer
std::string input_line;
std::string msg;

void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange,
                int16_t maxRange, int16_t position, uint8_t address,
                uint64_t TConstDur) {
    servo.setRange(minRange, maxRange);
    servo.setMode(mode);
    servo.setPosition(position); // Absolute angle (degrees)
    servo.setAddress(address);
    servo.setTConstantDuration(TConstDur);
}

void reset_all_servos_to_zero() {
    for (auto& servo : myServo) {
        servo.setPosition(0);
    }
    printf("Reset: All servos set to 0° (0 radians).\n");
}

int main(void) {
    stdio_init_all();

    // Wait for USB serial to be connected (only on USB, not UART)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("USB connected. 3-Servo Control Initialized.\n");
    printf("Usage:\n");
    printf("  m <servo> <angle_in_radians> → Move servo to absolute position.\n");
    printf("  e <servo> → Echo current position in radians.\n");
    printf("  r → Reset all servos to 0°.\n\n");


    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    myController.begin(100000); // 100kHz I2C

    int i = 0;
    for (auto& servo : myServo) {
        init_servo(servo, MODE_SCONSTANT, -90, 90, 0, i++, 1000000);  // Set to 0°
        servo.setAngularVelocity(50); // degrees per second
    }

    sleep_ms(3000);
    printf("Ready. Type m <n> <radians>, e <n>, or r + ENTER\n");

    while (true) {
        // Time update
        TNow = time_us_64();
        TEllapsed = TNow - TPrevious;
        TPrevious = TNow;

        // LED blink
        static bool led_state = false;
        static uint64_t last_blink = 0;
        if (TNow - last_blink > 500000) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            last_blink = TNow;
        }

        // Servo update loop
        for (auto& servo : myServo) {
            servo.loop(TEllapsed);
        }

        // Serial input
        int ch = getchar_timeout_us(0);  // non-blocking
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                if (!input_line.empty()) {
                    msg = input_line;
                    input_line.clear();

                    size_t last = 0;
                    size_t next = msg.find(" ");
                    std::string mode = msg.substr(last, next - last);

                    if (mode == "e") {
                        last = next + 1;
                        int nservo = std::stoi(msg.substr(last));
                        if (nservo >= 0 && static_cast<size_t>(nservo) < myServo.size()) {
                            float angle_rad = myServo[nservo].getPosition() * 3.14159f / 180.0f;
                            printf("Servo %d = %.4f radians\r\n", nservo, angle_rad);
                        }

                    } else if (mode == "m") {
                        last = next + 1;
                        next = msg.find(" ", last);
                        int nservo = std::stoi(msg.substr(last, next - last));

                        last = next + 1;
                        double angle_rad = std::stod(msg.substr(last));
                        int angle_deg = static_cast<int>(angle_rad * 180.0 / 3.14159);

                        if (nservo >= 0 && static_cast<size_t>(nservo) < myServo.size()) {
                            myServo[nservo].setPosition(angle_deg);
                            printf("Moving servo %d to %d degrees (%.2f rad)\r\n",
                                   nservo, angle_deg, angle_rad);
                        }

                    } else if (mode == "r") {
                        reset_all_servos_to_zero();
                    } else {
                        printf("Unknown command: %s\n", mode.c_str());
                    }
                }
            } else {
                input_line += static_cast<char>(ch);
            }
        }
    }

    return 0;
}
