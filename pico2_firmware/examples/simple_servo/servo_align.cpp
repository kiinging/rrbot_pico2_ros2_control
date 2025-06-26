// i m using this code to align all three servo to zero degree.
#include "PCA9685_servo_driver.h"
#include "PCA9685_servo.h"
#include <pico/stdlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#define LED_PIN 25

void StartMoveHandler(uint16_t Address);  // Servo callback
void StopMoveHandler(uint16_t Address);   // Servo callback
void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange, int16_t maxRange, int16_t position, uint8_t address, uint64_t TConstDur);

// Create the controller and servos
PCA9685_servo_driver myController(i2c0, 8, 9, 0x40);

// ✅ Three servos: channels 0, 1, and 2 (all initialized at 0°)
std::vector<PCA9685_servo> myServo = {
    PCA9685_servo(&myController, 0, 100, 540), // Servo 0 (min pulse 100, max pulse 540)
    PCA9685_servo(&myController, 1, 100, 540), // Servo 1
    PCA9685_servo(&myController, 2, 100, 540)  // Servo 2
};

// Variables for timing
uint64_t TNow = 0;
uint64_t TPrevious = 0;
uint64_t TEllapsed = 0;

int main(void) {
    // Initialize Pi Pico peripherals
    stdio_init_all();
    printf("Initializing 3-Servo Arm (All at 0°)\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize PCA9685
    myController.begin(100000); // 100kHz I2C

    // Initialize all servos to 0°
    int i = 0;
    for (auto& servo : myServo) {
        init_servo(servo, MODE_SCONSTANT, -90, 90, 0, i++, 1000000); // 0° position
        servo.setAngularVelocity(90); // 90° per second (adjust if needed)
    }

    // Wait 5 seconds to ensure servos physically reach 0°
    printf("Waiting 5 seconds for servos to align...\n");
    sleep_ms(5000);
    printf("All servos at 0°. Ready for robot arm programming!\n");

    while(1) {
        TNow = time_us_64();
        TEllapsed = TNow - TPrevious;
        TPrevious = TNow;

        // Blink LED every 500ms (optional)
        static bool led_state = false;
        static uint64_t last_blink = 0;
        if (TNow - last_blink > 500000) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            last_blink = TNow;
        }

        // Update servos (maintains 0° position)
        for(auto& servo : myServo) {
            servo.loop(TEllapsed);
        }

        // Add your robot arm movement logic here later
        // Example: myServo[0].setRelativePosition(45); // Move servo 0 to +45°
    }

    return 0;
}

// --- Helper Functions (Unchanged) ---
void StartMoveHandler(uint16_t Address) { return; }
void StopMoveHandler(uint16_t Address) { return; }

void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange, int16_t maxRange, int16_t position, uint8_t address, uint64_t TConstDur) {
    servo.setRange(minRange, maxRange);
    servo.setMode(mode);
    servo.setPosition(position); // Set to 0° initially
    servo.setAddress(address);
    servo.setTConstantDuration(TConstDur);
}