#include "PCA9685_servo_driver.h"
#include "PCA9685_servo.h"
#include <pico/stdlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#define LED_PIN 25

void StartMoveHandler(uint16_t Address);	// Servo callback
void StopMoveHandler(uint16_t Address);		// Servo callback
void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange, int16_t maxRange, int16_t position, uint8_t address, uint64_t TConstDur);

// create the controller and servos
PCA9685_servo_driver myController(i2c0, 8, 9, 0x40);

// ✅ Two servos: channel 0 and channel 1
std::vector<PCA9685_servo> myServo = {
    PCA9685_servo(&myController, 0, 100, 540), // Servo on channel 0 
    PCA9685_servo(&myController, 1, 100, 540)  // Servo on channel 1
};

// necessary variables to drive the servo with constant speed or time
uint64_t TNow = 0;
uint64_t TPrevious = 0;
uint64_t TEllapsed = 0;


int main(void)
{
    // initialize Pi Pico peripherals
    stdio_init_all();

    printf("Starting Dual Servo Opposite Motion\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // create connection to the PCA9685 and initialize it
    myController.begin(100000); // 100kHz I2C

    // Initialize each servo
    int i = 0;
    for (auto& servo : myServo) {
        init_servo(servo, MODE_SCONSTANT, -90, 90, 0, i++, 1000000);
        servo.setAngularVelocity(90); // 90° per second
    }

    sleep_ms(1000); // Delay for servo power-on stability

    bool direction = false; // false: move to -90, true: move to +90
    uint64_t last_move_time = time_us_64();

while(1)
{
    TNow = time_us_64();
    TEllapsed = TNow - TPrevious;
    TPrevious = TNow;

    // Blink LED every 500ms
    static bool led_state = false;
    static uint64_t last_blink = 0;
    if (TNow - last_blink > 500000) {
        led_state = !led_state;
        gpio_put(LED_PIN, led_state);
        last_blink = TNow;
    }

    // Update servos
    for(auto& servo : myServo){
        servo.loop(TEllapsed);
    }

            // Move every 5 seconds
    if (TNow - last_move_time > 5 * 1e6) {
        if (direction) {
            // Servo 0 to +90, Servo 1 to -90
            myServo[0].setRelativePosition(180);  // +90°
            myServo[1].setRelativePosition(-180); // -90°
            printf("Servo 0 -> +90, Servo 1 -> -90\n");
        } else {
            // Servo 0 to -90, Servo 1 to +90
            myServo[0].setRelativePosition(-180);  // -90°
            myServo[1].setRelativePosition(180);   // +90°
            printf("Servo 0 -> -90, Servo 1 -> +90\n");
        }
        direction = !direction;
        last_move_time = TNow;
    }
}


    return 0;
}

void StartMoveHandler(uint16_t Address)
{
    // called when a servo starts to move
    return;
}

void StopMoveHandler(uint16_t Address)
{
    // called when a servo stops moving
	return;
}

void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange, int16_t maxRange, int16_t position, uint8_t address, uint64_t TConstDur)
{
    servo.setRange(minRange, maxRange);
    servo.setMode(mode);
    servo.setPosition(position); // move to mid point
    servo.setAddress(address);
    servo.setTConstantDuration(TConstDur);
}
