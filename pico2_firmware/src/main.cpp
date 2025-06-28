/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

    
// === Servo Controller Includes ===
#include <PCA9685_servo_driver.h>
#include <PCA9685_servo.h>

// === Globals ===
TaskHandle_t gLEDtask = nullptr;
TaskHandle_t gReceivetask = nullptr;
TaskHandle_t gMoveServotask = nullptr;


// === Servo Controller and Servos ===
PCA9685_servo_driver myController(i2c0, 8, 9, 0x40);

std::vector<PCA9685_servo> myServos = {
    PCA9685_servo(&myController, 0, 100, 540),
    PCA9685_servo(&myController, 1, 100, 540),
    PCA9685_servo(&myController, 2, 100, 540),
};

// === Time Tracking ===
uint64_t TNow = 0;
uint64_t TPrevious = 0;
uint64_t TEllapsed = 0;

// Input buffer
std::string input_line;
std::string msg;


constexpr int RECEIVE_DELAY_MS = 3;

// === Servo Initialization Helper ===
void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange,
                int16_t maxRange, int16_t position, uint8_t address,
                uint64_t TConstDur)
{
    servo.setRange(minRange, maxRange);
    servo.setMode(mode);
    servo.setPosition(position);  // Move to mid-point
    servo.setAddress(address);
    servo.setTConstantDuration(TConstDur);
}

void reset_all_servos_to_zero() {
    for (auto& servo : myServos) {
        servo.setPosition(0);
    }
    printf("Reset: All servos set to 0° (0 radians).\n");
}

// Which core to run on if configNUMBER_OF_CORES==1
#ifndef RUN_FREE_RTOS_ON_CORE
#define RUN_FREE_RTOS_ON_CORE 0
#endif

// Whether to flash the led
#ifndef USE_LED
#define USE_LED 1
#endif

// Whether to busy wait in the led thread
#ifndef LED_BUSY_WAIT
#define LED_BUSY_WAIT 0
#endif

// Delay between led blinking
#define LED_DELAY_MS 500

// Priorities of our threads - higher numbers are higher priority
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )
#define RECEIVE_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define SERVO_TASK_PRIORITY    ( tskIDLE_PRIORITY + 4UL )

// Stack sizes of our threads in words (4 bytes)
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define RECEIVE_TASK_STACK_SIZE configMINIMAL_STACK_SIZE * 2
#define SERVO_TASK_STACK_SIZE configMINIMAL_STACK_SIZE * 2


#if USE_LED
// Turn led on or off
static void pico_set_led(bool led_on) {
#if defined PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

// Initialise led
static void pico_init_led(void) {
#if defined PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    hard_assert(cyw43_arch_init() == PICO_OK);
    pico_set_led(false); // make sure cyw43 is started
#endif
}

void  HeartbeatLEDTask(__unused void *params) {
    static int last_core_id = -1;
    bool on = false;
    printf("blink_task starts\n");
    pico_init_led();
    while (true) {
        last_core_id = portGET_CORE_ID();
#if configNUMBER_OF_CORES > 1        
        if (portGET_CORE_ID() != last_core_id) {            
            printf("first blink task is on core %d\n", last_core_id);
        }
#endif
        // printf("Blink =%s on core %d\n", on ? "true" : "false", last_core_id);
        pico_set_led(on);
        on = !on;

#if LED_BUSY_WAIT
        // You shouldn't usually do this. We're just keeping the thread busy,
        // experiment with BLINK_TASK_PRIORITY and LED_BUSY_WAIT to see what happens
        // if BLINK_TASK_PRIORITY is higher than TEST_TASK_PRIORITY main_task won't get any free time to run
        // unless configNUMBER_OF_CORES > 1
        busy_wait_ms(LED_DELAY_MS);
#else
        sleep_ms(LED_DELAY_MS);
#endif
    }
}
#endif // USE_LED



void receiveTask(__unused void *param)
{
    int delay = *static_cast<int*>(param);

    printf("USB connected. 3-Servo Control Initialized.\n");
    printf("Usage:\n");
    printf("  m <servo> <angle_in_radians> → Move servo to absolute position.\n");
    printf("  e <servo> → Echo current position in radians.\n");
    printf("  r → Reset all servos to 0°.\n\n");


    while (true) {
        int ch = getchar_timeout_us(0);  // Non-blocking input
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\n' || ch == '\r') {
                if (!input_line.empty()) {
                    msg = input_line;
                    input_line.clear();

                    size_t last = 0;
                    size_t next = msg.find(" ");
                    std::string mode = msg.substr(last, next - last);

                    if (mode == "e") {
                        last = next + 1;
                        int nservo = std::stoi(msg.substr(last));
                        if (nservo >= 0 && static_cast<size_t>(nservo) < myServos.size()) {
                            float angle_rad = myServos[nservo].getPosition() * 3.14159f / 180.0f;
                            printf("Servo %d = %.4f radians\r\n", nservo, angle_rad);
                        }

                    } else if (mode == "m") {
                        last = next + 1;
                        next = msg.find(" ", last);
                        int nservo = std::stoi(msg.substr(last, next - last));

                        last = next + 1;
                        double angle_rad = std::stod(msg.substr(last));
                        int angle_deg = static_cast<int>(angle_rad * 180.0 / 3.14159);

                        if (nservo >= 0 && static_cast<size_t>(nservo) < myServos.size()) {
                            myServos[nservo].setPosition(angle_deg);
                            printf("Moving servo %d to %d degrees (%.2f rad)\r\n",
                                   nservo, angle_deg, angle_rad);
                        }

                    } else if (mode == "r") {
                        reset_all_servos_to_zero();
                    } else {
                        printf("Unknown command: %s\r\n", mode.c_str());
                    }
                }
            } else {
                input_line += static_cast<char>(ch);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(delay)); // maintain cooperative multitasking
    }
}



void servoLoopTask(void *param)
{
    while (true) {
        TNow = time_us_64();
        TEllapsed = TNow - TPrevious;
        TPrevious = TNow;

        for (auto& servo : myServos) {
            servo.loop(TEllapsed);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init_all_servos() {
    int i = 0;
    for (auto& servo : myServos) {
        init_servo(servo, MODE_SCONSTANT, -90, 90, 0, i++, 1000000);
        servo.setAngularVelocity(50);
    }
}


void vLaunch( void) {
// #if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
//     // we must bind the main task to one core (well at least while the init is called)
//     vTaskCoreAffinitySet(task, 2); //  for core 1    
// #endif
    // Create heartbeat LED task
    xTaskCreate(HeartbeatLEDTask, "LEDThread",BLINK_TASK_STACK_SIZE  , nullptr, BLINK_TASK_PRIORITY, &gLEDtask);

    // Create serial receive task
    xTaskCreate(receiveTask, "ReceiveThread", RECEIVE_TASK_STACK_SIZE, (void*)&RECEIVE_DELAY_MS, RECEIVE_TASK_PRIORITY , &gReceivetask);

    // Create servo control loop task
    xTaskCreate(servoLoopTask, "ServoThread", SERVO_TASK_STACK_SIZE, nullptr, SERVO_TASK_PRIORITY , &gMoveServotask);

    // Start FreeRTOS
    vTaskStartScheduler();
}

int main( void )
{
    stdio_init_all();

    myController.begin(100000); // initialize I2C

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }


    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
#if (configNUMBER_OF_CORES > 1)
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

    init_all_servos();

#if (configNUMBER_OF_CORES > 1)
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif (RUN_FREE_RTOS_ON_CORE == 1 && configNUMBER_OF_CORES==1)
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}
