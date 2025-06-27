#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();


    // Wait for USB serial connection (optional: add timeout)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf(">> Ready for commands like 'servo:1:90'\n");

    char buf[100];
    while (true) {
        int i = 0;
        while (true) {
            int ch = getchar_timeout_us(0);
            if (ch == PICO_ERROR_TIMEOUT) continue;

            if (ch == '\n' || ch == '\r') {
                buf[i] = 0;
                break;
            } else {
                buf[i++] = ch;
                if (i >= 99) break;
            }
        }

        printf("Received: %s\r\n", buf);

        // Simple parsing
        if (strncmp(buf, "servo:", 6) == 0) {
            char* token = strtok(buf + 6, ":");
            if (token) {
                int channel = atoi(token);
                token = strtok(NULL, ":");
                if (token) {
                    int angle = atoi(token);
                    printf("Parsed command -> Channel: %d, Angle: %d\n", channel, angle);
                    // You could now call set_servo_angle(channel, angle);
                } else {
                    printf("⚠️ Invalid format: missing angle\n");
                }
            } else {
                printf("⚠️ Invalid format: missing channel\n");
            }
        } else {
            printf("⚠️ Unknown command\n");
        }
    }
}
