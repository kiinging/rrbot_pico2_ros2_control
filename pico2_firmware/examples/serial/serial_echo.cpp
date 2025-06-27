#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    // Wait for USB serial connection (optional: add timeout)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf(">> Serial Echo Test: Type something and press enter\n");

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
        printf("You typed: %s\r\n", buf);
    }
}
