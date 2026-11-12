#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/usb_serial_jtag_ll.h"
#include "pins/pins.h"
#include "test_trike_power.h"

static const char *TAG = "TEST_TRIKE_SERIAL";

// Fallback in case it's not yet in pins/pins.h
#ifndef PIN_KILL_SWITCH
#define PIN_KILL_SWITCH 21
#endif

static int get_serial_char(void)
{
    uint8_t usb_ch = 0;
    if (usb_serial_jtag_ll_read_rxfifo(&usb_ch, 1) > 0) {
        return (int)usb_ch;
    }
    int c = fgetc(stdin);
    if (c != EOF) {
        return c;
    }
    return -1;
}

void test_trike_power_run(void)
{
    // Disable stdin buffering so UART input (if used) arrives immediately
    setvbuf(stdin, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "Initializing Serial-Controlled Kill Switch on GPIO %d...", PIN_KILL_SWITCH);

    // 1. Configure the GPIO for the BJT base
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_KILL_SWITCH),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Ensure we start in a safe (OFF) state
    gpio_set_level(PIN_KILL_SWITCH, 0);

    ESP_LOGW(TAG, "===============================================");
    ESP_LOGW(TAG, "Send '1' in the serial monitor to turn ON");
    ESP_LOGW(TAG, "Send '0' in the serial monitor to turn OFF");
    ESP_LOGW(TAG, "===============================================");

    // 2. Infinite loop to read from serial monitor
    while (1) {
        int c = get_serial_char();

        if (c != -1) {
            if (c == '1') {
                gpio_set_level(PIN_KILL_SWITCH, 1);
                ESP_LOGW(TAG, "Kill Switch ON  [GPIO %d HIGH -> BJT Saturated]", PIN_KILL_SWITCH);
            } 
            else if (c == '0') {
                gpio_set_level(PIN_KILL_SWITCH, 0);
                ESP_LOGI(TAG, "Kill Switch OFF [GPIO %d LOW  -> BJT Cutoff]", PIN_KILL_SWITCH);
            }
            else if (c != '\r' && c != '\n') {
                ESP_LOGI(TAG, "Received '%c' (press '1' for ON, '0' for OFF)", (char)c);
            }
        }

        // Small delay to yield to the FreeRTOS watchdog and prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}