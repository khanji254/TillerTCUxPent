/**
 * @file bms_rs485_mux.c
 * @brief RS-485 Battery MUX Control — MC74HC4052ADG Implementation
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "bms_rs485_mux.h"
#include "board_config.h"
#include "rgb_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "BMS_MUX"

/* =========================================================================
 * MUX CHANNEL TABLE
 * =========================================================================
 * Maps battery ID to (A, B) select line state.
 * A=1, B=1 is the idle/disconnected state.
 * ========================================================================= */

typedef struct {
    uint8_t sel_a;
    uint8_t sel_b;
} mux_channel_t;

/*1 BATTERY SETUP*/
/*static const mux_channel_t MUX_CHANNELS[BMS_BATTERY_COUNT] = {
    [BMS_BATTERY_1] = {.sel_a = 0, .sel_b = 0},
};*/

/*2 BATTERY SETUP*/
static const mux_channel_t MUX_CHANNELS[BMS_BATTERY_COUNT] = {
    [BMS_BATTERY_1] = {.sel_a = 0, .sel_b = 0},
    [BMS_BATTERY_2] = {.sel_a = 1, .sel_b = 0},
};


/*3 BATTERY SETUP*/
/*static const mux_channel_t MUX_CHANNELS[BMS_BATTERY_COUNT] = {
    [BMS_BATTERY_1] = {.sel_a = 0, .sel_b = 0},
    [BMS_BATTERY_2] = {.sel_a = 1, .sel_b = 0},
    [BMS_BATTERY_3] = {.sel_a = 0, .sel_b = 1},
};*/

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

void bms_rs485_mux_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_MUX_SEL_A_PIN) |
                        (1ULL << BOARD_MUX_SEL_B_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Set idle state: A=1, B=1 → no channel selected */
    gpio_set_level(BOARD_MUX_SEL_A_PIN, 1);
    gpio_set_level(BOARD_MUX_SEL_B_PIN, 1);

    ESP_LOGI(TAG, "RS-485 MUX initialised (A=IO%d B=IO%d) — all channels idle",
             BOARD_MUX_SEL_A_PIN, BOARD_MUX_SEL_B_PIN);
}

void bms_rs485_disable_all(void)
{
    /* A=1, B=1 → MC74HC4052ADG disconnects all channels */
    gpio_set_level(BOARD_MUX_SEL_A_PIN, 1);
    gpio_set_level(BOARD_MUX_SEL_B_PIN, 1);
}

esp_err_t bms_rs485_enable_battery(bms_battery_id_t battery_id)
{
    if (battery_id >= BMS_BATTERY_COUNT) {
        ESP_LOGE(TAG, "Invalid battery ID: %d", (int)battery_id);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(BOARD_MUX_SEL_A_PIN, MUX_CHANNELS[battery_id].sel_a);
    gpio_set_level(BOARD_MUX_SEL_B_PIN, MUX_CHANNELS[battery_id].sel_b);

    /* Allow signal to settle through mux switches (~10 µs typical;
     * use a 10 ms delay for conservative margin with RS-485 transceiver) */
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGD(TAG, "MUX → Battery %d  (A=%d B=%d)",
             battery_id + 1,
             MUX_CHANNELS[battery_id].sel_a,
             MUX_CHANNELS[battery_id].sel_b);
    return ESP_OK;
}

esp_err_t bms_rs485_disable_battery(bms_battery_id_t battery_id)
{
    if (battery_id >= BMS_BATTERY_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    bms_rs485_disable_all();
    return ESP_OK;
}

esp_err_t bms_rs485_switch_to_battery(bms_battery_id_t battery_id)
{
    if (battery_id >= BMS_BATTERY_COUNT) {
        ESP_LOGE(TAG, "Invalid battery ID: %d", (int)battery_id);
        return ESP_ERR_INVALID_ARG;
    }

    /* Disconnect all channels first to prevent bus contention */
    bms_rs485_disable_all();
    vTaskDelay(pdMS_TO_TICKS(5));

    return bms_rs485_enable_battery(battery_id);
}