/**
 * @file bms_rs485_mux.h
 * @brief RS-485 Battery MUX Control — MC74HC4052ADG Analog Multiplexer
 *
 * Controls the MC74HC4052ADG dual 4-channel analog multiplexer that
 * routes the single RS-485 UART bus to one of three JK-BMS batteries.
 *
 * Channel selection is achieved via two GPIO lines (A and B):
 *
 *   A=0, B=0 → Battery 1 (BMS_BATTERY_1)
 *   A=1, B=0 → Battery 2 (BMS_BATTERY_2)
 *   A=0, B=1 → Battery 3 (BMS_BATTERY_3)
 *   A=1, B=1 → All channels disconnected (idle / safe state)
 *
 * All pin assignments are sourced from board_config.h.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef BMS_RS485_MUX_H_
#define BMS_RS485_MUX_H_

#include "bms_monitor_types.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise MUX select GPIOs and place bus in idle (disconnected) state.
 *
 * Configures BOARD_MUX_SEL_A_PIN and BOARD_MUX_SEL_B_PIN as outputs and
 * drives A=1, B=1 (all channels disconnected) as the safe starting state.
 */
void bms_rs485_mux_init(void);

/**
 * @brief Disconnect all battery channels (A=1, B=1 idle state).
 *
 * Safe to call at any time; ensures no two channels are simultaneously
 * active during transitions.
 */
void bms_rs485_disable_all(void);

/**
 * @brief Switch the RS-485 bus exclusively to the specified battery.
 *
 * Disables all channels first, waits for bus to settle, then enables
 * the requested channel.
 *
 * @param[in] battery_id  Battery to connect (BMS_BATTERY_1..3).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out-of-range ID.
 */
esp_err_t bms_rs485_switch_to_battery(bms_battery_id_t battery_id);

/**
 * @brief Disconnect the specified battery channel.
 *
 * Reverts to idle state (A=1, B=1) after communicating with a battery.
 *
 * @param[in] battery_id  Battery to disconnect.
 * @return ESP_OK on success.
 */
esp_err_t bms_rs485_disable_battery(bms_battery_id_t battery_id);

/**
 * @brief Enable the specified battery channel.
 *
 * Does not disable other channels first — call bms_rs485_disable_all()
 * before this if switching from another channel.
 *
 * @param[in] battery_id  Battery to connect.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for out-of-range ID.
 */
esp_err_t bms_rs485_enable_battery(bms_battery_id_t battery_id);

#ifdef __cplusplus
}
#endif

#endif /* BMS_RS485_MUX_H_ */