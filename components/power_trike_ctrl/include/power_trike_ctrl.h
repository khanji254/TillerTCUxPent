/**
 * @file power_trike_ctrl.h
 * @brief Power Management and Trike Power Control — Combined Module
 *
 * Merges the previously separate power_management and trike_power_ctrl
 * modules into a single cohesive component for the TCU V1.1 board.
 *
 * Responsibilities:
 *   - Mains power detection and loss notification via FreeRTOS event queue
 *   - Trike power ON/OFF pipeline with motion-aware delayed shutoff
 *   - NVS persistence of trike power state (survives reboots)
 *   - Modem brownout detection and recovery
 *   - Reset reason reporting
 *
 * Removed from V1.0:
 *   - Button detection (no button on V1.1 board)
 *   - NVS triple-press reset (no button; removed per design decision)
 *   - Backup battery ADC monitoring (not on TCU hardware)
 *   - MAINS_STATUS pin (removed in V1.1 hardware)
 *
 * ThingsBoard JSON field mapping:
 *   "1"  — Mains power present (0/1)
 *   "9"  — trike_power_resp_cmd: true = command queued (trike in motion),
 *                                 false = poweroff executed
 *   "10" — power_confirmation:   true = trike ON, false = trike OFF
 *   "11" — Reset reason string (omitted on normal power-on reset)
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef POWER_TRIKE_CTRL_H_
#define POWER_TRIKE_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * CONFIGURATION
 * ========================================================================= */

/** FreeRTOS event queue depth */
#define POWER_EVENT_QUEUE_SIZE          5

/** Maximum time (ms) the wait-for-stop task will wait before forcing
 *  trike poweroff regardless of motion state */
#define TRIKE_MAX_WAIT_FOR_STOP_MS      (30UL * 60UL * 1000UL)

/* =========================================================================
 * EVENT TYPES
 * ========================================================================= */

/**
 * @brief Power system event types posted to the event queue.
 */
typedef enum {
    POWER_EVENT_MODEM_BROWNOUT  = 0,   /**< Modem stopped responding         */
} power_event_type_t;

/**
 * @brief Power event structure for the FreeRTOS event queue.
 */
typedef struct {
    power_event_type_t type;
    uint32_t           timestamp_ms;
    union {
        struct {
            bool mains_present;
        } mains;
    } data;
} power_event_t;

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

/**
 * @brief Initialise power management and trike control.
 *
 * Configures mains detection GPIO, trike power GPIO, modem control GPIOs,
 * creates the event queue and monitoring task, and loads trike
 * power_confirmation state from NVS.
 *
 * Must be called after nvs_flash_init() and before any MQTT task starts.
 *
 * @return ESP_OK on success.
 */
esp_err_t power_trike_init(void);

/* =========================================================================
 * MAINS POWER API
 * ========================================================================= */
/**
 * @brief Get the power event queue handle.
 *
 * @return FreeRTOS queue handle for power events.
 */
QueueHandle_t power_mgmt_get_event_queue(void);

/**
 * @brief Return true if the last reset was a clean power-on reset.
 *
 * @return true if ESP_RST_POWERON.
 */
bool power_mgmt_is_power_on_reset(void);

/**
 * @brief Get reset reason as a short string for ThingsBoard field "11".
 *
 * @param[out] buffer  Buffer to receive the string.
 * @param[in]  size    Buffer size.
 * @return Number of characters written.
 */
int power_mgmt_get_reset_reason(char *buffer, size_t size);

/* =========================================================================
 * MODEM HEALTH API
 * ========================================================================= */

/**
 * @brief Test whether the modem responds to an AT command.
 *
 * Acquires uart_modem_mutex internally.
 *
 * @return true if modem responds, false if not.
 */
bool power_mgmt_check_modem_alive(void);

/**
 * @brief Attempt modem recovery (reset then full power cycle if needed).
 *
 * @return ESP_OK if recovery succeeded.
 */
esp_err_t power_mgmt_recover_modem(void);

/* =========================================================================
 * TRIKE POWER CONTROL API
 * ========================================================================= */

/**
 * @brief Handle a trike power command received via MQTT RPC.
 *
 * Routes the command through the motion-aware pipeline:
 *   trike_ctrl_value = 1 → Power OFF (with motion check)
 *   trike_ctrl_value = 2 → Power ON
 *
 * @param[in] trike_ctrl_value  1 = OFF command, 2 = ON command.
 */
void trike_ctrl_handle_command(uint8_t trike_ctrl_value);

/**
 * @brief Get the current NVS-backed power_confirmation state.
 *
 * @return true = trike ON, false = trike OFF.
 */
bool trike_ctrl_get_power_confirmation(void);

/**
 * @brief Save power_confirmation to NVS (wear-protected).
 *
 * @param[in] value  true = ON, false = OFF.
 */
void trike_ctrl_save_nvs(bool value);

/**
 * @brief Check if trike_power_resp_cmd field "9" should be included
 *        in the next metadata publish, and retrieve its value.
 *
 * Calling this function clears the pending flag; field "9" is only
 * ever included in the publish immediately following a power-off event.
 *
 * @param[out] resp_cmd_value  Value for field "9" (true = still moving,
 *                              false = poweroff executed).
 * @return true if the field should be included, false otherwise.
 */
bool trike_ctrl_consume_resp_cmd_flag(bool *resp_cmd_value);

/**
 * @brief Publish the current trike power state to ThingsBoard.
 *
 * Used at boot to restore the ThingsBoard slider widget state.
 * Publishes field "10" only (no field "9").
 */
void trike_ctrl_publish_state(void);

/**
 * @brief Assert TRIKE_PWR_CTRL GPIO to physically cut trike power.
 *
 * Called by the power-off pipeline after publishing final state.
 */
void power_mgmt_trike_poweroff(void);

/**
 * @brief De-assert TRIKE_PWR_CTRL GPIO to restore trike power.
 */
void power_mgmt_trike_poweron(void);

/**
 * @brief Set TRIKE_PWR_CTRL GPIO to the NVS-restored level at boot.
 *
 * Called once by power_trike_init() after loading power_confirmation
 * from NVS.  Does not trigger any MOSFET or publish any state.
 *
 * @param[in] trike_on  true = GPIO HIGH (ON), false = GPIO LOW (OFF).
 */
void power_mgmt_trike_set_gpio(bool trike_on);

#ifdef __cplusplus
}
#endif

#endif /* POWER_TRIKE_CTRL_H_ */
