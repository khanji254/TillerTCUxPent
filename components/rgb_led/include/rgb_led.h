/**
 * @file rgb_led.h
 * @brief RGB LED Alert System for KSC TCU V1.1
 *
 * Replaces the buzzer-tone alert system with visual LED alerts using
 * three independent GPIO-driven LEDs:
 *
 *   Red   (GPIO1)  — Power / fault indicator
 *   Green (GPIO2)  — Normal operation heartbeat
 *   Blue  (GPIO42) — MQTT / RPC activity indicator
 *
 * A background FreeRTOS task drives LED blink patterns.  Callers set
 * the current alert mode; the task handles timing autonomously so that
 * application code is never blocked waiting for an LED sequence.
 *
 * Alert Mode Priority (highest first):
 *   1. ALERT_NO_BATTERIES   — Red rapid blink 500 ms, others OFF
 *   2. ALERT_LOW_SOC_CRIT   — Red+Blue rapid blink 500 ms (SOC < 10%)
 *   3. ALERT_LOW_SOC_WARN   — Red+Blue slow blink 1000 ms (SOC < 20%)
 *   4. ALERT_RPC_RECEIVED   — Blue rapid blink 500 ms for 5 s, then resume
 *   5. ALERT_TRIKE_OFF      — Red steady ON, Green OFF, Blue OFF
 *   6. ALERT_NORMAL         — Green 1 s heartbeat, Red steady ON, Blue OFF
 *   7. ALERT_BOOT           — All LEDs ON briefly during init
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef RGB_LED_H_
#define RGB_LED_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * ALERT MODES
 * ========================================================================= */

/**
 * @brief System alert modes ordered by severity (highest first).
 *
 * The LED task always displays the highest active priority alert.
 */
typedef enum {
    ALERT_BOOT           = 0, /**< All LEDs ON — system initialising          */
    ALERT_NORMAL         = 1, /**< Green steady             */
    ALERT_TRIKE_OFF      = 2, /**< Red steady; Green OFF — trike powered off   */
    ALERT_RPC_RECEIVED   = 3, /**< Blue rapid blink 500 ms for 5 s then resume */
    ALERT_LOW_SOC_WARN   = 4, /**< Red+Blue 2000 ms blink — avg SOC < 20%      */
    ALERT_LOW_SOC_CRIT   = 5, /**< Red+Blue 1000 ms blink  — avg SOC < 10%      */
    ALERT_NO_BATTERIES   = 6, /**< Red rapid 500 ms blink — no BMS readable    */
    ALERT_VOLTAGE_ANOMALY= 7, /**< Red+Blue 500 ms rapid blink                 */
    ALERT_TRIKE_STOPPING = 8,  /* Trike moving, power off queued; blue fast blink, others off */
} rgb_alert_mode_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise GPIO pins and start the LED management task.
 *
 * Must be called once during system startup before any other rgb_led_*
 * function.  Sets Red LED ON (power indicator), Green and Blue OFF.
 *
 * @return ESP_OK on success, ESP_FAIL if the FreeRTOS task could not
 *         be created.
 */
esp_err_t rgb_led_init(void);

/**
 * @brief Set the current system alert mode.
 *
 * Thread-safe.  The LED task picks up the new mode within one tick.
 *
 * @param[in] mode  Alert mode to activate.
 */
//bool rgb_led_set_alert(rgb_alert_mode_t mode);
bool rgb_led_set_alert(rgb_alert_mode_t mode, bool force); 

/**
 * @brief Trigger a temporary RPC-received blink sequence.
 *
 * Blue LED blinks at 500 ms for 5 seconds then the LED task reverts to
 * the previous alert mode.  Can be called from any task or ISR context.
 */
void rgb_led_notify_rpc(void);

/**
 * @brief Directly set individual LED states (for factory test use only).
 *
 * Bypasses the alert state machine.  Call rgb_led_set_alert() to resume
 * normal managed operation after testing.
 *
 * @param[in] red    true = ON, false = OFF
 * @param[in] green  true = ON, false = OFF
 * @param[in] blue   true = ON, false = OFF
 */
void rgb_led_set_raw(bool red, bool green, bool blue);

/**
 * @brief Return the currently active alert mode.
 *
 * @return Current rgb_alert_mode_t value.
 */
rgb_alert_mode_t rgb_led_get_alert(void);

/**
 * @brief Convert an alert state to a human-readable string.
 * @param[in] alert  Alert value to convert.
 * @return Static string — do not free.
 */
const char *rgb_led_alert_to_str(rgb_alert_mode_t alert);

#ifdef __cplusplus
}
#endif

#endif /* RGB_LED_H_ */
