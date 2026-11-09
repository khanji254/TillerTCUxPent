/**
 * @file rgb_led.c
 * @brief RGB LED Alert System Implementation for KSC TCU V1.1
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "rgb_led.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>

#define TAG "RGB_LED"

/** Minimum time (ms) a high-priority alert holds before lower ones can overwrite */
#define LED_PRIORITY_HOLDOFF_MS   3000   /* 3 s — covers one publish cycle gap */

static volatile uint32_t s_alert_set_time_ms = 0;  /* When current alert was set */


/* =========================================================================
 * TIMING CONSTANTS (milliseconds)
 * ========================================================================= */

#define LED_HEARTBEAT_INTERVAL_MS   1000   /**< Green heartbeat period      */
#define LED_SLOW_BLINK_MS           2000   /**< Slow blink half-period      */
#define LED_FAST_BLINK_MS           1000   /**< Fast blink half-period      */
#define LED_RAPID_BLINK_MS           500   /**< Rapid blink half-period     */
#define LED_RPC_DURATION_MS         5000   /**< Blue blink after RPC        */
#define LED_TASK_TICK_MS              50   /**< Task polling resolution     */

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static volatile rgb_alert_mode_t s_alert_mode       = ALERT_BOOT;
static volatile bool             s_rpc_pending       = false;
static volatile uint32_t         s_rpc_start_ms      = 0;
static SemaphoreHandle_t         s_mutex             = NULL;

/* =========================================================================
 * INTERNAL HELPERS
 * ========================================================================= */

/**
 * @brief Set physical LED GPIO levels.
 *
 * @param[in] red    1 = ON, 0 = OFF
 * @param[in] green  1 = ON, 0 = OFF
 * @param[in] blue   1 = ON, 0 = OFF
 */
static inline void set_leds(uint8_t red, uint8_t green, uint8_t blue)
{
    gpio_set_level(BOARD_LED_RED_PIN,   red);
    gpio_set_level(BOARD_LED_GREEN_PIN, green);
    gpio_set_level(BOARD_LED_BLUE_PIN,  blue);
}

/* =========================================================================
 * LED MANAGEMENT TASK
 * ========================================================================= */

/**
 * @brief Background task that drives LED blink patterns.
 *
 * Runs at 50 ms resolution.  Implements all alert mode patterns
 * without blocking any application task.
 *
 * @param[in] pvParameters  Unused.
 */
static void rgb_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED task started on Core %d", xPortGetCoreID());

    uint32_t tick_count = 0;  /* Counts 50 ms ticks                         */
    bool     blink_state = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_TICK_MS));
        tick_count++;

        uint32_t now_ms = tick_count * LED_TASK_TICK_MS;

        /* Read current mode (volatile read is safe without mutex here) */
        rgb_alert_mode_t mode = s_alert_mode;

        /* ----------------------------------------------------------------
         * Temporary RPC override: blue blink for LED_RPC_DURATION_MS
         * ---------------------------------------------------------------- */
        if (s_rpc_pending) {
            uint32_t elapsed = now_ms - s_rpc_start_ms;
            if (elapsed < LED_RPC_DURATION_MS) {
                /* Blue blink at LED_FAST_BLINK_MS regardless of base mode */
                bool blue_on = ((tick_count % (LED_FAST_BLINK_MS / LED_TASK_TICK_MS)) <
                                (LED_FAST_BLINK_MS / LED_TASK_TICK_MS / 2));
                set_leds(1, 0, blue_on ? 1 : 0);
                continue;
            } else {
                s_rpc_pending = false;
            }
        }

        /* ----------------------------------------------------------------
         * Main alert pattern selection
         * ---------------------------------------------------------------- */
        switch (mode) {

            case ALERT_BOOT:
                /* All LEDs ON during boot */
                set_leds(1, 1, 1);
                break;

            case ALERT_NORMAL:
                /* Green steady ON; Red, Blue OFF */
                blink_state = ((tick_count % (LED_HEARTBEAT_INTERVAL_MS /
                                LED_TASK_TICK_MS)) <
                               (LED_HEARTBEAT_INTERVAL_MS / LED_TASK_TICK_MS / 2));
                set_leds(0,  1 , 0);               
                //set_leds(1, blink_state ? 1 : 0, 0);
                break;

            case ALERT_TRIKE_OFF:
                /* Red steady ON; Green and Blue OFF */
                set_leds(1, 0, 0);
                break;

            case ALERT_RPC_RECEIVED:
                /* This is handled by the s_rpc_pending path above.
                 * Fall through to NORMAL if reached here without pending. */
                blink_state = ((tick_count % (LED_HEARTBEAT_INTERVAL_MS /
                                LED_TASK_TICK_MS)) <
                               (LED_HEARTBEAT_INTERVAL_MS / LED_TASK_TICK_MS / 2));
                set_leds(1, blink_state ? 1 : 0, 0);
                break;

            case ALERT_LOW_SOC_WARN:
                /* Red+Blue 2000 ms blink */
                blink_state = ((tick_count % (LED_SLOW_BLINK_MS /
                                LED_TASK_TICK_MS)) <
                               (LED_SLOW_BLINK_MS / LED_TASK_TICK_MS / 2));
                set_leds(blink_state ? 1 : 0, 0, blink_state ? 1 : 0);
                //set_leds(0, 0, 1);
                break;

            case ALERT_LOW_SOC_CRIT:
                /* Red+Blue 1000 ms blink */
                blink_state = ((tick_count % (LED_FAST_BLINK_MS /
                                LED_TASK_TICK_MS)) <
                               (LED_FAST_BLINK_MS / LED_TASK_TICK_MS / 2));
                set_leds(blink_state ? 1 : 0, 0, blink_state ? 1 : 0);
                //set_leds( 0, 1, 0);
                break;

            case ALERT_NO_BATTERIES:
                /* Red fast 1000 ms blink; others OFF */
                blink_state = ((tick_count % (LED_FAST_BLINK_MS /
                                LED_TASK_TICK_MS)) <
                               (LED_FAST_BLINK_MS / LED_TASK_TICK_MS / 2));
                set_leds(blink_state ? 1 : 0, 0, 0);
                break;

            case ALERT_VOLTAGE_ANOMALY:
                /* Red+Blue 500 ms rapid blink */
                blink_state = ((tick_count % (LED_RAPID_BLINK_MS /
                                LED_TASK_TICK_MS)) <
                               (LED_RAPID_BLINK_MS / LED_TASK_TICK_MS / 2));
                set_leds(blink_state ? 1 : 0, 0, blink_state ? 1 : 0);
                break;
                
            case ALERT_TRIKE_STOPPING:
			    /* Blue 1000 ms fast blink — visually distinct from anomaly */
			    blink_state = ((tick_count % (LED_FAST_BLINK_MS / LED_TASK_TICK_MS)) < 
			                   (LED_FAST_BLINK_MS / LED_TASK_TICK_MS / 2));
			    set_leds(0, 0, blink_state ? 1 : 0);   /* Blue fast blink */
			    break;

            default:
                /* Fallback: Red steady */
                set_leds(1, 0, 0);
                break;
        }
    }
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

esp_err_t rgb_led_init(void)
{
    /* Configure GPIO outputs */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_LED_RED_PIN)   |
                        (1ULL << BOARD_LED_GREEN_PIN)  |
                        (1ULL << BOARD_LED_BLUE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Red ON (power indicator), Green and Blue OFF */
    gpio_set_level(BOARD_LED_RED_PIN,   1);
    gpio_set_level(BOARD_LED_GREEN_PIN, 0);
    gpio_set_level(BOARD_LED_BLUE_PIN,  0);

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LED mutex");
        return ESP_FAIL;
    }

    s_alert_mode  = ALERT_BOOT;
    s_rpc_pending = false;

    BaseType_t created = xTaskCreatePinnedToCore(
        rgb_led_task,
        "rgb_led",
        2048,
        NULL,
        2,      /* Low priority — purely cosmetic */
        NULL,
        0       /* Core 0; does not share UART with MQTT task on Core 1 */
    );

    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RGB LED system initialised (R=IO%d G=IO%d B=IO%d)",
             BOARD_LED_RED_PIN, BOARD_LED_GREEN_PIN, BOARD_LED_BLUE_PIN);
    return ESP_OK;
}

bool rgb_led_set_alert(rgb_alert_mode_t mode, bool force)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    rgb_alert_mode_t current = s_alert_mode;
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    bool allowed = false;

    if (force) {
        allowed = true;

    } else {
        /* ----------------------------------------------------------------
         * Trike-state immunity rule:
         * ALERT_NO_BATTERIES must never override an active trike power-state
         * alert (TRIKE_OFF or TRIKE_STOPPING). These are intentional system
         * states; a battery read failure during trike shutdown is expected
         * and should not mask the shutdown LED indication.
         * ---------------------------------------------------------------- */
        bool current_is_trike_state = (current == ALERT_TRIKE_OFF ||
                                        current == ALERT_TRIKE_STOPPING);
        bool incoming_is_no_batt    = (mode == ALERT_NO_BATTERIES);

        if (current_is_trike_state && incoming_is_no_batt) {
            /* Block unconditionally — trike state owns the LED */
            ESP_LOGD(TAG, "ALERT_NO_BATTERIES blocked — trike state %s active",
                     rgb_led_alert_to_str(current));
            xSemaphoreGive(s_mutex);
            return false;
        }

        if (mode >= current) {
            allowed = true;
        } else {
            uint32_t held_ms = now_ms - s_alert_set_time_ms;
            if (held_ms >= LED_PRIORITY_HOLDOFF_MS) {
                allowed = true;
            } else {
                ESP_LOGD(TAG,
                         "Alert %s blocked by %s (held %" PRIu32 "ms)",
                         rgb_led_alert_to_str(mode),
                         rgb_led_alert_to_str(current),
                         held_ms);
            }
        }
    }

    if (allowed) {
        s_alert_mode        = mode;
        s_alert_set_time_ms = now_ms;
    }

    xSemaphoreGive(s_mutex);
    return allowed;
}

/*bool rgb_led_set_alert(rgb_alert_mode_t mode, bool force)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    rgb_alert_mode_t current = s_alert_mode;
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    bool allowed = false;

    if (force) {
         Explicit system transitions always win 
        allowed = true;
    } else if (mode >= current) {
         Equal or higher priority always wins 
        allowed = true;
    } else {
         Lower priority: only allowed if holdoff has expired 
        uint32_t held_ms = now_ms - s_alert_set_time_ms;
        if (held_ms >= LED_PRIORITY_HOLDOFF_MS) {
            allowed = true;
        } else {
            ESP_LOGD(TAG,
                     "Alert %s blocked by higher-priority %s (held %" PRIu32 "ms)",
                     rgb_led_alert_to_str(mode),
                     rgb_led_alert_to_str(current),
                     held_ms);
        }
    }

    if (allowed) {
        s_alert_mode      = mode;
        s_alert_set_time_ms = now_ms;
    }

    xSemaphoreGive(s_mutex);
    return allowed;
}*/

/*bool rgb_led_set_alert(rgb_alert_mode_t mode)
{
	 if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
		 s_alert_mode = mode;
		 xSemaphoreGive(s_mutex);
		 return true;
	 }
    return false;
}*/

void rgb_led_notify_rpc(void)
{
    s_rpc_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    s_rpc_pending  = true;
}

void rgb_led_set_raw(bool red, bool green, bool blue)
{
    gpio_set_level(BOARD_LED_RED_PIN,   red   ? 1 : 0);
    gpio_set_level(BOARD_LED_GREEN_PIN, green ? 1 : 0);
    gpio_set_level(BOARD_LED_BLUE_PIN,  blue  ? 1 : 0);
}

rgb_alert_mode_t rgb_led_get_alert(void)
{
    return s_alert_mode;
}

// New string converter:
const char *rgb_led_alert_to_str(rgb_alert_mode_t alert)
{
    switch (alert) {
        case ALERT_NORMAL:          return "NORMAL";
        case ALERT_LOW_SOC_WARN:    return "LOW_SOC_WARN";
        case ALERT_LOW_SOC_CRIT:    return "LOW_SOC_CRIT";
        case ALERT_NO_BATTERIES:    return "NO_BATTERIES";
        case ALERT_TRIKE_OFF:       return "TRIKE_OFF";
        case ALERT_VOLTAGE_ANOMALY: return "VOLT_ANOMALY";
        case ALERT_TRIKE_STOPPING: return "TRIKE_STOPPING";
        default:                    return "UNKNOWN";
    }
}
