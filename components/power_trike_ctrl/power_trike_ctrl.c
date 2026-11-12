/**
 * @file power_trike_ctrl.c
 * @brief Power Management and Trike Power Control — Combined Implementation
 *
 * @author  Collins N. Osebe
 * @date    2026-11-09
 */

#include "power_trike_ctrl.h"
#include "board_config.h"
#include "rgb_led.h"
#include "Quectel_mqtt.h"
#include "bms_monitor_task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

#define TAG "PWR_TRIKE"

/* =========================================================================
 * NVS KEYS
 * ========================================================================= */

#define TRIKE_NVS_NAMESPACE     "trike_ctrl"
#define TRIKE_NVS_KEY_PCONF     "pwr_conf"
#define TRIKE_NVS_KEY_PPEND     "pwr_pend"

/* =========================================================================
 * WAIT-FOR-STOP TASK TIMING
 * ========================================================================= */

#define TRIKE_STOP_POLL_MS      5000   /**< Motion poll interval             */
#define TRIKE_LED_INTERVAL_MS   60000  /**< LED alert interval while waiting */

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static struct {
    QueueHandle_t    event_queue;
    SemaphoreHandle_t state_mutex;
    TaskHandle_t     monitor_task;

    bool             initialized;
    bool             is_power_on_reset;
} power_state = {
    .initialized       = false,
    .is_power_on_reset = false,
};

static volatile bool   s_power_confirmation   = false;
static volatile bool   s_resp_cmd_pending     = false;
static volatile bool   s_resp_cmd_value       = false;
static volatile bool   s_wait_task_running    = false;
static volatile uint32_t s_wait_task_gen      = 0;
static TaskHandle_t    s_wait_task_handle     = NULL;

/* =========================================================================
 * FORWARD DECLARATIONS
 * ========================================================================= */
static void power_monitor_task(void *pvParameters);
static void execute_trike_poweroff(uint32_t expected_gen);
static void trike_wait_for_stop_task(void *pvParameters);
static void start_wait_task(bool boot_recovery);
static bool load_nvs_power_confirmation(bool *out_value);
static bool load_nvs_power_pending(bool *out_pending);
static void trike_ctrl_save_nvs_pending(bool pending);

/* =========================================================================
 * INTERNAL: INTERNAL AT COMMAND (caller holds mutex)
 * ========================================================================= */

static bool send_at_internal(const char *cmd, const char *expected,
                              int retries, uint32_t timeout_ms)
{
    char response[BOARD_MODEM_BUF_SIZE];
    memset(response, 0, sizeof(response));

    for (int attempt = 0; attempt < retries; attempt++) {
        uart_write_bytes(BOARD_MODEM_UART_NUM, cmd, strlen(cmd));
        uart_write_bytes(BOARD_MODEM_UART_NUM, "\r\n", 2);

        uint64_t t0 = esp_timer_get_time();
        int idx = 0;
        while ((esp_timer_get_time() - t0) < ((uint64_t)timeout_ms * 1000ULL)) {
            uint8_t b;
            if (uart_read_bytes(BOARD_MODEM_UART_NUM, &b, 1,
                                20 / portTICK_PERIOD_MS) > 0) {
                if (idx < (int)sizeof(response) - 1) {
                    response[idx++] = (char)b;
                }
            }
        }
        response[idx] = '\0';
        if (strstr(response, expected)) {
            return true;
        }
    }
    return false;
}

/* =========================================================================
 * TRIKE POWEROFF / POWERON SEQUENCE
 * ========================================================================= */

static uint32_t next_wait_gen(void)
{
    s_wait_task_gen = (s_wait_task_gen + 1U) & 0x7FFFFFFFU;
    if (s_wait_task_gen == 0U) {
        s_wait_task_gen = 1U;
    }
    return s_wait_task_gen;
}

/* Caller holds state_mutex */
static void cancel_wait_task_locked(void)
{
    next_wait_gen();
    s_wait_task_running = false;
    TaskHandle_t h = s_wait_task_handle;
    s_wait_task_handle = NULL;
    if (h) {
        xTaskNotifyGive(h);
    }
}

static void start_wait_task(bool boot_recovery)
{
    uint32_t gen = 0;
    bool should_create = false;

    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }
    if (!s_wait_task_running) {
        s_wait_task_running = true;
        gen                 = next_wait_gen();
        should_create       = true;
    }
    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }

    if (!should_create) {
        return;
    }

    uintptr_t arg = (boot_recovery ? 0x80000000UL : 0UL) | (uintptr_t)gen;
    TaskHandle_t handle = NULL;
    BaseType_t res = xTaskCreatePinnedToCore(
        trike_wait_for_stop_task,
        "trike_wait",
        4096, (void *)arg, 4, &handle, 1
    );

    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }
    if (res != pdPASS) {
        if (s_wait_task_gen == gen) {
            s_wait_task_running = false;
            s_wait_task_handle  = NULL;
        }
        ESP_LOGE(TAG, "Failed to create trike_wait task");
    } else if (s_wait_task_gen == gen) {
        s_wait_task_handle = handle;
    } else {
        /* Cancelled before task creation returned — notify immediately to exit */
        xTaskNotifyGive(handle);
    }
    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }
}

static void execute_trike_poweroff(uint32_t expected_gen)
{
    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }

    if (expected_gen != 0U && expected_gen != s_wait_task_gen) {
        if (power_state.state_mutex) {
            xSemaphoreGive(power_state.state_mutex);
        }
        ESP_LOGI(TAG, "Power-off execution aborted due to cancellation");
        return;
    }

    /* Direct power-off invalidates any running wait-for-stop task */
    if (expected_gen == 0U && s_wait_task_running) {
        cancel_wait_task_locked();
    }

    /* 1. Actuate hardware IMMEDIATELY and atomically with state update */
    gpio_set_level(BOARD_TRIKE_PWR_PIN, 0);
    s_power_confirmation = false;
    trike_ctrl_save_nvs(false);
    trike_ctrl_save_nvs_pending(false);
    s_resp_cmd_pending = true;
    s_resp_cmd_value   = false;

    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }

    ESP_LOGW(TAG, "Executing trike power-off sequence: TRIKE_PWR_CTRL → LOW (trike OFF)");
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2. Visual status indication */
    if (rgb_led_set_alert(ALERT_TRIKE_OFF, true)) {
        ESP_LOGI(TAG, "Switched RGB LED status");
    } else {
        ESP_LOGE(TAG, "Failed to switch RGB LED status!");
    }

    /* 3. Publish telemetry confirmation (hardware is already safe and de-asserted) */
    if (!mqtt_pubclient_status()) {
        ESP_LOGW(TAG, "Failed to publish trike power-off state to telemetry");
    }

    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }
    s_resp_cmd_pending = false;
    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }
}

static void trike_wait_for_stop_task(void *pvParameters)
{
    uintptr_t arg      = (uintptr_t)pvParameters;
    bool boot_recovery = (arg & 0x80000000UL) != 0;
    uint32_t my_gen    = (uint32_t)(arg & 0x7FFFFFFFUL);

    ESP_LOGI(TAG, "Wait-for-stop task started%s",
             boot_recovery ? " (boot recovery)" : "");

    if (boot_recovery) {
        /* Allow bms_monitor_task to complete fresh sensor cycles before checking motion */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TRIKE_STOP_POLL_MS));
    }

    uint32_t start_ms     = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t last_led_ms  = start_ms;
    uint32_t last_warn_ms = start_ms;

    while (1) {
        bool cancelled = false;
        if (power_state.state_mutex) {
            xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
            cancelled = (my_gen != s_wait_task_gen);
            xSemaphoreGive(power_state.state_mutex);
        } else {
            cancelled = (my_gen != s_wait_task_gen);
        }

        if (cancelled) {
            ESP_LOGI(TAG, "Wait-for-stop task cancelled — shutdown aborted");
            break;
        }

        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* LED alert every minute while waiting */
        if ((now_ms - last_led_ms) >= TRIKE_LED_INTERVAL_MS) {
            //if( rgb_led_set_alert(ALERT_VOLTAGE_ANOMALY)) ESP_LOGI(TAG, "Switched RGB LED status");
            if( rgb_led_set_alert(ALERT_TRIKE_STOPPING, true)) ESP_LOGI(TAG, "Switched RGB LED status");
    		else ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
            last_led_ms = now_ms;
            ESP_LOGI(TAG, "Waiting for trike to stop...");
        }

        /* Check motion — try accelerometer first, fall back to current */
        bool still_moving = check_trike_motion();

        if (!still_moving) {
            ESP_LOGI(TAG, "Trike stopped — executing poweroff");
            execute_trike_poweroff(my_gen);
            break;
        }

        /*
         * Safety interlock: NEVER force poweroff while vehicle is in motion.
         * If wait exceeds TRIKE_MAX_WAIT_FOR_STOP_MS, emit a periodic warning
         * but keep motive power active to protect the operator and vehicle.
         */
        if ((now_ms - start_ms) >= TRIKE_MAX_WAIT_FOR_STOP_MS) {
            if ((now_ms - last_warn_ms) >= TRIKE_LED_INTERVAL_MS) {
                ESP_LOGW(TAG, "Wait-for-stop elapsed > %lu min — motion still detected, holding power ON for safety",
                         (unsigned long)(TRIKE_MAX_WAIT_FOR_STOP_MS / 60000UL));
                last_warn_ms = now_ms;
            }
        }

        /* Sleep until next poll or until notified of cancellation */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TRIKE_STOP_POLL_MS));
    }

    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }
    if (my_gen == s_wait_task_gen) {
        s_wait_task_running = false;
        s_wait_task_handle  = NULL;
    }
    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }

    vTaskDelete(NULL);
}

/* =========================================================================
 * NVS PERSISTENCE
 * ========================================================================= */

void trike_ctrl_save_nvs(bool value)
{
    static bool last_saved = false;
    static bool first_save = true;

    if (!first_save && last_saved == value) {
        return;
    }

    nvs_handle_t h;
    esp_err_t ret = nvs_open(TRIKE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return;
    }

    nvs_set_u8(h, TRIKE_NVS_KEY_PCONF, value ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);

    last_saved = value;
    first_save = false;
    ESP_LOGI(TAG, "power_confirmation saved to NVS: %s",
             value ? "ON" : "OFF");
}

static bool load_nvs_power_confirmation(bool *out_value)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(TRIKE_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) {
        *out_value = true;
        return false;
    }

    uint8_t v = 1;
    ret = nvs_get_u8(h, TRIKE_NVS_KEY_PCONF, &v);
    nvs_close(h);

    if (ret == ESP_OK) {
        *out_value = (v != 0);
        ESP_LOGI(TAG, "power_confirmation loaded from NVS: %s",
                 *out_value ? "ON" : "OFF");
        return true;
    }

    *out_value = true;
    return false;
}

static void trike_ctrl_save_nvs_pending(bool pending)
{
    static bool last_saved_pending = false;
    static bool first_save_pending = true;

    if (!first_save_pending && last_saved_pending == pending) {
        return;
    }

    nvs_handle_t h;
    esp_err_t ret = nvs_open(TRIKE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for pending state: %s", esp_err_to_name(ret));
        return;
    }

    nvs_set_u8(h, TRIKE_NVS_KEY_PPEND, pending ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);

    last_saved_pending = pending;
    first_save_pending = false;
    ESP_LOGI(TAG, "power_pending saved to NVS: %s", pending ? "YES" : "NO");
}

static bool load_nvs_power_pending(bool *out_pending)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(TRIKE_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) {
        *out_pending = false;
        return false;
    }

    uint8_t v = 0;
    ret = nvs_get_u8(h, TRIKE_NVS_KEY_PPEND, &v);
    nvs_close(h);

    if (ret == ESP_OK) {
        *out_pending = (v != 0);
        ESP_LOGI(TAG, "power_pending loaded from NVS: %s",
                 *out_pending ? "YES" : "NO");
        return true;
    }

    *out_pending = false;
    return false;
}

/* =========================================================================
 * POWER MONITOR TASK
 * ========================================================================= */

static void power_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Power monitor task started on Core %d", xPortGetCoreID());

    uint32_t last_modem_check = 0;
    const uint32_t MODEM_CHECK_INTERVAL_MS = 60000;

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        /*
         * Periodic modem health check.
         * On TCU V2.1 the mains detection pin has been removed; modem
         * health is checked regardless of power source.
         */
        if ((now - last_modem_check) >= MODEM_CHECK_INTERVAL_MS) {
            if (!power_mgmt_check_modem_alive()) {
                ESP_LOGW(TAG, "Modem brownout detected — queueing recovery");
                power_event_t ev = {
                    .type         = POWER_EVENT_MODEM_BROWNOUT,
                    .timestamp_ms = now
                };
                xQueueSend(power_state.event_queue, &ev, 0);
            }
            last_modem_check = now;
        }
    }
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

esp_err_t power_trike_init(void)
{
    if (power_state.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initialising power and trike control...");

    esp_reset_reason_t rr = esp_reset_reason();
    power_state.is_power_on_reset = (rr == ESP_RST_POWERON);

    power_state.state_mutex = xSemaphoreCreateMutex();
    if (!power_state.state_mutex) {
        return ESP_FAIL;
    }

    power_state.event_queue = xQueueCreate(POWER_EVENT_QUEUE_SIZE,
                                            sizeof(power_event_t));
    if (!power_state.event_queue) {
        return ESP_FAIL;
    }
    
    /* Configure BOARD_TRIKE_PWR_PIN (GPIO21) as output */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_TRIKE_PWR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Load trike state and pending shutdown state from NVS */
    bool stored = false;
    load_nvs_power_confirmation(&stored);

    bool pending_poweroff = false;
    load_nvs_power_pending(&pending_poweroff);

    s_resp_cmd_pending  = false;
    s_resp_cmd_value    = false;
    s_wait_task_running = false;
    s_wait_task_gen     = 0;
    s_wait_task_handle  = NULL;

    if (stored && pending_poweroff) {
        /*
         * Critical recovery: MCU rebooted while a remote power-off command was pending!
         * bms_monitor_task has not yet completed fresh sensor cycles at init time,
         * so keep motive power ON, set ALERT_TRIKE_STOPPING, and launch
         * trike_wait_for_stop_task in boot-recovery mode to evaluate real sensor
         * state before cutting power.
         */
        ESP_LOGW(TAG, "Boot recovery: Found pending poweroff — resuming wait-for-stop task");
        s_power_confirmation = true;
        power_mgmt_trike_set_gpio(true);
        rgb_led_set_alert(ALERT_TRIKE_STOPPING, true);
        start_wait_task(true);
    } else {
        if (pending_poweroff) {
            trike_ctrl_save_nvs_pending(false);
        }
        s_power_confirmation = stored;
        power_mgmt_trike_set_gpio(stored);

        /* Set LED status to trike off if gpio state is 0 */
        if (!stored) {
            rgb_led_set_alert(ALERT_TRIKE_OFF, false);
        }
    }

    /* Start power monitor task */
    BaseType_t created = xTaskCreatePinnedToCore(
        power_monitor_task,
        "power_monitor",
        4096,
        NULL,
        6,
        &power_state.monitor_task,
        1
    );

    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create power monitor task");
        return ESP_FAIL;
    }

    power_state.initialized = true;
    ESP_LOGI(TAG, "Power/trike control initialised — trike=%s",
             s_power_confirmation ? "ON" : "OFF");
    return ESP_OK;
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

QueueHandle_t power_mgmt_get_event_queue(void)
{
    return power_state.event_queue;
}

bool power_mgmt_is_power_on_reset(void)
{
    return power_state.is_power_on_reset;
}

int power_mgmt_get_reset_reason(char *buffer, size_t size)
{
    if (!buffer || size == 0) return 0;
    const char *s = "UNKNOWN";
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   s = "POWER_ON";   break;
        case ESP_RST_SW:        s = "SOFTWARE";   break;
        case ESP_RST_PANIC:     s = "PANIC";       break;
        case ESP_RST_INT_WDT:   s = "INT_WDT";    break;
        case ESP_RST_TASK_WDT:  s = "TASK_WDT";   break;
        case ESP_RST_WDT:       s = "WDT";         break;
        case ESP_RST_DEEPSLEEP: s = "DEEP_SLEEP";  break;
        case ESP_RST_BROWNOUT:  s = "BROWNOUT";    break;
        default: break;
    }
    return snprintf(buffer, size, "%s", s);
}

bool power_mgmt_check_modem_alive(void)
{
    extern SemaphoreHandle_t uart_modem_mutex;
    if (!uart_modem_mutex) return false;
    if (xSemaphoreTake(uart_modem_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return false;
    }
    bool ok = send_at_internal("AT", "OK", 2, 300);
    xSemaphoreGive(uart_modem_mutex);
    return ok;
}

esp_err_t power_mgmt_recover_modem(void)
{
    ESP_LOGW(TAG, "Attempting modem recovery...");
    vTaskDelay(pdMS_TO_TICKS(500));
    gsm_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (power_mgmt_check_modem_alive()) {
        if (activate_pdp() && open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password) && mqtt_resubscribe()) {
            ESP_LOGI(TAG, "Modem recovered via reset");
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Reset failed — attempting full power cycle");
    powerdown_modem();
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (poweron_modem() && activate_pdp() && open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password) && mqtt_resubscribe()) {
        ESP_LOGI(TAG, "Modem recovered via power cycle");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Modem recovery failed");
    return ESP_FAIL;
}

void power_mgmt_trike_poweroff(void)
{
    gpio_set_level(BOARD_TRIKE_PWR_PIN, 0);
    ESP_LOGW(TAG, "TRIKE_PWR_CTRL → LOW (trike OFF)");
    vTaskDelay(pdMS_TO_TICKS(50));
}

void power_mgmt_trike_poweron(void)
{
    gpio_set_level(BOARD_TRIKE_PWR_PIN, 1);
    ESP_LOGI(TAG, "TRIKE_PWR_CTRL → HIGH (trike ON)");
    vTaskDelay(pdMS_TO_TICKS(50));
}

void power_mgmt_trike_set_gpio(bool trike_on)
{
    int level = trike_on ? 1 : 0;
    gpio_set_level(BOARD_TRIKE_PWR_PIN, level);
    ESP_LOGI(TAG, "Boot GPIO restore: TRIKE_PWR → %d (%s)",
             level, trike_on ? "ON" : "OFF");
}

/* Caller holds state_mutex */
static void set_power_direct_locked(bool trike_on)
{
    if (s_wait_task_running) {
        ESP_LOGI(TAG, "AP Kill Switch: cancelling pending wait-for-stop task");
        cancel_wait_task_locked();
    }
    gpio_set_level(BOARD_TRIKE_PWR_PIN, trike_on ? 1 : 0);
    s_power_confirmation = trike_on;
    trike_ctrl_save_nvs(trike_on);
    trike_ctrl_save_nvs_pending(false);
}

static void notify_direct_power_state(bool trike_on)
{
    if (trike_on) {
        rgb_led_set_alert(ALERT_NORMAL, true);
        ESP_LOGI(TAG, "AP Kill Switch Toggle: TRIKE_PWR_CTRL → HIGH (System ON)");
    } else {
        rgb_led_set_alert(ALERT_TRIKE_OFF, true);
        ESP_LOGW(TAG, "AP Kill Switch Toggle: TRIKE_PWR_CTRL → LOW (System OFF)");
    }
}

void trike_ctrl_set_power_direct(bool trike_on)
{
    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }
    set_power_direct_locked(trike_on);
    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }
    notify_direct_power_state(trike_on);
}

bool trike_ctrl_toggle_power(void)
{
    if (power_state.state_mutex) {
        xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
    }
    bool next_state = !s_power_confirmation;
    set_power_direct_locked(next_state);
    if (power_state.state_mutex) {
        xSemaphoreGive(power_state.state_mutex);
    }
    notify_direct_power_state(next_state);
    return next_state;
}

bool trike_ctrl_get_power_confirmation(void)
{
    bool val;
    if (power_state.state_mutex &&
        xSemaphoreTake(power_state.state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        val = s_power_confirmation;
        xSemaphoreGive(power_state.state_mutex);
    } else {
        val = s_power_confirmation;
    }
    return val;
}

void trike_ctrl_handle_command(uint8_t trike_ctrl_value)
{
    ESP_LOGI(TAG, "Trike command: %d", trike_ctrl_value);

    if (trike_ctrl_value == 2) {
        /* Power ON */
        if (power_state.state_mutex) {
            xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
        }
        if (s_wait_task_running) {
            ESP_LOGI(TAG, "Cancelling pending wait-for-stop poweroff task due to Power ON command");
            cancel_wait_task_locked();
        }
        gpio_set_level(BOARD_TRIKE_PWR_PIN, 1);
        s_power_confirmation = true;
        trike_ctrl_save_nvs(true);
        trike_ctrl_save_nvs_pending(false);
        if (power_state.state_mutex) {
            xSemaphoreGive(power_state.state_mutex);
        }

        ESP_LOGI(TAG, "TRIKE_PWR_CTRL → HIGH (trike ON)");
        vTaskDelay(pdMS_TO_TICKS(50));

        if (rgb_led_set_alert(ALERT_NORMAL, true)) {
            ESP_LOGI(TAG, "Switched RGB LED status");
        } else {
            ESP_LOGE(TAG, "Failed to switch RGB LED status!");
        }
        ESP_LOGI(TAG, "Trike power ON");

        if (!mqtt_pubclient_status()) {
            ESP_LOGW(TAG, "Trike ON ACK publish failed; telemetry will update on next cycle");
        }
        return;
    }

    if (trike_ctrl_value == 1) {
        /* Power OFF */
        if (!check_trike_motion()) {
            /* Stationary — immediate poweroff */
            ESP_LOGI(TAG, "Trike stationary — immediate poweroff");
            execute_trike_poweroff(0U);
        } else {
            /* Moving — queue poweroff */
            ESP_LOGW(TAG, "Trike in motion — queuing poweroff");

            if (power_state.state_mutex) {
                xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
            }
            s_power_confirmation = true;
            trike_ctrl_save_nvs(true);
            trike_ctrl_save_nvs_pending(true);
            s_resp_cmd_pending = true;
            s_resp_cmd_value   = true;
            if (power_state.state_mutex) {
                xSemaphoreGive(power_state.state_mutex);
            }

            if (rgb_led_set_alert(ALERT_TRIKE_STOPPING, false)) {
                ESP_LOGI(TAG, "Switched RGB LED status");
            } else {
                ESP_LOGE(TAG, "Failed to switch RGB LED status!");
            }

            start_wait_task(false);

            /* Publish queued poweroff alert to telemetry */
            if (!mqtt_pubclient_status()) {
                ESP_LOGW(TAG, "Failed to publish queued poweroff alert; telemetry will update on next cycle");
            }
            if (power_state.state_mutex) {
                xSemaphoreTake(power_state.state_mutex, portMAX_DELAY);
            }
            s_resp_cmd_pending = false;
            if (power_state.state_mutex) {
                xSemaphoreGive(power_state.state_mutex);
            }
        }
        return;
    }

    ESP_LOGW(TAG, "Unknown trike_ctrl value: %d", trike_ctrl_value);
}

bool trike_ctrl_consume_resp_cmd_flag(bool *resp_cmd_value)
{
    if (!resp_cmd_value) return false;

    bool result = false;
    if (power_state.state_mutex &&
        xSemaphoreTake(power_state.state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_resp_cmd_pending) {
            *resp_cmd_value    = s_resp_cmd_value;
            s_resp_cmd_pending = false;  /* Atomically consume and clear */
            result             = true;
        }
        xSemaphoreGive(power_state.state_mutex);
    } else {
        if (s_resp_cmd_pending) {
            *resp_cmd_value    = s_resp_cmd_value;
            s_resp_cmd_pending = false;
            result             = true;
        }
    }
    return result;
}

void trike_ctrl_publish_state(void)
{
    if (!mqtt_pubclient_status()) {
        ESP_LOGW(TAG, "Failed to publish restored trike state to telemetry");
    } else {
        ESP_LOGI(TAG, "Trike state restored on ThingsBoard: %s",
                 trike_ctrl_get_power_confirmation() ? "ON" : "OFF");
    }
}