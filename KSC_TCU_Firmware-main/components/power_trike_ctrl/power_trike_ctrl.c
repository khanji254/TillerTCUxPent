/**
 * @file power_trike_ctrl.c
 * @brief Power Management and Trike Power Control — Combined Implementation
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
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

static volatile bool   s_power_confirmation = false;
static volatile bool   s_resp_cmd_pending   = false;
static volatile bool   s_resp_cmd_value     = false;
static volatile bool   s_wait_task_running  = false;

/* =========================================================================
 * FORWARD DECLARATIONS
 * ========================================================================= */
static void power_monitor_task(void *pvParameters);
static void execute_trike_poweroff(void);
static void trike_wait_for_stop_task(void *pvParameters);
static bool load_nvs_power_confirmation(bool *out_value);

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

static void execute_trike_poweroff(void)
{
    ESP_LOGW(TAG, "Executing trike power-off sequence");

    s_power_confirmation = false;
    trike_ctrl_save_nvs(false);

    s_resp_cmd_pending = true;
    s_resp_cmd_value   = false;

    if (!mqtt_pubclient_status(false)) {
        ESP_LOGE(TAG, "Failed to publish trike power-off state..retrying");
        mqtt_pubclient_status(false);
    }

    s_resp_cmd_pending = false;

    if(rgb_led_set_alert(ALERT_TRIKE_OFF, true)) ESP_LOGI(TAG, "Switched RGB LED status");
    else ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
    
    power_mgmt_trike_poweroff();
}

static void trike_wait_for_stop_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Wait-for-stop task started");
    s_wait_task_running = true;

    uint32_t start_ms    = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t last_led_ms = start_ms;

    while (1) {
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

/*         Safety timeout — force poweroff after TRIKE_MAX_WAIT_FOR_STOP_MS 
        if ((now_ms - start_ms) >= TRIKE_MAX_WAIT_FOR_STOP_MS) {
            ESP_LOGE(TAG, "Max wait exceeded — forcing trike poweroff");
            execute_trike_poweroff();
            break;
        }*/

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
            execute_trike_poweroff();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(TRIKE_STOP_POLL_MS));
    }

    s_wait_task_running = false;
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
        *out_value = false;
        return false;
    }

    uint8_t v = 0;
    ret = nvs_get_u8(h, TRIKE_NVS_KEY_PCONF, &v);
    nvs_close(h);

    if (ret == ESP_OK) {
        *out_value = (v != 0);
        ESP_LOGI(TAG, "power_confirmation loaded from NVS: %s",
                 *out_value ? "ON" : "OFF");
        return true;
    }

    *out_value = false;
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
         * On TCU V1.1 the mains detection pin has been removed; modem
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
    
    /* Configure GPIO40 as output */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_TRIKE_PWR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Load trike state from NVS and restore GPIO level */
    bool stored = false;
    load_nvs_power_confirmation(&stored);
    s_power_confirmation = stored;
    power_mgmt_trike_set_gpio(stored);
    
    /*Set LED status to trike off if gpio state is 0*/
    if(!stored) rgb_led_set_alert(ALERT_TRIKE_OFF,false);

    s_resp_cmd_pending  = false;
    s_resp_cmd_value    = false;
    s_wait_task_running = false;

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
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGW(TAG, "TRIKE_PWR_CTRL → LOW (trike OFF)");
    gpio_set_level(BOARD_TRIKE_PWR_PIN, 0);
}

void power_mgmt_trike_poweron(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "TRIKE_PWR_CTRL → HIGH (trike ON)");
    gpio_set_level(BOARD_TRIKE_PWR_PIN, 1);
}

void power_mgmt_trike_set_gpio(bool trike_on)
{
    int level = trike_on ? 1 : 0;
    gpio_set_level(BOARD_TRIKE_PWR_PIN, level);
    ESP_LOGI(TAG, "Boot GPIO restore: TRIKE_PWR → %d (%s)",
             level, trike_on ? "ON" : "OFF");
}

bool trike_ctrl_get_power_confirmation(void)
{
    return s_power_confirmation;
}

void trike_ctrl_handle_command(uint8_t trike_ctrl_value)
{
    ESP_LOGI(TAG, "Trike command: %d", trike_ctrl_value);

    if (trike_ctrl_value == 2) {
        /* Power ON */
        s_power_confirmation = true;
        trike_ctrl_save_nvs(true);
        power_mgmt_trike_poweron();
        //if(rgb_led_set_alert(ALERT_NORMAL)) ESP_LOGI(TAG, "Switched RGB LED status");
        if(rgb_led_set_alert(ALERT_NORMAL, true)) ESP_LOGI(TAG, "Switched RGB LED status");
    	else ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
        ESP_LOGI(TAG, "Trike power ON");

        if (!mqtt_pubclient_status(false)) {
            ESP_LOGE(TAG, "Trike ON ACK publish failed, retrying ...");
            mqtt_pubclient_status(false);
        }
        return;
    }

    if (trike_ctrl_value == 1) {
        /* Power OFF */
        if (!check_trike_motion()) {
            /* Stationary — immediate poweroff */
            ESP_LOGI(TAG, "Trike stationary — immediate poweroff");
            execute_trike_poweroff();
        } else {
            /* Moving — queue poweroff */
            ESP_LOGW(TAG, "Trike in motion — queuing poweroff");
            s_power_confirmation = true;
            trike_ctrl_save_nvs(true);
            s_resp_cmd_pending = true;
            s_resp_cmd_value   = true;

            if (!mqtt_pubclient_status(false)) {
                ESP_LOGE(TAG, "Failed to publish queued poweroff alert; retrying..");
                mqtt_pubclient_status(false);
            }
            s_resp_cmd_pending = false;

            if(rgb_led_set_alert(ALERT_TRIKE_STOPPING, false)) ESP_LOGI(TAG, "Switched RGB LED status");
    		else ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 

            if (!s_wait_task_running) {
                xTaskCreatePinnedToCore(
                    trike_wait_for_stop_task,
                    "trike_wait",
                    3072, NULL, 4, NULL, 1
                );
            }
        }
        return;
    }

    ESP_LOGW(TAG, "Unknown trike_ctrl value: %d", trike_ctrl_value);
}

bool trike_ctrl_consume_resp_cmd_flag(bool *resp_cmd_value)
{
    if (s_resp_cmd_pending) {
        *resp_cmd_value = s_resp_cmd_value;
        return true;
    }
    return false;
}

void trike_ctrl_publish_state(void)
{
    if (!mqtt_pubclient_status()) {
        ESP_LOGE(TAG, "Failed to publish restored trike state; retying..");
        mqtt_pubclient_status();
    } else {
        ESP_LOGI(TAG, "Trike state restored on ThingsBoard: %s",
                 s_power_confirmation ? "ON" : "OFF");
    }
}