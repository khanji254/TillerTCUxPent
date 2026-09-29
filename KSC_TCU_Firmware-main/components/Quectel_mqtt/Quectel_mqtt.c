/**
 * @file Quectel_mqtt.c
 * @brief Quectel EG915N Modem MQTT Transport Layer Implementation
 *
 * Full provisioning workflow for KSC TCU V1.1.  Key changes from original:
 *
 *  1. Runtime credential buffers (mqtt_username / mqtt_password /
 *     mqtt_client_id) replace the compile-time MQTT_USERNAME macros.
 *     Credentials are loaded from NVS or obtained via TB provisioning.
 *
 *  2. tcu_load_or_provision_creds() is called from mqtt_subclient()
 *     before any normal MQTT connection attempt.
 *
 *  3. open_mqtts_with_creds() replaces open_mqtts() to accept runtime
 *     credential parameters.
 *
 *  4. read_buffered_messages() now properly uses publish_rpc_responses()
 *     to acknowledge RPC commands to ThingsBoard.  See the "RPC
 *     ACKNOWLEDGEMENT" comment block for the design explanation.
 *
 *  5. NVS corruption detection:
 *     - If tcu_nvs_load_creds() fails after tcu_nvs_is_provisioned()
 *       returned true, tcu_nvs_repair() is called and provisioning
 *       is re-run.
 *
 *  6. Device deleted on TB:
 *     - If open_mqtts_with_creds() returns +QMTCONN: 0,5 (auth fail)
 *       with stored credentials, tcu_force_reprovision() is called
 *       and the device restarts to re-provision.
 *
 * @author  Mary Mbugua
 * @date    2026-04-10
 */
 
#include "Quectel_mqtt.h"
#include "power_trike_ctrl.h"
#include "rgb_led.h"
#include "board_config.h"
#include "tcu_nvs_creds.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
 
/* =========================================================================
 * MODULE STATE
 * ========================================================================= */
 
 volatile bool uart_installed = false;
 volatile bool provisioning_in_progress = false;
 
extern char deviceSerial[];
char HUB_NAME[32]  = {0};
char APN[32]       = "safaricomiot";
 
/* Runtime MQTT credential buffers — loaded from NVS or set during provision */
char mqtt_username[TCU_CRED_MAX_LEN] = {0};
char mqtt_password[TCU_CRED_MAX_LEN] = {0};
char mqtt_client_id[TCU_CRED_MAX_LEN] = {0};
 
int32_t provision  = 1;   /* 0 = in provisioning mode, 1 = normal mode */
bool    prov_resub = false;
 
QueueHandle_t urc_notification_queue = NULL;
SemaphoreHandle_t uart_modem_mutex   = NULL;
 
static volatile uint8_t average_battery_soc = 0;
 
/* Tracks whether last connection attempt was rejected due to bad credentials */
static bool mqtt_auth_failed = false;
 
extern void check_voltage_anomaly_and_act(bms_queued_data_t *all_battery_data,
                                           int batteries_read);
 
/* =========================================================================
 * URC QUEUE
 * ========================================================================= */

void urc_queue_init(void)
{
    urc_notification_queue = xQueueCreate(10, sizeof(urc_notification_t));
    if (!urc_notification_queue) {
        ESP_LOGE(TAG_GSM, "Failed to create URC queue");
    }
}

/* =========================================================================
 * UART MODEM MUTEX
 * ========================================================================= */

void uart_modem_mutex_init(void)
{
    if (uart_modem_mutex == NULL) {
        uart_modem_mutex = xSemaphoreCreateMutex();
        if (!uart_modem_mutex) {
            ESP_LOGE(TAG_GSM, "Failed to create UART modem mutex");
        }
    }
}

/* =========================================================================
 * UART INITIALISATION
 * ========================================================================= */

void uart_init(void)
{
    if (uart_installed) {
        return;
    }

    uart_config_t cfg = {
        .baud_rate  = QUECTEL_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(MODEM_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(MODEM_UART_NUM, &cfg);
    uart_set_pin(MODEM_UART_NUM, TX_PIN, RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_installed = true;
    ESP_LOGI(TAG_GSM, "Modem UART%d installed (TX=IO%d RX=IO%d)",
             MODEM_UART_NUM, TX_PIN, RX_PIN);
}

/* =========================================================================
 * MODEM HARDWARE CONTROL
 * ========================================================================= */

void gsm_reset(void)
{
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));
}

/*void gsm_poweron(void)
{
    esp_rom_gpio_pad_select_gpio(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PWR_PIN, 0);
    
    ESP_LOGI(TAG_GSM, "Wait 12s after power on");
    vTaskDelay(pdMS_TO_TICKS(12000));
    ESP_LOGI(TAG_GSM, "Waited 12s after power on");
}*/

void gsm_poweron(void)
{
    // Spec §3.8.1: PWRKEY low for ≥ 500 ms, then release HIGH and leave it there 
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(600));   // 500 ms minimum + 100 ms margin
    gpio_set_level(PWR_PIN, 1);       // release — keep HIGH during entire boot

     //Spec Figure 13: UART active ~10 s after PWRKEY pulse 
    ESP_LOGI(TAG_GSM, "Waiting 20s for modem UART ready...");
    vTaskDelay(pdMS_TO_TICKS(20000));
    
     //Flush URCs (RDY etc.) that arrived during boot before sending AT 
    uart_flush_input(MODEM_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/*void hardware_poweroff(void)
{
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
}*/

void hardware_poweroff(void)
{
     //Spec §3.9.1: PWRKEY low for ≥ 650 ms to trigger shutdown 
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(800));   // 650 ms minimum + 150 ms margin
    gpio_set_level(PWR_PIN, 1);       // release and KEEP HIGH (spec note: 
                                       // don't drive low or module re-powers)

     //Allow power-down procedure to complete (spec: 2–14 s) 
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void at_poweroff(void)
{
    for (int i = 0; i < 3; i++) {
        if (send_at_command("AT+QPOWD=1", "OK", RETRIES,
                            CMD_DELAY_MS, NULL, 0)) {
            return;
        }
    }
    ESP_LOGE(TAG_GSM, "AT power down failed");
}

/* =========================================================================
 * AT COMMAND ENGINE
 * ========================================================================= */
 
bool send_at_command(const char *command,
                     const char *expected_response,
                     int         retries,
                     uint32_t    timeout_ms,
                     char       *response_buf,
                     size_t      response_buf_size)
{
	 /* Static buffer — safe because uart_modem_mutex serialises all callers.
     * Only one send_at_command() can execute at a time due to the mutex,
     * so there is no re-entrancy risk. Moves 832 bytes off every task stack. */
    static char local_response[BUF_SIZE];
 
    if (uart_modem_mutex == NULL) {
        ESP_LOGE(TAG_GSM, "UART mutex not initialised");
        return false;
    }
 
    if (xSemaphoreTake(uart_modem_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG_GSM, "Failed to acquire UART mutex for: %s", command);
        return false;
    }
 
    bool success = false;
 
    for (int attempt = 0; attempt < retries && !success && uart_installed; attempt++) {
    
	    //flush UART for previous responses
	    uart_flush_input(MODEM_UART_NUM);
	    vTaskDelay(pdMS_TO_TICKS(10));		
		
        uart_write_bytes(MODEM_UART_NUM, command, strlen(command));
        uart_write_bytes(MODEM_UART_NUM, "\r\n", 2);
 
        uint64_t t0  = esp_timer_get_time();
        int      idx = 0;
        bool     got_terminator = false;
        memset(local_response, 0, sizeof(local_response));
 
/*        while ((esp_timer_get_time() - t0) < ((uint64_t)timeout_ms * 1000ULL)) {
            uint8_t b;
            if (uart_read_bytes(MODEM_UART_NUM, &b, 1,
                                10 / portTICK_PERIOD_MS) > 0) {
                if (idx < BUF_SIZE - 1)
                    local_response[idx++] = (char)b;
            }
        }*/
		while ((esp_timer_get_time() - t0) < ((uint64_t)timeout_ms * 1000ULL) && !got_terminator) {
		    uint8_t b;
		    if (uart_read_bytes(MODEM_UART_NUM, &b, 1, 10 / portTICK_PERIOD_MS) > 0) {
		        if (idx < BUF_SIZE - 1)
		            local_response[idx++] = (char)b;
		        // Early exit if we have a complete response terminal
		        if (
					//strstr(local_response, "\r\nOK\r\n") ||
		            strstr(local_response, "\r\nERROR\r\n") ||
		            //strstr(local_response, "+CME ERROR") ||
		            strstr(local_response, "POWERED DOWN")) {
		            got_terminator = true;
		        }
		    }
		}        
        local_response[idx] = '\0';
        
		// Intercept any embedded URC notifications
		if (strstr(local_response, "+QMTRECV:") && urc_notification_queue) {
			urc_notification_t notif = {0};
		    char *urc_ptr = strstr(local_response, "+QMTRECV:");
		    if (sscanf(urc_ptr, "+QMTRECV: %" SCNu8 ",%" SCNu8,
		               &notif.client_idx, &notif.recv_id) == 2) {
		        notif.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
		        if (xQueueSend(urc_notification_queue, &notif, 0) != pdPASS) {
		            ESP_LOGE(TAG_GSM, "URC queue full — QMTRECV lost!");
		        } else {
		            ESP_LOGI(TAG_GSM, "QMTRECV intercepted in AT response, queued");
		        }
		    }
		}

		// Also intercept +QMTSTAT for connection drop handling
		if (strstr(local_response, "+QMTSTAT:")) {
		    int cidx, ecode;
		    char *stat_ptr = strstr(local_response, "+QMTSTAT:");
		    if (sscanf(stat_ptr, "+QMTSTAT: %d,%d", &cidx, &ecode) == 2) {
		        ESP_LOGE(TAG_GSM, "QMTSTAT in AT response: error %d", ecode);
		        // Don't call handle_mqtt_error here (we hold the mutex) — flag it
		        // handle_mqtt_error will be triggered by the failed QMTCONN? check
		    }
		}        
 
        ESP_LOGI(TAG_GSM, "CMD: %s  RSP: %s", command, local_response);
 
        /* Detect authentication failure — device deleted from TB */
        if (strstr(local_response, "+QMTCONN: 0,0,5") ||
            strstr(local_response, "+QMTCONN: 0,1,5") ||
            strstr(local_response, "+QMTCONN: 0,2,5")) {
            ESP_LOGE(TAG_GSM, "MQTT auth failed (return code 5) — credentials rejected by broker");
            mqtt_auth_failed = true;
        }
 
        if (strstr(local_response, expected_response)) {
            success = true;
        } else if (attempt < retries - 1) {
            ESP_LOGW(TAG_GSM, "Attempt %d failed, retrying: %s", attempt + 1, command);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
 
        //vTaskDelay(pdMS_TO_TICKS(timeout_ms * (attempt + 1)));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
 
    if (response_buf != NULL && response_buf_size > 0) {
        strncpy(response_buf, local_response, response_buf_size - 1);
        response_buf[response_buf_size - 1] = '\0';
    }
 
    if (!success)
        ESP_LOGE(TAG_GSM, "Command '%s' failed after %d retries", command, retries);
 
    xSemaphoreGive(uart_modem_mutex);
    return success;
}
/* =========================================================================
 * MQTT STATUS PUBLISH
 * ========================================================================= */

bool mqtt_pubclient_status()
{
    /* No battery snapshot available — use whatever shunt value the
     * BMS task last committed. Not guaranteed to be cycle-aligned
     * but this path only runs when no batteries are readable anyway. */
     
    float status_shunt_max = bms_monitor_get_shunt_max();
    char *metadata = create_metadata_payload(bms_monitor_get_shunt_snapshot(), &status_shunt_max);
    if (!metadata) {
        ESP_LOGE(TAG_GSM, "Failed to create metadata payload");
        return false;
    }

    char cmd[120];
    snprintf(cmd, sizeof(cmd),
             "AT+QMTPUBEX=0,0,0,0,\"v1/devices/trikes/telemetry\",%d",
             (int)strlen(metadata));

    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
        if (!send_at_command(cmd, ">", RETRIES, CMD_DELAY_MS, NULL, 0)) {
            continue;
        }
        uart_write_bytes(MODEM_UART_NUM, metadata, strlen(metadata));
        vTaskDelay(pdMS_TO_TICKS(12));
        uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
        vTaskDelay(pdMS_TO_TICKS(3));
        ok = true;
    }

    free(metadata);

    if (!ok) {
        ESP_LOGE(TAG_GSM, "Metadata publish failed");
    }
    return ok;
}

/* =========================================================================
 * BATTERY DATA PUBLISH
 * ========================================================================= */

bool mqtt_pubclient_battery(void)
{
    /*
     * CHANGED: queue removed. Battery data now comes from the atomic
     * cycle snapshot via bms_monitor_get_snapshot(). s_last_consumed_cycle
     * is static — persists across calls so the same cycle is never
     * published twice.
     *
     * Partial publish supported: entries with status == BMS_STATUS_NO_DATA
     * are skipped; entries with BMS_STATUS_OK are published individually.
     * This matches the behaviour of the original queue-based code.
     */
    static uint32_t s_last_consumed_cycle = 0;

    bms_queued_data_t snapshot[BMS_BATTERY_COUNT];
    
    float publish_shunt_a = 0.0f;
    float publish_shunt_max_a = 0.0f;
    
    if (!bms_monitor_get_snapshot(snapshot, &s_last_consumed_cycle,
                                  &publish_shunt_a, &publish_shunt_max_a)) {
        ESP_LOGW(TAG_GSM,
                 "mqtt_pubclient_battery: no new BMS cycle available — skipping");
        return false;
    }

    int     batteries_published = 0;
    uint32_t soc_sum            = 0;
    int      valid_soc_count    = 0;

    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {

        if (snapshot[i].status != BMS_STATUS_OK) {
            ESP_LOGW(TAG_GSM, "Skipping battery %d (status=%d)",
                     snapshot[i].battery_id + 1, snapshot[i].status);
            continue;
        }

        soc_sum += snapshot[i].soc;
        valid_soc_count++;

        char *payload = create_battery_json_payload(
            &snapshot[i], snapshot, BMS_BATTERY_COUNT);
        if (!payload) {
            ESP_LOGE(TAG_GSM, "JSON alloc failed for battery %d",
                     snapshot[i].battery_id + 1);
            continue;
        }

        char cmd[120];
        snprintf(cmd, sizeof(cmd),
                 "AT+QMTPUBEX=0,0,0,0,\"v1/devices/trikes/telemetry\",%d",
                 (int)strlen(payload));

        bool pub_ok = false;
        for (int attempt = 0; attempt < 2 && !pub_ok; attempt++) {
            if (!send_at_command(cmd, ">", RETRIES, CMD_DELAY_MS, NULL, 0)) {
                continue;
            }
            uart_write_bytes(MODEM_UART_NUM, payload, strlen(payload));
            vTaskDelay(pdMS_TO_TICKS(12));
            uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
            vTaskDelay(pdMS_TO_TICKS(3));
            pub_ok = true;
        }

        free(payload);

        if (pub_ok) {
            batteries_published++;
            ESP_LOGI(TAG_GSM, "Battery %d published (cycle %" PRIu32 ")",
                     snapshot[i].battery_id + 1, s_last_consumed_cycle);
        } else {
            ESP_LOGE(TAG_GSM, "Battery %d publish failed",
                     snapshot[i].battery_id + 1);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /*
     * Update module-level average SOC — read by get_average_battery_soc()
     * which is called from create_metadata_payload() (field "27") and
     * from main.c for the SOC-based LED alert logic.
     */
    average_battery_soc = (valid_soc_count > 0)
                          ? (uint8_t)(soc_sum / valid_soc_count)
                          : 0;

    /*
     * Cross-battery voltage anomaly check.
     * CHANGED: previously called here AND duplicated in main.c.
     * It remains here only — main.c no longer calls it separately.
     */
    check_voltage_anomaly_and_act(snapshot, BMS_BATTERY_COUNT);

    /* Publish metadata after battery payloads */
    /* publish_shunt_a is now a local frozen copy. The BMS task will
     * overwrite s_shunt_current_snapshot_a during the next cycle's
     * pass 1 (~440ms from now) but this variable is unaffected.
     * All battery payloads AND the metadata P29 will use this value. */    
    char *metadata = create_metadata_payload(publish_shunt_a, &publish_shunt_max_a);
    if (metadata) {
        char cmd[120];
        snprintf(cmd, sizeof(cmd),
                 "AT+QMTPUBEX=0,0,0,0,\"v1/devices/trikes/telemetry\",%d",
                 (int)strlen(metadata));

        bool meta_ok = false;
        for (int attempt = 0; attempt < 2 && !meta_ok; attempt++) {
            if (!send_at_command(cmd, ">", RETRIES, CMD_DELAY_MS,
                                 NULL, 0)) {
                continue;
            }
            uart_write_bytes(MODEM_UART_NUM, metadata, strlen(metadata));
            vTaskDelay(pdMS_TO_TICKS(12));
            uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
            vTaskDelay(pdMS_TO_TICKS(3));
            meta_ok = true;
        }
        free(metadata);
    }

    return (batteries_published > 0);
}

/* =========================================================================
 * READ BUFFERED MESSAGES (RPC commands from ThingsBoard)
 * =========================================================================
 * This function now:
 *  1. Parses the TRPWR command and executes it.
 *  2. Builds an RPC response acknowledging the command to ThingsBoard.
 *  3. Calls publish_rpc_responses() to send the ACK.
 *
 * This closes the RPC loop — TB dashboard will show the command as
 * successfully processed, not as "timed out".
 * ========================================================================= */
 
bool read_buffered_messages(int recv_id)
{
    /* Only check MQTT connection state in normal (non-provisioning) mode.
     * In provisioning mode (provision == 0) the connection is already
     * established by the caller and will have a different client identity. */
    if (provision != 0) {
        char conn_buf[64] = {0};
        if (!send_at_command("AT+QMTCONN?", "+QMTCONN: 0,3",
                             1, 1000, conn_buf, sizeof(conn_buf))) {
            ESP_LOGW(TAG_GSM,
                     "read_buffered_messages: MQTT not connected (%.40s), deferring",
                     conn_buf);
            return false;
        }
    }	
	
    RpcResponse responses[7];
    int response_count = 0;
    memset(responses, 0, sizeof(responses));
 
    for (int i = 0; i <= recv_id; i++) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+QMTRECV=0,%d", i);
        uart_flush_input(MODEM_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));
 
        char recv_buf[BUF_SIZE];
        if (!send_at_command(cmd, "+QMTRECV:", 1, 3000,
                             recv_buf, sizeof(recv_buf))) {
            printf("No message in slot %d\n", i);
            continue;
        }
 
        /* Parse topic and payload from +QMTRECV response */
        char *topic_start = strchr(recv_buf, '\"');
        if (!topic_start) continue;
        char *topic_end = strchr(topic_start + 1, '\"');
        if (!topic_end) continue;
 
        char *len_start     = strchr(topic_end + 1, ',');
        char *payload_start = strchr(len_start ? len_start : topic_end, '\"');
        if (!payload_start) continue;
 
        if (provision == 0) {
            /* Provisioning response — find the end of the JSON payload */
            char *ptr = payload_start + 1;
            int qcount = 1;
            char *q10 = NULL, *q24 = NULL;
            while (*ptr) {
                if (*ptr == '\"') {
                    qcount++;
                    if (qcount == 10) q10 = ptr;
                    if (qcount == 24) { q24 = ptr; break; }
                }
                ptr++;
            }
            char *pend = q24 ? q24 : q10;
            if (!pend) {
                ESP_LOGE(TAG_GSM, "Provisioning response parse error — restarting");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
            *topic_end = *pend = '\0';
        } else {
            char *ptr = payload_start + 1;
            int qcount = 1;
            while (*ptr && qcount < 8) {
                if (*ptr == '\"') qcount++;
                ptr++;
            }
            if (qcount < 8 || !*ptr) continue;
            *topic_end = *ptr = '\0';
        }
 
        char topic[64], topic_id[64], json_str[512];
        strncpy(topic,    topic_start + 1,   sizeof(topic)    - 1);
        strncpy(json_str, payload_start + 1, sizeof(json_str) - 1);
        topic[sizeof(topic)       - 1] = '\0';
        json_str[sizeof(json_str) - 1] = '\0';
 
        const char *id_str = strrchr(topic, '/');
        int request_id = -1;
        if (id_str) {
            strncpy(topic_id, id_str + 1, sizeof(topic_id) - 1);
            topic_id[sizeof(topic_id) - 1] = '\0';
            request_id = atoi(topic_id);
        }
        if (request_id < 0) continue;
 
        printf("RX [%s]: %s  req_id=%d\n", topic, json_str, request_id);
 
        rgb_led_notify_rpc();
 
        cJSON *root = cJSON_Parse(json_str);
        if (!root) {
            ESP_LOGE(TAG_GSM, "JSON parse failed: %s", json_str);
            continue;
        }
 
        /* ----------------------------------------------------------------
         * Provisioning response handling
         * ---------------------------------------------------------------- */
        if (provision == 0) {
            cJSON *status_item = cJSON_GetObjectItem(root, "status");
            if (!status_item || !cJSON_IsString(status_item)) {
                cJSON_Delete(root);
                ESP_LOGE(TAG_GSM, "Provisioning: bad status field — restarting");
                esp_restart();
            }
 
            if (strcmp(status_item->valuestring, "SUCCESS") == 0) {
                /* Extract the credentialsValue object */
                cJSON *creds_val = cJSON_GetObjectItem(root, "credentialsValue");
                if (creds_val && cJSON_IsObject(creds_val)) {
                    cJSON *uname  = cJSON_GetObjectItem(creds_val, "userName");
                    cJSON *passwd = cJSON_GetObjectItem(creds_val, "password");
                    cJSON *cid    = cJSON_GetObjectItem(creds_val, "clientId");
 
                    if (uname && passwd && cid &&
                        cJSON_IsString(uname) &&
                        cJSON_IsString(passwd) &&
                        cJSON_IsString(cid)) {
 
                        tcu_mqtt_creds_t new_creds;
                        strncpy(new_creds.username,    uname->valuestring,  TCU_CRED_MAX_LEN - 1);
                        strncpy(new_creds.password,    passwd->valuestring, TCU_CRED_MAX_LEN - 1);
                        strncpy(new_creds.client_id,   cid->valuestring,    TCU_CRED_MAX_LEN - 1);
                        strncpy(new_creds.device_name, HUB_NAME,            TCU_CRED_MAX_LEN - 1);
                        new_creds.username[TCU_CRED_MAX_LEN - 1]    = '\0';
                        new_creds.password[TCU_CRED_MAX_LEN - 1]    = '\0';
                        new_creds.client_id[TCU_CRED_MAX_LEN - 1]   = '\0';
                        new_creds.device_name[TCU_CRED_MAX_LEN - 1] = '\0';
 
                        if (tcu_nvs_save_creds(&new_creds) == ESP_OK) {
                            ESP_LOGI(TAG_GSM, "Provisioning SUCCESS — credentials saved, restarting");
                        } else {
                            ESP_LOGE(TAG_GSM, "Failed to save provisioning credentials!");
                        }
                    } else {
                        ESP_LOGE(TAG_GSM, "Provisioning: missing credential fields");
                    }
                } else {
                    ESP_LOGE(TAG_GSM, "Provisioning: credentialsValue missing or wrong type");
                }
                provision = 1;
            } else {
                ESP_LOGE(TAG_GSM, "Provisioning failed with status: %s",
                         status_item->valuestring);
            }
 
            cJSON_Delete(root);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
            continue;
        }
 
        /* ----------------------------------------------------------------
         * Normal RPC command handling
         * ---------------------------------------------------------------- */
        cJSON *method = cJSON_GetObjectItem(root, "method");
        cJSON *params = cJSON_GetObjectItem(root, "params");
 
        if (!method || !params || !cJSON_IsString(method)) {
            cJSON_Delete(root);
            continue;
        }
 
        const char *mstr     = method->valuestring;
        const char *cmd_type = NULL;
 
        /* Method format: "TR<TYPE>" e.g. "TRPWR", TRSW (to clear NVS credentials) */
        if (strlen(mstr) >= 4 && mstr[0] == 'T' && mstr[1] == 'R') {
            cmd_type = &mstr[2];
        }
 
        bool cmd_executed = false;
        uint8_t cmd_value = 0;
 
        if (cmd_type && strcmp(cmd_type, "PWR") == 0 &&
            cJSON_IsNumber(params)) {
            cmd_value = (uint8_t)params->valueint;
            printf("TCU PWR command: %d\n", cmd_value);
            trike_ctrl_handle_command(cmd_value);
            cmd_executed = true;
		} else if (cmd_type && strcmp(cmd_type, "SW") == 0) {
		    ESP_LOGW(TAG_GSM, "Trike swap RPC received — forcing re-provisioning");
		    tcu_force_reprovision();
		    esp_restart();
		}        
         else {
            ESP_LOGW(TAG_GSM, "Unknown command: %s", mstr);
        }
 
        /* ----------------------------------------------------------------
         * Build and queue RPC acknowledgement response
         * ----------------------------------------------------------------
         * Publishing a response to v1/devices/me/rpc/response/{requestId}
         * tells ThingsBoard the command was received and acted upon.
         * Without this, TB marks the RPC as "timed out" on the dashboard.
         * ---------------------------------------------------------------- */
        if (cmd_executed && response_count < (int)(sizeof(responses) / sizeof(responses[0]))) {
            cJSON *resp_json = cJSON_CreateObject();
            cJSON_AddStringToObject(resp_json, "status", "OK");
            cJSON_AddStringToObject(resp_json, "method", mstr);
            if (strcmp(cmd_type, "PWR") == 0) {
                /* Echo back the current power state after executing command */
                cJSON_AddBoolToObject(resp_json, "power",
                                      trike_ctrl_get_power_confirmation());
                cJSON_AddNumberToObject(resp_json, "requestedValue", cmd_value);
            }
            responses[response_count].request_id       = request_id;
            responses[response_count].response_payload = cJSON_PrintUnformatted(resp_json);
            cJSON_Delete(resp_json);
            response_count++;
        }
 
        cJSON_Delete(root);
    }
 
    /* Send all accumulated RPC responses in one pass */
    if (response_count > 0) {
        ESP_LOGI(TAG_GSM, "Publishing %d RPC response(s)", response_count);
        publish_rpc_responses(responses, response_count);
    }
 
    return true;
}

/* =========================================================================
 * URC HANDLER
 * ========================================================================= */

bool check_mqtt_urc(void)
{
    static char urc_buf[192];
    static int  urc_idx = 0;
    uint8_t     byte;

    while (uart_read_bytes(MODEM_UART_NUM, &byte, 1,
                           20 / portTICK_PERIOD_MS) > 0) {
        if (urc_idx < (int)(sizeof(urc_buf) - 1)) {
            urc_buf[urc_idx++] = (char)byte;

            if (byte == '\n') {
                urc_buf[urc_idx] = '\0';

                if (strstr(urc_buf, "+QMTRECV:")) {
                    urc_notification_t notif = {0};
                    if (sscanf(urc_buf, "+QMTRECV: %" SCNu8 ",%" SCNu8,
                               &notif.client_idx, &notif.recv_id) == 2) {
                        notif.timestamp_ms = (uint32_t)(
                            xTaskGetTickCount() * portTICK_PERIOD_MS);
                        if (urc_notification_queue) {
                            if (xQueueSend(urc_notification_queue,
                                           &notif, 0) != pdPASS) {
                                ESP_LOGE(TAG_GSM, "URC queue full!");
                            }
                        }
                    }
                    urc_idx = 0;
                    return true;
                } else if (strstr(urc_buf, "+QMTSTAT:")) {
                    int cidx, ecode;
                    if (sscanf(urc_buf, "+QMTSTAT: %d,%d",
                               &cidx, &ecode) == 2) {
                        ESP_LOGE(TAG_GSM, "MQTT error: %d", ecode);
                        handle_mqtt_error(ecode);
                    }
                }
                urc_idx = 0;
                return true;
            }
        } else {
            urc_idx = 0;
        }
    }
    return false;
}

/* =========================================================================
 * MODEM INIT / DEINIT
 * ========================================================================= */

bool poweron_modem(void)
{
    uart_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_rom_gpio_pad_select_gpio(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 1);
    esp_rom_gpio_pad_select_gpio(RESET_PIN);
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_PIN, 0);
    
/*     Configure PWRKEY — idle state is HIGH (not asserted) 
    esp_rom_gpio_pad_select_gpio(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 1);   // ← HIGH idle, NOT low

     Configure RESET_N — idle state is HIGH (not asserted, active low) 
    esp_rom_gpio_pad_select_gpio(RESET_PIN);
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_PIN, 1); // ← HIGH idle, NOT low */   
    
     /* Configure modem control DTR(MCU output pin) and RI (MCU input pin) */
    gpio_config_t dtr_io = {
        .pin_bit_mask = (1ULL << BOARD_MODEM_DTR_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&dtr_io);

    gpio_config_t ri_io = {
        .pin_bit_mask = (1ULL << BOARD_MODEM_RI_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ri_io);
    
    /* Spec §3.8.1 Note 1: VBAT stable ≥ 30 ms before PWRKEY pulse.
     * VBAT is already stable well before app_main reaches here.
     * This 200 ms settle ensures PWRKEY has been HIGH long enough
     * for a clean idle state before the first ON pulse. */
    vTaskDelay(pdMS_TO_TICKS(200));
          
    gsm_poweron();



    for (int attempt = 0; attempt < 2; attempt++) {
        if (send_at_command("AT", "OK", RETRIES, CMD_DELAY_MS, NULL, 0)) {
            ESP_LOGI(TAG_GSM, "Modem online");
            return true;
        }
        ESP_LOGW(TAG_GSM, "Modem not responding (attempt %d)", attempt + 1);
        gsm_poweron();
    }

    ESP_LOGE(TAG_GSM, "Modem power-on failed");
    return false;
}

bool activate_pdp(void)
{
    for (int attempt = 0; attempt < 5; attempt++) {
        if (send_at_command("AT+CGREG?", "5", RETRIES,
                            CMD_DELAY_MS, NULL, 0)) {
            ESP_LOGI(TAG_GSM, "Network registered");
            break;
        }
        if (attempt == 4) {
            ESP_LOGE(TAG_GSM, "Network registration failed");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10 * (attempt + 1)));
    }

    char apncmd[60];
    snprintf(apncmd, sizeof(apncmd),
             "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", APN);

    for (int attempt = 0; attempt < RETRIES; attempt++) {
        /* Deactivate first in case a stale context exists */
        if (attempt > 0) {
            send_at_command("AT+QIDEACT=1", "OK", 2, 5000, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (send_at_command(apncmd, "OK", RETRIES, CMD_DELAY_MS, NULL, 0) &&
            send_at_command("AT+QIACT=1", "OK", 1, 10000, NULL, 0)) {
            ESP_LOGI(TAG_GSM, "PDP activated");
            break;
        }
        if (attempt == RETRIES - 1) {
            ESP_LOGE(TAG_GSM, "PDP activation failed");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10 * (attempt + 1)));
    }

    for (int attempt = 0; attempt < 3; attempt++) {
        if (configure_mqtt_settings()) break;
        if (attempt == 2) return false;
    }
    return true;
}
 
/* =========================================================================
 * OPEN MQTT WITH EXPLICIT CREDENTIALS
 * =========================================================================
 * Replaces the old open_mqtts() and open_provision_mqtts() with a single
 * parameterised function.  Both normal and provisioning paths call this.
 * ========================================================================= */
 
/*bool open_mqtts_with_creds(const char *client_id,
                             const char *username,
                             const char *password)
{
     Open the MQTT connection 
    char open_cmd[100];
    snprintf(open_cmd, sizeof(open_cmd),
             "AT+QMTOPEN=0,\"%s\",%d", MQTT_BROKER, MQTT_PORT);
 
    for (int attempt = 0; attempt < 3; attempt++) {
        if (send_at_command(open_cmd, "+QMTOPEN: 0,0", RETRIES,
                            CMD_DELAY_MS, NULL, 0)) break;
        if (attempt == 2) {
            ESP_LOGE(TAG_GSM, "MQTT open failed");
            return false;
        }
    }
 
     Connect with provided credentials 
    char conn_cmd[200];
    if (password && strlen(password) > 0) {
        snprintf(conn_cmd, sizeof(conn_cmd),
                 "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                 client_id, username, password);
    } else {
         Provisioning uses username only, no password 
        snprintf(conn_cmd, sizeof(conn_cmd),
                 "AT+QMTCONN=0,\"%s\",\"%s\"",
                 client_id, username);
    }
 
    mqtt_auth_failed = false;   Reset before attempt 
 
    for (int attempt = 0; attempt < 3; attempt++) {
        if (send_at_command(conn_cmd, "+QMTCONN: 0,0", RETRIES,
                            CMD_DELAY_MS, NULL, 0)) {
            ESP_LOGI(TAG_GSM, "MQTT connected as %s", username);
            return true;
        }
 
         Auth failure detected in send_at_command via return code 5 
        if (mqtt_auth_failed) {
            ESP_LOGE(TAG_GSM, "MQTT auth rejected — credentials invalid on broker");
            return false;
        }
    }
 
    ESP_LOGE(TAG_GSM, "MQTT CONNECT failed for client=%s user=%s", client_id, username);
    return false;
}*/

bool open_mqtts_with_creds(const char *client_id,
                             const char *username,
                             const char *password)
{
    char open_cmd[100];
    snprintf(open_cmd, sizeof(open_cmd),
             "AT+QMTOPEN=0,\"%s\",%d", MQTT_BROKER, MQTT_PORT);

    /* QMTOPEN: TCP connection to broker can take up to 15s over cellular.
     * The modem immediately ACKs the command with OK, then later sends
     * +QMTOPEN: 0,0 when the TCP connection is established.
     * Must wait for the URC, not the OK — use a long timeout and search
     * for +QMTOPEN in the response buffer, not just OK. */
    bool open_ok = false;
    for (int attempt = 0; attempt < 3 && !open_ok; attempt++) {
        char open_buf[128] = {0};
        /* 15000ms timeout to capture the async +QMTOPEN: 0,0 URC */
        if (send_at_command(open_cmd, "+QMTOPEN: 0,0",
                            1, 15000, open_buf, sizeof(open_buf))) {
            open_ok = true;
        } else {
            /* Check if already open (error code 2 = already connected) */
            if (strstr(open_buf, "+QMTOPEN: 0,2")) {
                ESP_LOGW(TAG_GSM, "MQTT socket already open — proceeding");
                open_ok = true;
            } else {
                ESP_LOGW(TAG_GSM, "QMTOPEN attempt %d failed: %s",
                         attempt + 1, open_buf);
                /* Close any partial connection before retry */
                send_at_command("AT+QMTCLOSE=0", "OK", 2, 5000, NULL, 0);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
    }
    if (!open_ok) {
        ESP_LOGE(TAG_GSM, "MQTT open failed after 3 attempts");
        return false;
    }

    /* QMTCONN: similarly wait longer — broker may take a moment to ACK */
    char conn_cmd[200];
    if (password && strlen(password) > 0) {
        snprintf(conn_cmd, sizeof(conn_cmd),
                 "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                 client_id, username, password);
    } else {
        snprintf(conn_cmd, sizeof(conn_cmd),
                 "AT+QMTCONN=0,\"%s\",\"%s\"",
                 client_id, username);
    }

    mqtt_auth_failed = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        char conn_buf[128] = {0};
        if (send_at_command(conn_cmd, "+QMTCONN: 0,0",
                            1, 10000, conn_buf, sizeof(conn_buf))) {
            ESP_LOGI(TAG_GSM, "MQTT connected as %s", username);
            return true;
        }
        if (mqtt_auth_failed) {
            ESP_LOGE(TAG_GSM, "MQTT auth rejected");
            return false;
        }
        ESP_LOGW(TAG_GSM, "QMTCONN attempt %d failed: %s",
                 attempt + 1, conn_buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE(TAG_GSM, "MQTT connect failed");
    return false;
}

bool mqtts_disconnect(void)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        if (send_at_command("AT+QMTDISC=0", "+QMTDISC: 0,0",
                            RETRIES, CMD_DELAY_MS, NULL, 0)) {
            return true;
        }
    }
    return false;
}

bool deactivate_pdp(void)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        if (send_at_command("AT+QIDEACT=1", "OK", RETRIES,
                            CMD_DELAY_MS, NULL, 0)) {
            return true;
        }
    }
    return false;
}

void powerdown_modem(void)
{
    bool down = false;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (send_at_command("AT+QPOWD=1", "OK", RETRIES,
                            CMD_DELAY_MS, NULL, 0)) {
            down = true;
            break;
        }
    }
    if (!down) {
        hardware_poweroff();
    }
    if (uart_installed) {
        uart_driver_delete(MODEM_UART_NUM);
        uart_installed = false;
    }
}
 
bool mqtts_init(void)
{
    if (!poweron_modem() || !activate_pdp()) {
        ESP_LOGE(TAG_GSM, "MQTT init failed at modem/PDP stage");
        powerdown_modem();
        return false;
    }
    
    /* Probe modem is still alive before attempting MQTT connection.
     * A brownout during PDP activation can leave the modem silent. */
    if (!send_at_command("AT", "OK", 3, 500, NULL, 0)) {
        ESP_LOGE(TAG_GSM, "Modem not responding after PDP — brownout suspected");
        powerdown_modem();
        vTaskDelay(pdMS_TO_TICKS(3000));   /* let supply recover */
        return false;
    }    
 
    /* Connect using runtime credentials (loaded from NVS by this point) */
    if (!open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password)) {
        /*
         * Auth failure with stored credentials means the device was
         * deleted from ThingsBoard.  Force re-provisioning.
         */
        if (mqtt_auth_failed) {
            ESP_LOGE(TAG_GSM, "Device rejected by TB — likely deleted. Triggering re-provisioning.");
            tcu_force_reprovision();
            powerdown_modem();
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_restart();
        }
        powerdown_modem();
        return false;
    }
    return true;
}
 
bool provision_mqtts_init(void)
{
    if (!poweron_modem() || !activate_pdp()) {
        powerdown_modem();
        return false;
    }
    /* Provisioning always uses the fixed TB provisioning identity */
    if (!open_mqtts_with_creds(deviceSerial, MQTT_PROV_USERNAME, MQTT_PROV_PASSWORD)) {
        powerdown_modem();
        return false;
    }
    return true;
}

void mqtts_deinit(void)
{
    if (!mqtts_disconnect() || !deactivate_pdp()) {
        powerdown_modem();
    } else {
        powerdown_modem();
    }
}

 
/* =========================================================================
 * CREDENTIAL RESOLUTION — LOAD FROM NVS OR PROVISION
 * =========================================================================
 * This is the core of the provisioning workflow.  Called once at boot
 * from mqtt_subclient() before any normal MQTT connection.
 *
 * Flow:
 *  1. Check NVS for existing credentials.
 *     a. If prov_done=1 AND load succeeds → populate runtime buffers → return true.
 *     b. If prov_done=1 BUT load fails    → NVS corrupt → repair → fall through to provision.
 *     c. If prov_done=0                   → never provisioned → fall through to provision.
 *
 *  2. Run provisioning:
 *     a. Connect to TB with username="provision".
 *     b. Subscribe to /provision/response.
 *     c. Publish to /provision/request.
 *     d. Wait TCU_PROV_RESPONSE_TIMEOUT_MS for /provision/response URC.
 *     e. read_buffered_messages() parses the response, saves to NVS, restarts.
 *
 *  NVS CORRUPTION FALLBACK:
 *  If NVS contains prov_done=1 but credential strings are unreadable
 *  (e.g. flash sector wear), tcu_nvs_repair() is called first.  The device
 *  then re-provisions using the same deterministic credential derivation
 *  (HUB_NAME-based).  Since TB uses "Allow creating new devices" strategy,
 *  re-provisioning a device that already exists updates its credentials
 *  rather than creating a duplicate.
 * ========================================================================= */
 
bool tcu_load_or_provision_creds(void)
{
/*	snprintf(mqtt_username, sizeof(mqtt_username), "%s", "tcutest");
	snprintf(mqtt_password, sizeof(mqtt_password), "%s", "tcutest");
	snprintf(mqtt_client_id, sizeof(mqtt_client_id), "%s", "4004976");*/
	
/*	snprintf(mqtt_username, sizeof(mqtt_username), "%s", "TCU_0006");
	snprintf(mqtt_password, sizeof(mqtt_password), "%s", "TCU_0006_ksc");
	snprintf(mqtt_client_id, sizeof(mqtt_client_id), "%s", "9C139E1AF430");

    ESP_LOGI(TAG_GSM, "Credentials hardcoded");	 
	return true;*/
	
	//tcu_nvs_erase_creds();
	
    /* ---- Try NVS first ---- */
    if (tcu_nvs_is_provisioned()) {
        tcu_mqtt_creds_t creds;
        if (tcu_nvs_load_creds(&creds) == ESP_OK) {
            strncpy(mqtt_username,  creds.username,  TCU_CRED_MAX_LEN - 1);
            strncpy(mqtt_password,  creds.password,  TCU_CRED_MAX_LEN - 1);
            strncpy(mqtt_client_id, creds.client_id, TCU_CRED_MAX_LEN - 1);
            mqtt_username[TCU_CRED_MAX_LEN  - 1] = '\0';
            mqtt_password[TCU_CRED_MAX_LEN  - 1] = '\0';
            mqtt_client_id[TCU_CRED_MAX_LEN - 1] = '\0';
            ESP_LOGI(TAG_GSM, "Credentials loaded from NVS — user=%s clientId=%s",
                     mqtt_username, mqtt_client_id);
            return true;
        }
 
        /* NVS read failed despite prov_done=1 — attempt repair */
        ESP_LOGE(TAG_GSM, "NVS credential read failed — running NVS repair");
        if (tcu_nvs_repair() != ESP_OK) {
            ESP_LOGE(TAG_GSM, "NVS repair failed — full erase and re-provision");
        }
        /* Fall through to provisioning */
    }
 
    /* ---- Run provisioning flow ---- */
    ESP_LOGI(TAG_GSM, "Device not provisioned — starting MQTT provisioning flow");
    provisioning_in_progress = true;
    provision = 0;  /* Signal to read_buffered_messages() we're in prov mode */
 
    for (int prov_attempt = 0; prov_attempt < TCU_PROV_MAX_RETRIES; prov_attempt++) {
        ESP_LOGI(TAG_GSM, "Provisioning attempt %d/%d",
                 prov_attempt + 1, TCU_PROV_MAX_RETRIES);
 
        if (!mqtt_provision_subclient()) {
            ESP_LOGE(TAG_GSM, "Failed to connect provisioning client");
            vTaskDelay(pdMS_TO_TICKS(5000 * (prov_attempt + 1)));
            continue;
        }
 
        if (!mqtt_provision_pubclient()) {
            ESP_LOGE(TAG_GSM, "Failed to publish provisioning request");
            mqtts_disconnect();
            vTaskDelay(pdMS_TO_TICKS(5000 * (prov_attempt + 1)));
            continue;
        }
        
		/* Wait for /provision/response URC */
		ESP_LOGI(TAG_GSM, "Waiting for provisioning response...");
		uint32_t t_start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
		bool prov_response_received = false;
		
		while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - t_start
		       < TCU_PROV_RESPONSE_TIMEOUT_MS) {
		
		    /* --- Strategy 1: check if URC was intercepted by send_at_command() --- */
		    urc_notification_t notif;
		    if (xQueueReceive(urc_notification_queue, &notif, 0) == pdPASS) {
		        ESP_LOGI(TAG_GSM, "Provisioning URC received (slot %d) — fetching...",
		                 notif.recv_id);
		        read_buffered_messages(notif.recv_id);
		        /* read_buffered_messages() calls esp_restart() on success — 
		         * if we return here, parsing failed */
		        prov_response_received = true;
		        break;
		    }
		
		    /* --- Strategy 2: directly poll UART for URC (with mutex) --- */
		    if (xSemaphoreTake(uart_modem_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
		        /* Drain UART looking for +QMTRECV */
		        static char prov_urc_buf[128];
		        static int  prov_urc_idx = 0;
		        uint8_t b;
		        bool found_urc = false;
		
		        uint64_t t0 = esp_timer_get_time();
		        while ((esp_timer_get_time() - t0) < 150000ULL) { /* 150 ms scan */
		            if (uart_read_bytes(MODEM_UART_NUM, &b, 1,
		                                5 / portTICK_PERIOD_MS) > 0) {
		                if (prov_urc_idx < (int)(sizeof(prov_urc_buf) - 1)) {
		                    prov_urc_buf[prov_urc_idx++] = (char)b;
		                }
		                if (b == '\n') {
		                    prov_urc_buf[prov_urc_idx] = '\0';
		                    if (strstr(prov_urc_buf, "+QMTRECV:")) {
		                        found_urc = true;
		                        urc_notification_t prov_notif = {0};
		                        sscanf(prov_urc_buf, "+QMTRECV: %" SCNu8 ",%" SCNu8,
		                               &prov_notif.client_idx, &prov_notif.recv_id);
		                        /* Queue it for read_buffered_messages() */
		                        xQueueSend(urc_notification_queue, &prov_notif, 0);
		                        ESP_LOGI(TAG_GSM,
		                                 "Provisioning: QMTRECV found in direct poll, slot=%d",
		                                 prov_notif.recv_id);
		                    }
		                    prov_urc_idx = 0;
		                    memset(prov_urc_buf, 0, sizeof(prov_urc_buf));
		                }
		            }
		        }
		        xSemaphoreGive(uart_modem_mutex);
		
		        if (found_urc) {
		            /* Process it immediately */
		            urc_notification_t queued;
		            if (xQueueReceive(urc_notification_queue, &queued,
		                              pdMS_TO_TICKS(100)) == pdPASS) {
		                read_buffered_messages(queued.recv_id);
		                /* Returns only on parse failure; restart handles success */
		                prov_response_received = true;
		            }
		            break;
		        }
		    }
		
		    vTaskDelay(pdMS_TO_TICKS(50)); /* tight poll — don't miss the URC */
		}
		
		if (!prov_response_received) {
		    ESP_LOGW(TAG_GSM, "Provisioning response timed out on attempt %d",
		             prov_attempt + 1);
		}        
 
/*        //Wait for /provision/response URC 
        ESP_LOGI(TAG_GSM, "Waiting for provisioning response...");
        uint32_t t_start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
 
        while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - t_start
               < TCU_PROV_RESPONSE_TIMEOUT_MS) {
            if (check_mqtt_urc()) {
                urc_notification_t notif;
                if (xQueueReceive(urc_notification_queue, &notif,
                                  pdMS_TO_TICKS(500)) == pdPASS) {
                    
                     // read_buffered_messages() in provisioning mode parses
                     // the response, saves credentials to NVS, and calls
                     // esp_restart().  Execution does not return here.
                     
                    read_buffered_messages(notif.recv_id);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
 
        ESP_LOGW(TAG_GSM, "Provisioning response timed out on attempt %d",
                 prov_attempt + 1);*/
        mqtts_disconnect();
        powerdown_modem();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    provisioning_in_progress = false; /* only reached on full failure */
 
    ESP_LOGE(TAG_GSM, "Provisioning failed after %d attempts — restarting",
             TCU_PROV_MAX_RETRIES);
    powerdown_modem();
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
    return false;  /* Never reached */
}
 
esp_err_t tcu_force_reprovision(void)
{
    ESP_LOGW(TAG_GSM, "Force re-provisioning requested — erasing NVS credentials");
    esp_err_t err = tcu_nvs_erase_creds();
    if (err == ESP_OK) {
        ESP_LOGI(TAG_GSM, "NVS credentials erased — device will re-provision on next boot");
    } else {
        ESP_LOGE(TAG_GSM, "Failed to erase NVS credentials: %s", esp_err_to_name(err));
    }
    return err;
}

/* =========================================================================
 * NORMAL MQTT SUBSCRIBE + START
 * ========================================================================= */
 
/*bool mqtt_subclient(void)
{
    
     * STEP 1: Resolve credentials — load from NVS or run full provisioning.
     * This call will restart the device after successful provisioning.
     * If we reach STEP 2, credentials are in mqtt_username/password/client_id.
     
    if (!tcu_load_or_provision_creds()) {
        return false;
    }
 
     STEP 2: Establish normal MQTT connection with provisioned credentials 
    for (int attempt = 0; attempt < 2; attempt++) {
        if (mqtts_init()) break;
        if (attempt == 1) {
            powerdown_modem();
            return false;
        }
	     Give GPIO and UART peripheral time to fully release before retry 
	    vTaskDelay(pdMS_TO_TICKS(500));
    }
 
     STEP 3: Subscribe to RPC request topic 
    char sub_cmd[100];
    snprintf(sub_cmd, sizeof(sub_cmd),
             "AT+QMTSUB=0,1,\"v1/devices/me/rpc/request/+\",0");
 
    for (int attempt = 0; attempt < 3; attempt++) {
        if (send_at_command(sub_cmd, "+QMTSUB: 0,1,0,0",
                            RETRIES, CMD_DELAY_MS, NULL, 0)) {
            prov_resub = false;
            ESP_LOGI(TAG_GSM, "Subscribed to RPC topic as %s", mqtt_username);
            return true;
        }
    }
 
    ESP_LOGE(TAG_GSM, "RPC subscription failed");
    return false;
}*/

bool mqtt_subclient(void)
{
    if (!tcu_load_or_provision_creds()) {
        return false;
    }

    for (int attempt = 0; attempt < 2; attempt++) {
        if (mqtts_init()) break;
        if (attempt == 1) {
            powerdown_modem();
            return false;
        }
    }

    /* QMTSUB: wait for async +QMTSUB URC — modem first ACKs OK,
     * then sends +QMTSUB: 0,1,0,0 when subscription is confirmed.
     * Use a long timeout and accept either form as success. */
    char sub_cmd[100];
    snprintf(sub_cmd, sizeof(sub_cmd),
             "AT+QMTSUB=0,1,\"v1/devices/me/rpc/request/+\",0");

    for (int attempt = 0; attempt < 3; attempt++) {
        char sub_buf[128] = {0};
        /* 8s timeout to capture async +QMTSUB URC */
        bool ok = send_at_command(sub_cmd, "+QMTSUB: 0,1,0,0", 1, 8000,
                                  sub_buf, sizeof(sub_buf));
        if (ok) {
            prov_resub = false;
            ESP_LOGI(TAG_GSM, "Subscribed to RPC topic");
            return true;
        }
        /* Fallback: if we got OK but no URC yet, the sub likely succeeded
         * — the URC may arrive slightly after the timeout window */
        if (strstr(sub_buf, "OK")) {
            ESP_LOGW(TAG_GSM, "QMTSUB got OK without URC — treating as success");
            prov_resub = false;
            return true;
        }
        ESP_LOGW(TAG_GSM, "QMTSUB attempt %d: %s", attempt + 1, sub_buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE(TAG_GSM, "RPC subscription failed");
    return false;
}

/*bool mqtt_provision_subclient(void)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        if (provision_mqtts_init()) break;
        if (attempt == 1) {
            powerdown_modem();
            return false;
        }
    }

    for (int attempt = 0; attempt < 3; attempt++) {
        if (send_at_command(
                "AT+QMTSUB=0,1,\"/provision/response\",0",
                "+QMTSUB: 0,1,1,1", RETRIES, CMD_DELAY_MS, NULL, 0)) {
            prov_resub = true;
            return true;
        }
    }
    return false;
}*/

bool mqtt_provision_subclient(void)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        if (provision_mqtts_init()) break;
        if (attempt == 1) {
            powerdown_modem();
            return false;
        }
    }
    
    /* Flush any stale URCs before subscribing */
    uart_flush_input(MODEM_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Use long timeout — +QMTSUB URC arrives async after broker ACK.
     * Also accept bare OK as success (URC may arrive slightly late). */
    for (int attempt = 0; attempt < 3; attempt++) {
        char sub_buf[128] = {0};
        bool ok = send_at_command(
            "AT+QMTSUB=0,1,\"/provision/response\",0",
            "+QMTSUB: 0,1,1,1",
            1, 8000,           /* ← was RETRIES + CMD_DELAY_MS */
            sub_buf, sizeof(sub_buf));
        if (ok) {
            prov_resub = true;
            ESP_LOGI(TAG_GSM, "Provisioning subscribed to /provision/response");
            return true;
        }
        /* Fallback: OK without URC — subscription likely succeeded */
        if (strstr(sub_buf, "OK")) {
            ESP_LOGW(TAG_GSM, "QMTSUB got OK without URC — treating as success");
            prov_resub = true;
            return true;
        }
        ESP_LOGW(TAG_GSM, "Provisioning QMTSUB attempt %d failed: %.80s",
                 attempt + 1, sub_buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGE(TAG_GSM, "Provisioning subscription failed after 3 attempts");
    return false;
}

 
bool mqtt_resubscribe(void)
{
    char sub_cmd[100];
    if (prov_resub) {
        snprintf(sub_cmd, sizeof(sub_cmd),
                 "AT+QMTSUB=0,1,\"/provision/response\",0");
    } else {
        snprintf(sub_cmd, sizeof(sub_cmd),
                 "AT+QMTSUB=0,1,\"v1/devices/me/rpc/request/+\",0");
    }
    for (int attempt = 0; attempt < 2; attempt++) {
        if (send_at_command(sub_cmd, "+QMTSUB: 0,1,0,0",
                            RETRIES, CMD_DELAY_MS, NULL, 0))
            return true;
    }
    return false;
}

void check_mqtt_link_status(void)
{
    if (!send_at_command("AT+QMTCONN?", "3", RETRIES,
                         CMD_DELAY_MS, NULL, 0)) {
        ESP_LOGE(TAG_GSM, "MQTT disconnected — reconnecting...");
        mqtts_error_reconnect();
    }
}

bool configure_mqtt_settings(void)
{
    return send_at_command("AT+QMTCFG=\"recv/mode\",0,1,1", "OK",
                           RETRIES, CMD_DELAY_MS, NULL, 0);
}

bool hex_to_ascii(const char *hex, char *output)
{
    if (strlen(hex) % 2 != 0) return false;
    size_t len = strlen(hex);
    for (size_t i = 0; i < len; i += 2) {
        char hb[3] = {hex[i], hex[i+1], '\0'};
        *output++ = (char)strtol(hb, NULL, 16);
    }
    *output = '\0';
    return true;
}

 
void handle_mqtt_error(int err_code)
{
    switch (err_code) {
        case 1: case 3: case 4: case 6:
            mqtts_error_reconnect();
            break;
        case 2:
            gsm_reset();
            if (!activate_pdp() ||
                !open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password) ||
                !mqtt_resubscribe()) {
                powerdown_modem();
                esp_restart();
            }
            break;
        case 5:
            /* Auth failure — device was deleted on TB */
            ESP_LOGE(TAG_GSM, "MQTT auth error — forcing re-provisioning");
            tcu_force_reprovision();
            powerdown_modem();
            esp_restart();
            break;
        case 7:
            powerdown_modem();
            esp_restart();
            break;
        default:
            powerdown_modem();
            esp_restart();
            break;
    }
}
 
void mqtts_error_reconnect(void)
{
    if (!open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password)) {
        gsm_reset();
        if (!activate_pdp() ||
            !open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password) ||
            !mqtt_resubscribe()) {
            powerdown_modem();
            esp_restart();
        }
        return;
    }
    if (!mqtt_resubscribe()) {
        gsm_reset();
        if (!activate_pdp() ||
            !open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password) ||
            !mqtt_resubscribe()) {
            powerdown_modem();
            esp_restart();
        }
    }
}

void handle_rpc_state_request(int slave_id, int request_id,
                               const char *param_name,
                               RpcResponse *responses, int *response_count)
{
    /* Stub — extend for future state-read RPC commands */
    (void)slave_id;
    (void)request_id;
    (void)param_name;
    (void)responses;
    (void)response_count;
}

bool publish_rpc_responses(RpcResponse *responses, int count)
{
    if (!responses || count <= 0) return false;

    bool all_ok = true;
    for (int i = 0; i < count; i++) {
        char pub_cmd[128], topic[64];
        snprintf(topic, sizeof(topic),
                 "v1/devices/me/rpc/response/%d", responses[i].request_id);
        snprintf(pub_cmd, sizeof(pub_cmd),
                 "AT+QMTPUBEX=0,0,0,0,\"%s\",%d",
                 topic, (int)strlen(responses[i].response_payload));

        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            if (!send_at_command(pub_cmd, ">", RETRIES,
                                 CMD_DELAY_MS, NULL, 0)) {
                vTaskDelay(pdMS_TO_TICKS(10 * (attempt + 1)));
                continue;
            }
            uart_write_bytes(MODEM_UART_NUM, responses[i].response_payload,
                             strlen(responses[i].response_payload));
            vTaskDelay(pdMS_TO_TICKS(10));
            uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
            vTaskDelay(pdMS_TO_TICKS(3));
            ok = true;
        }

        if (!ok) all_ok = false;
        free(responses[i].response_payload);
    }
    return all_ok;
}

/* =========================================================================
 * PROVISIONING PAYLOAD BUILDER
 * =========================================================================
 * Builds the MQTT_BASIC provisioning request per ThingsBoard spec:
 * {
 *   "deviceName":            "TCU_A4CF12B3E501",
 *   "provisionDeviceKey":    "c1mtrworzsysp7vrth4z",
 *   "provisionDeviceSecret": "30irfv1tah0d5on6b1tx",
 *   "credentialsType":       "MQTT_BASIC",
 *   "username":              "TCU_A4CF12B3E501",
 *   "password":              "TCU_A4CF12B3E501_ksc",
 *   "clientId":              "A4CF12B3E501"
 * }
 *
 * Convention: username = HUB_NAME, password = HUB_NAME + "_ksc",
 * clientId = deviceSerial (MAC).  This keeps credentials derivable
 * from the hardware identity so they can be reconstructed if NVS
 * is corrupted (see NVS fallback discussion in header comments).
 * ========================================================================= */
 
char *create_provision_json_payload(void)
{
    char derived_password[64];
    snprintf(derived_password, sizeof(derived_password), "%s_ksc", HUB_NAME);
 
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "deviceName",            HUB_NAME);
    cJSON_AddStringToObject(root, "provisionDeviceKey",    PROVISION_KEY);
    cJSON_AddStringToObject(root, "provisionDeviceSecret", PROVISION_SECRET);
    cJSON_AddStringToObject(root, "credentialsType",       CREDENTIALS_TYPE);
    cJSON_AddStringToObject(root, "username",              HUB_NAME);
    cJSON_AddStringToObject(root, "password",              derived_password);
    cJSON_AddStringToObject(root, "clientId",              deviceSerial);
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return s;
}

bool gps_mqtt_init(void)
{
    gps_config_t cfg;
    gps_get_default_config(&cfg);
    return (gps_init(&cfg) == ESP_OK);
}

bool mqtt_dummypub(void)
{
    const char *payload = "ping";
    char cmd[80];
    snprintf(cmd, sizeof(cmd),
             "AT+QMTPUBEX=0,0,0,0,\"ksc/uplink/telemetry\",%d",
             (int)strlen(payload));
    for (int attempt = 0; attempt < 3; attempt++) {
        if (!send_at_command(cmd, ">", RETRIES, CMD_DELAY_MS, NULL, 0)) {
            continue;
        }
        uart_write_bytes(MODEM_UART_NUM, payload, strlen(payload));
        vTaskDelay(pdMS_TO_TICKS(12));
        uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
        vTaskDelay(pdMS_TO_TICKS(3));
        return true;
    }
    return false;
}

bool mqtt_provision_pubclient(void)
{
    char *payload = create_provision_json_payload();
    char cmd[100];
    snprintf(cmd, sizeof(cmd),
             "AT+QMTPUBEX=0,0,0,0,\"/provision/request\",%d",
             (int)strlen(payload));

    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
        if (!send_at_command(cmd, ">", RETRIES, CMD_DELAY_MS, NULL, 0)) {
            continue;
        }
        uart_write_bytes(MODEM_UART_NUM, payload, strlen(payload));
        vTaskDelay(pdMS_TO_TICKS(200));
        uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        ok = true;
    }
    free(payload);
    return ok;
}

uint8_t get_average_battery_soc(void)
{
    return average_battery_soc;
}

/* =========================================================================
 * GSM SIGNAL QUALITY
 * ========================================================================= */

/**
 * @brief Read current GSM/LTE signal quality via AT+CSQ.
 *
 * Per EG915N AT Commands Manual: response time 300 ms.
 * rssi=99 = not detectable (no signal).
 * Valid rssi range: 0–31 (GSM) or 100–191 (LTE extended RSSI).
 *
 * @param[out] rssi_out   Raw RSSI value (0–31, 99=invalid, 100–191 LTE).
 * @param[out] dbm_out    Calibrated value in dBm (-113 to -51, or INT16_MIN if invalid).
 * @return true if a valid reading was obtained.
 */
bool get_gsm_signal_quality(int *rssi_out, int *dbm_out)
{
    if (!rssi_out || !dbm_out) return false;

    char raw[128] = {0};
    bool ok = send_at_command("AT+CSQ", "+CSQ:", 4, 1000, raw, sizeof(raw));
    if (!ok) {
        *rssi_out = 99;
        *dbm_out  = -999;
        return false;
    }

    char *p = strstr(raw, "+CSQ:");
    if (!p) {
        *rssi_out = 99;
        *dbm_out  = -999;
        return false;
    }

    int rssi = 99, ber = 99;
    sscanf(p, "+CSQ: %d,%d", &rssi, &ber);

    *rssi_out = rssi;

    if (rssi == 99) {
        *dbm_out = -999;
        return false;
    }

    /* Convert to dBm per 3GPP TS 27.007 */
    if (rssi <= 31) {
        *dbm_out = -113 + 2 * rssi;
    } else if (rssi >= 100 && rssi <= 191) {
        /* LTE extended: maps -116 to +25 dBm in 1 dB steps */
        *dbm_out = -116 + (rssi - 100);
    } else {
        *dbm_out = -999;
        return false;
    }

    return true;
}