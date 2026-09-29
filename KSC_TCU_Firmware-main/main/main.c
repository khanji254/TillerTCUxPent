/**
 * @file main.c
 * @brief KSC TCU V1.1 — ESP32-S3 Multi-Battery Monitor System
 *
 * Application entry point and MQTT publish task for the Trike Control Unit
 * Version 1.1.  Combines JK-BMS battery monitoring, Quectel EG915N MQTT
 * telemetry, GPS position streaming, trike power control, accelerometer
 * motion detection, and analogue sensor readings.
 *
 * Hardware: TCU V1.1 board (ESP32-S3, MC74HC4052ADG RS-485 MUX,
 *           INA240A1PWR current sense, LIS3DHTR accelerometer).
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <math.h>

#include "board_config.h"
#include "version.h"
#include "rgb_led.h"
#include "trike_sensors.h"
#include "lis3dhtr.h"
#include "power_trike_ctrl.h"
#include "Quectel_mqtt.h"
#include "Quectel_gps.h"
#include "bms_monitor_types.h"
#include "bms_monitor_task.h"
#include "factory_test.h"
#include "tcu_nvs_creds.h"

#define TAG "MAIN"

/* =========================================================================
 * TIMING CONFIGURATION
 * ========================================================================= */

#define PUBLISH_INTERVAL_MS          (60  * 1000)
#define GPS_UPDATE_INTERVAL_MS       (30  * 1000)
#define BMS_HEALTH_CHECK_INTERVAL_MS (60  * 1000)
#define KEEPALIVE_INTERVAL_MS        (30  * 1000)
#define MODEM_CHECK_INTERVAL_MS      (60  * 1000)
#define URC_POLL_INTERVAL_MS          100

/* =========================================================================
 * SAFETY THRESHOLDS
 * ========================================================================= */

/** Cross-battery voltage deviation that triggers trike shutdown (mV) */
#define BATT_VOLTAGE_DEVIATION_MV    5000

/** Average SOC below which WARN alert is activated (%) */
#define SOC_WARN_THRESHOLD_PCT       20

/** Average SOC below which CRIT alert is activated (%) */
#define SOC_CRIT_THRESHOLD_PCT       10

/* =========================================================================
 * WATCHDOG CONFIGURATION
 * =========================================================================
 * 300 s (5 min) is appropriate for production.
 * The MQTT publish task resets the watchdog every ~1 s in the main loop.
 * If the publish task hangs for more than 5 minutes the system restarts.
 * ========================================================================= */

#define WATCHDOG_TIMEOUT_S           300

/* =========================================================================
 * MODULE-LEVEL GLOBALS
 * ========================================================================= */
char deviceSerial[13] = {0};
int     slave_id  = 0;

static uint8_t returned_average_soc       = 0;
static volatile bool publishing_status_only = false, gps_fetch_lock = false;
static volatile bool impact = false;

extern bool check_trike_motion(void);
extern bool get_gsm_signal_quality(int *rssi_out, int *dbm_out);
extern volatile bool provisioning_in_progress;

/* =========================================================================
 * CHIP ID (full 6-byte MAC hex string)
 * ========================================================================= */

/**
 * @brief Build a 12-character unique device identifier from the ESP32-S3
 *        base MAC address.
 *
 * Uses all 6 MAC bytes in hex format (e.g. "A4CF12B3E501") — guaranteed
 * unique and matches the label printed on the ESP32-S3 module.
 *
 * @param[out] buf   Buffer to receive the string (minimum 13 bytes).
 * @param[in]  size  Buffer size.
 */
void getChipIdString(char *buf, size_t size)
{
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);
    snprintf(buf, size, "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device serial: %s", buf);
}

/* =========================================================================
 * BATTERY JSON PAYLOAD
 * ========================================================================= */
static void add_float(cJSON *obj, const char *key, float val, int dp)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f", dp, (double)val);
    cJSON_AddItemToObject(obj, key, cJSON_CreateRaw(buf));
}

/**
 * @brief Build the telemetry JSON payload for one battery.
 *
 * If the battery's voltage deviates more than BATT_VOLTAGE_DEVIATION_MV
 * from any other battery, field Bx_P26 is set to 1 (alarm).  When the
 * deviation clears, Bx_P26 is set to 0 so ThingsBoard can auto-clear
 * the alarm.
 *
 * @param[in] battery_data      Data for the battery being published.
 * @param[in] all_battery_data  Full array of all battery readings (for
 *                              cross-battery voltage deviation check).
 * @param[in] batteries_read    Number of valid entries in all_battery_data.
 * @return Heap-allocated JSON string.  Caller must free() after use.
 *         Returns NULL if battery_data is NULL.
 */
char *create_battery_json_payload(bms_queued_data_t *battery_data,
                                   bms_queued_data_t *all_battery_data,
                                   int batteries_read)
{
    if (!battery_data) return NULL;

    cJSON *root = cJSON_CreateObject();
    char  prefix[16], key[32];

    snprintf(prefix, sizeof(prefix), "B%d", battery_data->battery_id + 1);

#define ADD_NUM(field, value) \
    snprintf(key, sizeof(key), "%s_" #field, prefix); \
    cJSON_AddNumberToObject(root, key, (value))

    snprintf(key, sizeof(key), "%s_P1", prefix);
	add_float(root, key,
	    (float)(lroundf((float)battery_data->batt_v / 10.0f)) / 100.0f, 2);

	/* P13: Current — batt_i in mA, publish in A to 2dp */
	snprintf(key, sizeof(key), "%s_P2", prefix);
	add_float(root, key,
	    (float)(lroundf((float)battery_data->batt_i / 10.0f)) / 100.0f, 2);
	  
    snprintf(key, sizeof(key), "%s_P3", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->soc);

    snprintf(key, sizeof(key), "%s_P4", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->soh);

    snprintf(key, sizeof(key), "%s_P5", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->charge_cycles);

    snprintf(key, sizeof(key), "%s_P6", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->status);

    snprintf(key, sizeof(key), "%s_P7", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->charge_stat);

    snprintf(key, sizeof(key), "%s_P8", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->discharge_stat);

    snprintf(key, sizeof(key), "%s_P9", prefix);
    cJSON_AddStringToObject(root, key, battery_data->serial_no);

/*	 P21: Power — batt_power in mW/10, publish in kWh to 2dp 
	snprintf(key, sizeof(key), "%s_P21", prefix);
	cJSON_AddNumberToObject(root, key,
	    (float)(lroundf((float)battery_data->batt_power / 10.0f)) / 100.0f);*/

    snprintf(key, sizeof(key), "%s_P10", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->cells_diff);

    snprintf(key, sizeof(key), "%s_P11", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->alarms);

    snprintf(key, sizeof(key), "%s_P12", prefix);
    cJSON_AddNumberToObject(root, key, battery_data->cell_count);

	/* P25: Remaining capacity — in mAh/10, publish in Ah to 2dp */
	snprintf(key, sizeof(key), "%s_P13", prefix);
	add_float(root, key,
	    (float)(lroundf((float)battery_data->remaining_capacity / 10.0f)) / 100.0f, 2);   /* Cross-battery voltage deviation check */
	    
    bool voltage_anomaly = false;
    if (all_battery_data && batteries_read > 1 &&
        battery_data->status == BMS_STATUS_OK) {
        for (int j = 0; j < batteries_read; j++) {
            if (all_battery_data[j].battery_id == battery_data->battery_id)
                continue;
            if (all_battery_data[j].status != BMS_STATUS_OK)
                continue;
            int32_t diff = (int32_t)battery_data->batt_v -
                           (int32_t)all_battery_data[j].batt_v;
            if (diff < 0) diff = -diff;
            if ((uint32_t)diff > BATT_VOLTAGE_DEVIATION_MV) {
                voltage_anomaly = true;
                ESP_LOGW(TAG, "B%d voltage anomaly vs B%d: %" PRId32 " mV",
                         battery_data->battery_id + 1,
                         all_battery_data[j].battery_id + 1, diff);
                break;
            }
        }
    }

    /* Always publish P26 so ThingsBoard can clear the alarm when normal */
    snprintf(key, sizeof(key), "%s_P14", prefix);
    cJSON_AddNumberToObject(root, key, voltage_anomaly ? 1 : 0);

#undef ADD_NUM

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

/* =========================================================================
 * METADATA PAYLOAD
 * ========================================================================= */

/**
 * @brief Build the device metadata JSON payload for ThingsBoard.
 * @TODO changes to doxygen to match new parameters added
 * Fields published:
 *   "1"   — Mains present (1/0)  [always 1 on V1.1 — no mains detect pin]
 *   "2"   — Device serial (MAC string)
 *   "3"   — Valid battery count
 *   "4–7" — GPS latitude, longitude, altitude, satellites (if valid fix)
 *   "8"   — GPS fix quality
 *   "9"   — trike_power_resp_cmd (only during power-off pipeline)
 *   "10"  — power_confirmation (trike ON/OFF state)
 *   "11"  — Reset reason (omitted on clean power-on reset)
 *   "27"  — Average battery SOC (only when all batteries readable)
 *   "P28" — Trike voltage (V)
 *   "P29" — Trike current (A)
 *   "P30" — Powertrain temperature (°C)
 *   "P31" — Accelerometer roll (degrees)
 *   "P32" — Accelerometer pitch (degrees)
 *   "P33" — Impact detected and published (0/1)
 *   "P34" — Motion detected and published (0/1)
 *   "fw"  — Firmware version string
 *   "hw"  — Hardware version string
 *
 * @return Heap-allocated JSON string.  Caller must free() after use.
 */
 
 /*
 * CHANGED in this function:
 *
 * 1. P29 (pack_current_a) no longer calls trike_sensors_read_all() live.
 *    It now uses bms_monitor_get_shunt_snapshot() which was captured at the
 *    end of the most recent BMS read cycle — temporally aligned with the
 *    BMS battery data being published in the same cycle.
 *
 * 2. P_bms_cycle_ms added: reports the duration of the most recent complete
 *    3-battery read cycle in milliseconds. Provides remote visibility of
 *    Modbus retry health in the field.
 *
 * Voltage (P28) and temperature (P30) still read live — both are
 * slow-moving signals where a few seconds of latency is inconsequential.
 */
char *create_metadata_payload(float shunt_snapshot_a, float *shunt_max_out)
{

    cJSON *root = cJSON_CreateObject();

    /* Field "2": device serial */
    cJSON_AddStringToObject(root, "1", deviceSerial);

    /* Field "3": valid battery count */
    cJSON_AddNumberToObject(root, "2", bms_monitor_get_ok_count());
    
	/* Fields P37–P38: GSM signal quality and GPS satellite count */
    int gsm_rssi = 99, gsm_dbm = -999;
    if (get_gsm_signal_quality(&gsm_rssi, &gsm_dbm)) {
        cJSON_AddNumberToObject(root, "3", gsm_dbm);   /* GSM signal dBm */
    } else {
        cJSON_AddNumberToObject(root, "3", -999);      /* No signal      */
    } 
    
   

    /* Fields "4–8": GPS position */
    gps_position_t gps_pos;
    memset(&gps_pos, 0, sizeof(gps_pos));
    
    gps_fetch_lock = true;				/*Lock access of gps_get_latest_position for diag_log_task at publish time*/

    if (gps_get_latest_position(&gps_pos) == ESP_OK && gps_pos.valid) {
		add_float(root, "4", gps_pos.latitude,  6);
		add_float(root, "5", gps_pos.longitude, 6);
		add_float(root, "6", gps_pos.altitude,  1);
        cJSON_AddNumberToObject(root, "7", gps_pos.satellites);
        add_float(root, "8", gps_pos.speed_kmh, 1);
    }
    cJSON_AddNumberToObject(root, "9", gps_pos.fix);

	gps_fetch_lock = false;				/*Unlock access of gps_get_latest_position for diag_log_task at publish time*/
    // -----------------------------------------------------------------------
    //Field "9" only during trike power-off pipeline
    //Value responds to confirm if the trike will switch off (responds FALSE if power off command executed, reponds TRUE if command is rejected because trike is in motion)
    //Field "9" = TRUE means: "received the off command but trike is still moving"
    //Field "9" = FALSE means: "the power-off command was carried out"
    // -----------------------------------------------------------------------
    bool resp_cmd_val = false;
    if (trike_ctrl_consume_resp_cmd_flag(&resp_cmd_val)) {
        cJSON_AddBoolToObject(root, "10", resp_cmd_val);
    }
 
    // -----------------------------------------------------------------------
    // REQ 1: Field "10" - power_confirmation from trike_power_ctrl (NVS-backed); Also the current actual
    //power status of trike (TRUE when trike authorized, FALSE when trike unauthorized)
    // -----------------------------------------------------------------------
    cJSON_AddBoolToObject(root, "11", trike_ctrl_get_power_confirmation());

    /* Field "27": average SOC — only when all batteries are valid */
    if (!publishing_status_only) {
        returned_average_soc = get_average_battery_soc();
        if (bms_monitor_get_ok_count() == BMS_BATTERY_COUNT &&
            returned_average_soc > 0) {
            cJSON_AddNumberToObject(root, "12", returned_average_soc);
        } else {
            cJSON_AddNumberToObject(root, "12", 0);
        }
    }

    /*
     * CHANGED: ADC sensor readings split into two paths:
     *
     * P28 (voltage) and P30 (temperature): read live — slow-moving, fine.
     *
     * P29 (current): use bms_monitor_get_shunt_snapshot() instead of a
     * live ADC read. The snapshot was taken immediately after all 3 batteries
     * were read in the same BMS cycle, making it temporally consistent with
     * the battery payload published this cycle. A live read here would sample
     * at a completely different operating point — up to 60s later.
     */    

    /* Fields P28–P32: analogue sensor readings */
    trike_sensor_data_t sensors;
    if (trike_sensors_read_all(&sensors) == ESP_OK && sensors.valid) {
		add_float(root, "13", sensors.voltage_v,     2);
		add_float(root, "14", shunt_snapshot_a,      2);
		add_float(root, "15", *shunt_max_out,      2);
		add_float(root, "16", sensors.temperature_c, 1);
    }
    

	/* Fields 17–22: FIFO-windowed accelerometer statistics.
	 * Retrieves 60-second RMS-of-RMS per axis, vector magnitude, and peaks.
	 * Impact flag (22) still comes from the click interrupt — unaffected by FIFO. */
	lis3dhtr_publish_stats_t accel_stats;
	if (lis3dhtr_get_publish_stats(&accel_stats) == ESP_OK &&
	    accel_stats.windows_captured > 0) {
	    add_float(root, "17", accel_stats.x_rms60_mg,         1);  /* X RMS-of-RMS   */
	    add_float(root, "18", accel_stats.y_rms60_mg,         1);  /* Y RMS-of-RMS   */
	    add_float(root, "19", accel_stats.z_rms60_mg,         1);  /* Z RMS-of-RMS   */
	    add_float(root, "20", accel_stats.magnitude_rms60_mg, 1);  /* Vector mag     */
	    add_float(root, "21", accel_stats.x_peak_signed_mg,      1);  /* X peak signed     */
	    add_float(root, "22", accel_stats.y_peak_signed_mg,      1);  /* y peak signed     */
	    add_float(root, "23", accel_stats.z_peak_signed_mg,      1);  /* z peak signed     */
	    cJSON_AddNumberToObject(root, "24", impact ? 1 : 0);
	    /* Reset accumulator immediately after consuming */
	    lis3dhtr_accumulator_reset();
	} else {
	    /* Fallback — accumulator empty, publish zeros */
	    add_float(root, "17", 0.0f, 1);
	    add_float(root, "18", 0.0f, 1);
	    add_float(root, "19", 0.0f, 1);
	    add_float(root, "20", 0.0f, 1);
	    add_float(root, "21", 0.0f, 1);
	    add_float(root, "22", 0.0f, 1);
	    add_float(root, "23", 0.0f, 1);
	    cJSON_AddNumberToObject(root, "24", impact ? 1 : 0);
	}

		
	/* P38: trike motion — GPS speed primary (from diag_log_task cache),
	 * battery current secondary. Reflects check_trike_motion(). */
	cJSON_AddNumberToObject(root, "25", check_trike_motion() ? 1 : 0);
		    
    /* After the existing P28–P30 sensor block, add: */
	add_float(root, "P_offset_precfg", trike_sensors_get_precfg_offset(),  3);
	add_float(root, "P_offset_board",  trike_sensors_get_current_offset(), 3);
	
    
	 /*
     * NEW: P_bms_cycle_ms — duration of the most recent complete 3-battery
     * read cycle in milliseconds. Typical range 2600–5300 ms.
     * High values (>4000ms) indicate Modbus retries are occurring.
     */
    cJSON_AddNumberToObject(root, "P_bms_cycle_ms",
        bms_monitor_get_last_cycle_ms());
    cJSON_AddNumberToObject(root, "P_bms_pass1_ms",
        bms_monitor_get_last_pass1_ms());
    cJSON_AddNumberToObject(root, "P_bms_pass2_ms",
        bms_monitor_get_last_pass2_ms());        
        	    
    /* Firmware and hardware version */
    cJSON_AddStringToObject(root, "fw", FW_VERSION_STR);
    cJSON_AddStringToObject(root, "hw", HW_VERSION_STR);
    

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json;
}

/* =========================================================================
 * RS-485 UART INITIALISATION
 * ========================================================================= */

/**
 * @brief Initialise the RS-485 UART and driver.
 *
 * RS-485 half-duplex mode is used.  The MC74HC4052ADG MUX handles bus
 * routing; the UART driver handles TX/RX direction automatically.
 */
static void rs485_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = BOARD_RS485_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(BOARD_RS485_UART_NUM,
                                        256 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_RS485_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_RS485_UART_NUM,
                                  BOARD_RS485_TX_PIN,
                                  BOARD_RS485_RX_PIN,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(BOARD_RS485_UART_NUM,
                                   UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "RS-485 UART%d initialised (TX=IO%d RX=IO%d)",
             BOARD_RS485_UART_NUM,
             BOARD_RS485_TX_PIN,
             BOARD_RS485_RX_PIN);
}

static int rs485_write(const uint8_t *data, const uint16_t len)
{
    int n = uart_write_bytes(BOARD_RS485_UART_NUM, data, len);
    uart_wait_tx_done(BOARD_RS485_UART_NUM, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
    return n;
}

static int rs485_read(uint8_t *dst, uint16_t len)
{
    return uart_read_bytes(BOARD_RS485_UART_NUM, dst, len,
                           pdMS_TO_TICKS(50));
}

/* =========================================================================
 * VOLTAGE ANOMALY CHECK
 * ========================================================================= */

/**
 * @brief Check for cross-battery voltage deviation and initiate trike
 *        shutdown if an anomaly is detected.
 *
 * Called after successfully publishing battery data.
 *
 * @param[in] all_battery_data  Array of battery data from the last read cycle.
 * @param[in] batteries_read    Number of valid entries.
 */
void check_voltage_anomaly_and_act(bms_queued_data_t *all_battery_data,
                                    int batteries_read)
{
    bool anomaly = false;

    for (int i = 0; i < batteries_read && !anomaly; i++) {
        if (all_battery_data[i].status != BMS_STATUS_OK) continue;
        for (int j = i + 1; j < batteries_read; j++) {
            if (all_battery_data[j].status != BMS_STATUS_OK) continue;
            int32_t diff = (int32_t)all_battery_data[i].batt_v -
                           (int32_t)all_battery_data[j].batt_v;
            if (diff < 0) diff = -diff;
            if ((uint32_t)diff > BATT_VOLTAGE_DEVIATION_MV) {
                ESP_LOGE(TAG, "Voltage anomaly B%d vs B%d = %" PRId32 " mV",
                         all_battery_data[i].battery_id + 1,
                         all_battery_data[j].battery_id + 1, diff);
                anomaly = true;
                break;
            }
        }
    }

    if (!anomaly) return;

    if( rgb_led_set_alert(ALERT_VOLTAGE_ANOMALY, false))  ESP_LOGI(TAG, "Switched RGB LED status");
    else{
		ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
	}
    ESP_LOGW(TAG, "Triggering trike poweroff due to voltage anomaly");
    trike_ctrl_handle_command(1);
}

/* =========================================================================
 * URC PROCESSING TASK
 * ========================================================================= */

/**
 * @brief Dedicated task that processes queued MQTT URC notifications.
 *
 * Runs on Core 1.  Receives URC notification structs from the queue
 * populated by check_mqtt_urc(), then calls read_buffered_messages()
 * to parse the MQTT command and dispatch to the appropriate handler.
 *
 * @param[in] pvParameters  Unused.
 */
static void process_urcs_task(void *pvParameters)
{
    ESP_LOGI(TAG, "URC task started on Core %d", xPortGetCoreID());
    urc_notification_t notif;

    while (1) {
        if (xQueueReceive(urc_notification_queue, &notif,
                          pdMS_TO_TICKS(URC_POLL_INTERVAL_MS)) == pdPASS) {
            uint32_t age = (uint32_t)(xTaskGetTickCount() *
                           portTICK_PERIOD_MS) - notif.timestamp_ms;
            if (age > 1000) {
                ESP_LOGW(TAG, "URC latency %" PRIu32 " ms", age);
            }
            if (!read_buffered_messages(notif.recv_id)) {
                ESP_LOGE(TAG, "Failed to read slot %d", notif.recv_id);
            }
        } else {
            check_mqtt_urc();
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* =========================================================================
 * MQTT PUBLISH TASK
 * ========================================================================= */

/**
 * @brief Main MQTT publish task — runs on Core 1.
 *
 * Responsibilities:
 *   - Initialise modem MQTT connection and GPS streaming
 *   - Periodic battery telemetry and metadata publish (every 60 s)
 *   - Triage SOC-based LED alerts
 *   - Trike shutdown on low battery count or voltage anomaly
 *   - Modem health monitoring and recovery
 *   - Power event handling
 *
 * @param[in] pvParameters  Unused.
 */
static void mqtt_publish_task(void *pvParameters)
{
    /*
     * CHANGED: bms_queue (QueueHandle_t) removed entirely.
     * Battery data is now consumed via bms_monitor_get_snapshot() inside
     * mqtt_pubclient_battery() in Quectel_mqtt.c. That function holds its
     * own static s_last_consumed_cycle so it never re-publishes a cycle.
     *
     * s_snapshot_ready_check_cycle_id: used by the BMS health check to
     * detect whether the BMS task cycle ID has advanced between two
     * consecutive 60 s health check ticks.
     */
    static uint32_t s_snapshot_ready_check_cycle_id = 0;
    QueueHandle_t power_queue = power_mgmt_get_event_queue();

    ESP_LOGI(TAG, "MQTT publish task started on Core %d", xPortGetCoreID());

    static bool low_battery_shutdown_triggered = false;
    
    // URC queue must exist before provisioning flow runs,
    // because tcu_load_or_provision_creds() calls xQueueReceive on it.
    urc_queue_init();    

     //Initialise MQTT connection 
    if (!mqtt_subclient()) {
        ESP_LOGE(TAG, "mqtt_subclient failed — restarting");
        powerdown_modem();
        esp_restart();
    }

    //Initialise GPS NMEA streaming 
    if (!gps_mqtt_init()) {
        ESP_LOGW(TAG, "GPS init failed — continuing without GPS");
    }

    /* Timing state */
    uint32_t last_publish_ms      = 0;
    uint32_t last_bms_health_ms   = 0;
    uint32_t last_keepalive_ms    = 0;
    uint32_t last_modem_check_ms  = 0;
    uint32_t last_stack_check_ms  = 0;

    // Restore trike state on ThingsBoard 
    vTaskDelay(pdMS_TO_TICKS(2000));
    trike_ctrl_publish_state();

    // Register with watchdog 
    esp_task_wdt_add(NULL);

    xTaskCreatePinnedToCore(process_urcs_task, "urc_proc",
                             6144, NULL, 5, NULL, 1);

     //Transition to normal operation LED state 
    if(rgb_led_set_alert(ALERT_NORMAL, false)) ESP_LOGI(TAG, "Switched RGB LED status");
    else ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 

    while (1) {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        esp_task_wdt_reset();

        // Stack monitoring (every 60 s)

        if ((now - last_stack_check_ms) >= 60000) {
            uint32_t hwm = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "MQTT task stack HWM: %" PRIu32 " words", hwm);
            if (hwm < 256) {
                ESP_LOGE(TAG, "Stack critically low!");
            }
            last_stack_check_ms = now;
        }

/*        // Power events

        power_event_t pev;
        if (xQueueReceive(power_queue, &pev, 0) == pdPASS) {
            switch (pev.type) {
                case POWER_EVENT_MODEM_BROWNOUT:
                    ESP_LOGW(TAG, "Modem brownout — attempting recovery");
                    if (power_mgmt_recover_modem() != ESP_OK) {
                        ESP_LOGE(TAG, "Modem recovery failed — restarting");
                        esp_restart();
                    }
                    break;

                default:
                    break;
            }
        }*/

        // Keep-alive (every 30 s)

        if ((now - last_keepalive_ms) >= KEEPALIVE_INTERVAL_MS) {
            check_mqtt_link_status();
            last_keepalive_ms = now;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /*
         * CHANGED: BMS health check no longer monitors queue depth.
         * Instead checks whether the cycle ID has advanced since the
         * last publish tick, indicating the BMS task is still running
         * and producing fresh complete cycles.
         */
        if ((now - last_bms_health_ms) >= BMS_HEALTH_CHECK_INTERVAL_MS) {
            uint32_t current_cycle_id = bms_monitor_get_last_cycle_id();
            if (s_snapshot_ready_check_cycle_id == current_cycle_id &&
                bms_monitor_task_is_running()) {
                ESP_LOGW(TAG,
                         "BMS cycle ID unchanged since last check "
                         "(id=%" PRIu32 ") — task may be stalled",
                         current_cycle_id);
            }
            s_snapshot_ready_check_cycle_id = current_cycle_id;
            last_bms_health_ms = now;
        }

        // Modem health check (every 60 s) 
        
        if ((now - last_modem_check_ms) >= MODEM_CHECK_INTERVAL_MS) {
            if (!power_mgmt_check_modem_alive()) {
                ESP_LOGW(TAG, "Modem health check failed — queuing recovery");
                power_event_t ev = {
                    .type         = POWER_EVENT_MODEM_BROWNOUT,
                    .timestamp_ms = now
                };
                xQueueSend(power_queue, &ev, 0);
            }
            last_modem_check_ms = now;
        }

        // Periodic publish (every 60 s)
		impact = lis3dhtr_consume_impact();
		
		if ((now - last_publish_ms) >= PUBLISH_INTERVAL_MS || impact) {
		    if (impact) {
		        ESP_LOGI(TAG, "IMPACT DETECTED!!! Immediate publish.");
		    }
			
            uint8_t ok_count = bms_monitor_get_ok_count();
            ESP_LOGI(TAG, "Publish tick — batteries_ok=%d", ok_count);

            if (ok_count == 0 || impact) {
                // No valid batteries or impact has been detected— publish metadata only 
                publishing_status_only = true;
                if(ok_count ==0 && rgb_led_set_alert(ALERT_NO_BATTERIES, false)){ESP_LOGI(TAG, "Switched RGB LED status");}
    			else { ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
				}
                if (!mqtt_pubclient_status()) {
                    ESP_LOGE(TAG, "Metadata-only publish failed");
                }

            } else {
                publishing_status_only = false;
                /*
                 * mqtt_pubclient_battery() now consumes the atomic snapshot
                 * from bms_monitor_get_snapshot() internally (no queue).
                 * It publishes all batteries with status == BMS_STATUS_OK
                 * and skips those with BMS_STATUS_NO_DATA, so partial
                 * battery data (1 or 2 readable packs) is still published.
                 * It also updates average_battery_soc used by the LED alert
                 * and metadata field "27".
                 * Returns false only if no new cycle is available or publish failed.
                 */
                if (!mqtt_pubclient_battery()) {
                    ESP_LOGE(TAG, "Battery publish failed");
                } else {
                    ESP_LOGI(TAG, "Battery data published");

                    // Trike shutdown if fewer than all batteries readable 
                    if (ok_count < BMS_BATTERY_COUNT &&
                        !low_battery_shutdown_triggered) {
                        ESP_LOGE(TAG,
                                 "Only %d/%d batteries readable — shutdown",
                                 ok_count, BMS_BATTERY_COUNT);
                        low_battery_shutdown_triggered = true;
                        trike_ctrl_handle_command(1);
                    } else if (ok_count == BMS_BATTERY_COUNT) {
                        low_battery_shutdown_triggered = false;
                    }
                }

                // SOC-based LED alert 
                uint8_t avg_soc = get_average_battery_soc();
                bool success = true;
                if (ok_count == BMS_BATTERY_COUNT) {
                    if (avg_soc > 0 && avg_soc <= SOC_CRIT_THRESHOLD_PCT) {
                        success = rgb_led_set_alert(ALERT_LOW_SOC_CRIT, false);
                    } else if (avg_soc > SOC_CRIT_THRESHOLD_PCT &&
                               avg_soc <= SOC_WARN_THRESHOLD_PCT) {
                        success = rgb_led_set_alert(ALERT_LOW_SOC_WARN, false);
                    } else if (trike_ctrl_get_power_confirmation()) {
                        success = rgb_led_set_alert(ALERT_NORMAL, false);
                    } else {
                       success = rgb_led_set_alert(ALERT_TRIKE_OFF, false);
                    }
                }
                if(!success)ESP_LOGE(TAG, "Failed to switch RGB LED status!"); 
            }

            last_publish_ms = now;
            vTaskDelay(pdMS_TO_TICKS(10));
            check_mqtt_link_status();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* =========================================================================
 * STATISTICS TASK
 * ========================================================================= */

/**
 * @brief Periodic statistics logging task (every 30 s).
 *
 * @param[in] pvParameters  Unused.
 */
static void stats_monitor_task(void *pvParameters)
{
    bms_system_stats_t stats;
    ESP_LOGI(TAG, "Stats task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        bms_monitor_get_stats(&stats);
        ESP_LOGI(TAG, "=== BMS Statistics ===");
        ESP_LOGI(TAG, "Total: %" PRIu32 "  Overruns: %" PRIu32,
                 stats.total_readings, stats.queue_overruns);

        for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
            ESP_LOGI(TAG, "  B%d OK=%" PRIu32 " FAIL=%" PRIu32
                     " TO=%" PRIu32 " CRC=%" PRIu32,
                     i + 1,
                     stats.battery_stats[i].successful_reads,
                     stats.battery_stats[i].failed_reads,
                     stats.battery_stats[i].timeout_errors,
                     stats.battery_stats[i].crc_errors);
        }

        if (urc_notification_queue) {
            UBaseType_t uq = uxQueueMessagesWaiting(urc_notification_queue);
            if (uq > 5) {
                ESP_LOGW(TAG, "URC queue depth %d!", (int)uq);
            }
        }
        ESP_LOGI(TAG, "======================");
    }
}

/* =========================================================================
 * ACCELEROMETER FIFO DRAIN TASK — 200 Hz, dedicated
 * =========================================================================
 * Runs at HIGHER priority than diag_log_task and mqtt_publish_task.
 * The only job of this task is to drain the LIS3DHTR FIFO every 160 ms.
 * At 200 Hz ODR the FIFO fills (32 samples) in exactly 160 ms.
 * Nothing else runs in this task — no UART, no mutex, no logging.
 * Logging of RMS values happens in diag_log_task at 1 Hz from the ring buffer.
 * ========================================================================= */
static void accel_drain_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Accel drain task started on Core %d", xPortGetCoreID());
    TickType_t last_wake = xTaskGetTickCount();
    
     // Register with watchdog 
    esp_task_wdt_add(NULL);

    while (1) {
		esp_task_wdt_reset();
        /* Drain FIFO and push window into ring buffer */
        if (lis3dhtr_fifo_drain_accumulate() != ESP_OK) {
            ESP_LOGW(TAG, "[ACCEL] FIFO drain failed");
        }
        /* vTaskDelayUntil gives precise 160ms period regardless of drain duration.
         * At 200Hz, drain takes ~3-5ms (I2C burst read of 192 bytes at 100kHz).
         * This leaves ~155ms of slack — more than enough. */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(160));
    }
}

/* =========================================================================
 * DIAGNOSTIC LOGGING TASK — 1 Hz
 * =========================================================================
 * Logs all sensor readings once per second.
 * Accelerometer: reads 1s RMS snapshot from ring buffer (non-destructive).
 *   last_n_windows = 6 covers the last ~960ms of 160ms windows ≈ 1 second.
 * GSM/GPS: AT commands are fine here — 1Hz budget is ~1000ms, well above
 *   the longest AT command (~600ms for GPS).
 * Remove or gate before production deployment.
 * ========================================================================= */
static void diag_log_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Diag log task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "======================");
        ESP_LOGI(TAG, "PRINTING LOGS AT 1HZ");
        ESP_LOGI(TAG, "======================");

        /* --- Serial number --- */
        ESP_LOGI(TAG, "ESP32 Serial: %s", deviceSerial);

        /* --- Accelerometer 1s RMS snapshot (last 6 windows ≈ 960ms) ---
         * 6 windows × 32 samples × 200Hz = ~192 samples ≈ 1 second of data.
         * Does NOT reset the ring buffer — accel_drain_task and
         * mqtt_publish_task are unaffected. */
        lis3dhtr_publish_stats_t accel_snap;
        if (lis3dhtr_get_snapshot_rms(&accel_snap, 6) == ESP_OK &&
            accel_snap.windows_captured > 0) {
            ESP_LOGI(TAG, "[ACCEL] 1s RMS: X=%.1f Y=%.1f Z=%.1f Mag=%.1f mg "
                     "(%d windows, %d samples)",
                     accel_snap.x_rms60_mg,
                     accel_snap.y_rms60_mg,
                     accel_snap.z_rms60_mg,
                     accel_snap.magnitude_rms60_mg,
                     accel_snap.windows_captured,
                     accel_snap.windows_captured * 32);
        }

        /* --- ADC sensors --- */
        trike_sensor_data_t sensors;
        float diag_voltage_v = 0.0f, diag_temp_c = 0.0f;
        bool v_ok = (trike_sensors_read_voltage(&sensors, &diag_voltage_v) == ESP_OK);
        bool t_ok = (trike_sensors_read_temperature(&sensors, &diag_temp_c) == ESP_OK);
        float diag_shunt_a     = bms_monitor_get_shunt_snapshot();
        float diag_shunt_max_a = bms_monitor_get_shunt_max();
        if (v_ok && t_ok) {
            ESP_LOGI(TAG, "[ADC] V=%.2fV I=%.3fA(snap) I_max=%.3fA T=%.1f°C",
                     diag_voltage_v, diag_shunt_a, diag_shunt_max_a, diag_temp_c);
        } else {
            ESP_LOGW(TAG, "[ADC] sensor read failed I=%.3fA I_max=%.3fA",
                     diag_shunt_a, diag_shunt_max_a);
        }

        /* --- BMS --- */
        uint8_t ok_count = bms_monitor_get_ok_count();
        ESP_LOGI(TAG, "[BMS] batteries_ok=%d/%d avg_soc=%d%%",
                 ok_count, BMS_BATTERY_COUNT, get_average_battery_soc());

        /* --- GPS --- */
        gps_position_t gps;
        memset(&gps, 0, sizeof(gps));
        ESP_LOGI(TAG, "GPS lock flag %s",gps_fetch_lock? "TRUE":"FALSE");
        if (!gps_fetch_lock && gps_get_latest_position(&gps) == ESP_OK && gps.valid) {
            ESP_LOGI(TAG, "[GPS] lat=%.5f lon=%.5f spd=%.1fkm/h sats=%d",
                     gps.latitude, gps.longitude, gps.speed_kmh, gps.satellites);
            gps_update_speed_cache(gps.speed_kmh, true);
        } else {
			if(!gps_fetch_lock ){
	            ESP_LOGI(TAG, "[GPS] no fix");
	            gps_update_speed_cache(0.0f, false);				
			}

        }

        /* --- GSM signal quality --- */
        if (!provisioning_in_progress) {
            int gsm_rssi = 99, gsm_dbm = -999;
            if (get_gsm_signal_quality(&gsm_rssi, &gsm_dbm)) {
                ESP_LOGI(TAG, "[GSM] rssi=%d signal=%ddBm", gsm_rssi, gsm_dbm);
            } else {
                ESP_LOGI(TAG, "[GSM] no signal (rssi=99)");
            }
        }

        /* --- Trike power state --- */
        ESP_LOGI(TAG, "[PWR] trike_on=%d motion=%d avg_soc=%d%%",
                 trike_ctrl_get_power_confirmation(),
                 check_trike_motion(),
                 get_average_battery_soc());

        /* --- RGB LED --- */
        ESP_LOGI(TAG, "[LED] alert=%s",
                 rgb_led_alert_to_str(rgb_led_get_alert()));
    }
}

/* =========================================================================
 * APP_MAIN
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  KSC TCU V1.1  —  ESP32-S3");
    ESP_LOGI(TAG, "  FW: %s   HW: %s", FW_VERSION_STR, HW_VERSION_STR);
    ESP_LOGI(TAG, "  Built: %s %s", FW_BUILD_DATE, FW_BUILD_TIME);
    ESP_LOGI(TAG, "========================================");

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    //tcu_nvs_erase_creds();		/*Erase NVS provisioning creds for re- provisioning*/

    /* Watchdog — 300 s production timeout */
    esp_task_wdt_config_t wdt = {
        .timeout_ms    = WATCHDOG_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt));
    ESP_LOGI(TAG, "Watchdog: %d s timeout", WATCHDOG_TIMEOUT_S);
    
    //Add main loop to watchdog
    //ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    /* RGB LEDs — must be first so fault LEDs work during init */
    ESP_ERROR_CHECK(rgb_led_init());

    /* Chip serial */
    getChipIdString(deviceSerial, sizeof(deviceSerial));
    
    /* --- Serial number --- */
    ESP_LOGI(TAG, "ESP32 Serial: %s", deviceSerial);

    //Use hub name as the actual TCU number e.g jmbc_0008
    //snprintf(HUB_NAME, sizeof(HUB_NAME), "%s", TCU_DEVICE_8);
    snprintf(HUB_NAME, sizeof(HUB_NAME), "%s", TCU_DEVICE_2);
    //snprintf(HUB_NAME, sizeof(HUB_NAME), "%s", TCU_DEVICE_7);
    //snprintf(HUB_NAME, sizeof(HUB_NAME), "TCU_%s", deviceSerial);

    /* LIS3DHTR accelerometer */
    ret = lis3dhtr_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Accelerometer init failed — motion detection degraded");
        esp_restart();
    }
    
	/* Enable FIFO stream mode for windowed accumulation.
	 * Click/motion interrupts are unaffected (operate on raw ODR pipeline). */
	ret = lis3dhtr_fifo_enable();
	if (ret != ESP_OK) {
	    ESP_LOGW(TAG, "FIFO enable failed — single-sample fallback active");
	}    
    
    vTaskDelay(pdMS_TO_TICKS(200));
    // Explicitly switch to TEST mode so HPF is off and gravity is visible
    lis3dhtr_configure_mode(LIS3DHTR_MODE_TEST);
    
    /* Gravity calibration — must run before accel_drain_task starts.
     * Sensor must be stationary. 200 samples at 5ms = ~1 second. */
    ret = lis3dhtr_calibrate_gravity(1000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Gravity calibration failed — offsets will be zero");
    }    

    /* ADC sensors */
    ESP_ERROR_CHECK(trike_sensors_init());

    /* Log reset reason */
    char reset_reason[32];
    power_mgmt_get_reset_reason(reset_reason, sizeof(reset_reason));
    ESP_LOGI(TAG, "Reset reason: %s", reset_reason);

    /* RS-485 UART */
    rs485_uart_init();
    uart_modem_mutex_init();

    /* Modbus RTU interface */
    struct modbus_rtu_interface_s rtu_interface = {
        .write = rs485_write,
        .read  = rs485_read,
    };

    /* Assign interface to JK-BMS devices */
    jk_device_t *jk_devices = bms_monitor_get_devices();
    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
        jk_devices[i].modbus.interface = rtu_interface;
    }

    /* BMS monitoring task */
    ret = bms_monitor_task_init(&rtu_interface);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMS monitor init failed");
        esp_restart();
    }

    /* Factory / startup self-test */
    ESP_LOGI(TAG, "Running factory self-test...");
    esp_err_t test_result = factory_test_run();
    if (test_result != ESP_OK) {
        ESP_LOGW(TAG, "One or more factory tests failed — continuing anyway");
    } 
    
    // Power and trike control 
    ret = power_trike_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power/trike init failed");
        esp_restart();
    } 
    
	/* Application tasks */
    xTaskCreatePinnedToCore(mqtt_publish_task,  "mqtt_publish",
                             6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(stats_monitor_task, "stats_monitor",
                             3072, NULL, 3, NULL, 1);

    /* Accelerometer drain task — MUST run at higher priority than diag_log
     * and must meet the 160ms deadline. Pinned to Core 0 (same as BMS task)
     * to keep I2C operations off Core 1 (MQTT/modem UART). */
    xTaskCreatePinnedToCore(accel_drain_task, "accel_drain",
                             3072, NULL, 6, NULL, 0);

    /* Diagnostic logging — 1Hz, no timing pressure, all metrics */
    xTaskCreatePinnedToCore(diag_log_task, "diag_log",
                             5120, NULL, 2, NULL, 0);                                                        

//    ESP_LOGI(TAG, "All tasks started");
    ESP_LOGI(TAG, "========================================");

    /* Main loop — watchdog feed for app_main task */
    while (1) {
	    vTaskDelay(pdMS_TO_TICKS(1000));
	    //esp_task_wdt_reset();			//reset the watchdog
    }
}