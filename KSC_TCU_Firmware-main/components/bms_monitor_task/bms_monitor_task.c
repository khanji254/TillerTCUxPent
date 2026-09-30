/**
 * @file bms_monitor_task.c
 * @brief Battery Monitoring Task Implementation for KSC TCU V1.1 (Shunt-Only Mode)
 *
 * In this mode, BMS communication and RS-485 Modbus are disabled.
 * Battery current is sampled directly from the analog shunt sensor.
 */

#include "bms_monitor_task.h"
#include "bms_rs485_mux.h"
#include "jkbms_serial_storage.h"
#include "jk_bms.h"
#include "lis3dhtr.h"
#include "rgb_led.h"
#include "trike_sensors.h"
#include "board_config.h"
#include "Quectel_gps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "esp_task_wdt.h"

#define TAG "BMS_MONITOR"

/* =========================================================================
 * TASK CONFIGURATION
 * ========================================================================= */

#define BMS_MONITOR_TASK_STACK  4096
#define BMS_MONITOR_TASK_PRIO   5
#define BMS_MONITOR_TASK_CORE   0

#define BMS_LED_TASK_STACK      2048
#define BMS_LED_TASK_PRIO       4
#define BMS_LED_TASK_CORE       0

#define BMS_DEVICE_ADDRESS      0x01

/* =========================================================================
 * MOTION DETECTION
 * =========================================================================
 * Primary:   GPS speed cache (speed >= threshold).
 * Fallback:  Analog shunt current magnitude > MOTION_CURRENT_THRESHOLD_A.
 * ========================================================================= */

#define MOTION_CURRENT_THRESHOLD_A  2

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

/* ── Atomic cycle snapshot ── */
static bms_queued_data_t  s_cycle_snapshot[BMS_BATTERY_COUNT];
static volatile bool      s_snapshot_ready    = false;
static volatile uint32_t  s_snapshot_cycle_id = 0;
static SemaphoreHandle_t  s_snapshot_mutex    = NULL;

/* ── Shunt ADC snapshot — captured each cycle ── */
static volatile float     s_shunt_current_snapshot_a = 0.0f;
static volatile float     s_shunt_snap[3]            = {0.0f, 0.0f, 0.0f};
static volatile float     s_shunt_snap_avg           = 0.0f;
static volatile float     s_shunt_snap_max           = 0.0f;
static volatile float     s_shunt_snap_rms           = 0.0f;

/* ── Cycle timing diagnostic ── */
static volatile uint32_t  s_last_cycle_ms = 0;

static TaskHandle_t       bms_monitor_task_handle = NULL;
static TaskHandle_t       bms_led_task_handle     = NULL;
static SemaphoreHandle_t  led_status_mutex        = NULL;

static jk_device_t        jk_devices[BMS_BATTERY_COUNT];
static bms_system_stats_t system_stats            = {0};
static bool               task_running            = false;

static volatile bool      trike_in_motion         = false;
static volatile uint8_t   s_batteries_ok_count_final = 0;

/* ── Per-pass timing diagnostics ── */
static volatile uint32_t  s_last_pass1_ms = 0;
static volatile uint32_t  s_last_pass2_ms = 0;

/* =========================================================================
 * PUBLIC API — SNAPSHOT
 * ========================================================================= */

bool bms_monitor_get_snapshot(bms_queued_data_t out[BMS_BATTERY_COUNT],
                               uint32_t         *cycle_id_inout,
                               float            *shunt_a_out,
                               float            *shunt_max_out)
{
    if (!s_snapshot_ready || !out || !cycle_id_inout || !shunt_a_out) {
        if (shunt_a_out) *shunt_a_out = 0.0f;
        if (shunt_max_out) *shunt_max_out = 0.0f;
        return false;
    }
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    bool is_new = (*cycle_id_inout != s_snapshot_cycle_id);
    if (is_new) {
        memcpy(out, s_cycle_snapshot,
               BMS_BATTERY_COUNT * sizeof(bms_queued_data_t));
        *shunt_a_out    = s_shunt_current_snapshot_a;
        *cycle_id_inout = s_snapshot_cycle_id;
        if (shunt_max_out) *shunt_max_out = s_shunt_snap_max;
    } else {
        if (shunt_max_out) *shunt_max_out = 0.0f;
        *shunt_a_out = 0.0f;
    }
    xSemaphoreGive(s_snapshot_mutex);
    return is_new;
}

uint32_t bms_monitor_get_last_cycle_id(void)
{
    return s_snapshot_cycle_id;
}

uint32_t bms_monitor_get_last_cycle_ms(void)
{
    return s_last_cycle_ms;
}

float bms_monitor_get_shunt_snapshot(void)
{
    return s_shunt_current_snapshot_a;
}

/* =========================================================================
 * LED STATUS TASK
 * ========================================================================= */

static void bms_led_status_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED status task started on Core %d (shunt-only mode)", xPortGetCoreID());
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* =========================================================================
 * MAIN SHUNT MONITORING TASK
 * ========================================================================= */

static void bms_monitor_task(void *pvParameters)
{
    bms_queued_data_t staging[BMS_BATTERY_COUNT];

    s_batteries_ok_count_final = 0;

    ESP_LOGI(TAG, "Battery monitor (shunt-only) task started on Core %d", xPortGetCoreID());
    task_running = true;

    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();

        int64_t cycle_start_us = esp_timer_get_time();

        /* Read physical current from analog shunt */
        trike_sensor_data_t shunt_tmp = {0};
        float snap = 0.0f;
        esp_err_t err = trike_sensors_read_current(&shunt_tmp, &snap);
        if (err != ESP_OK) {
            snap = s_shunt_current_snapshot_a;
            ESP_LOGW(TAG, "Shunt current read failed — retaining last value: %.3fA", snap);
        }

        for (int b = 0; b < BMS_BATTERY_COUNT; b++) {
            memset(&staging[b], 0, sizeof(bms_queued_data_t));
            staging[b].battery_id = (bms_battery_id_t)b;
            staging[b].status     = BMS_STATUS_NO_DATA;
            staging[b].batt_i     = (int32_t)(snap * 1000.0f);
            staging[b].timestamp  = (uint32_t)(esp_timer_get_time() / 1000);
        }

        s_batteries_ok_count_final = 0;
        system_stats.total_readings++;

        int64_t elapsed_us = esp_timer_get_time() - cycle_start_us;
        s_last_pass1_ms = (uint32_t)(elapsed_us / 1000);
        s_last_pass2_ms = 0;
        s_last_cycle_ms = s_last_pass1_ms;

        /* Commit snapshot atomically */
        xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
        memcpy(s_cycle_snapshot, staging, sizeof(staging));
        for (int b = 0; b < 3; b++) {
            s_shunt_snap[b] = snap;
        }
        s_shunt_current_snapshot_a = snap;
        s_shunt_snap_avg           = snap;
        s_shunt_snap_max           = snap;
        s_shunt_snap_rms           = fabsf(snap);
        s_snapshot_cycle_id++;
        s_snapshot_ready = true;
        xSemaphoreGive(s_snapshot_mutex);

        /* Motion detection: GPS speed cache or shunt current threshold */
        float gps_speed_kmh = 0.0f;
        bool  gps_valid     = gps_get_speed_cache(&gps_speed_kmh);
        bool  gps_motion    = false;

        if (gps_valid) {
            gps_motion = (gps_speed_kmh >= GPS_MOTION_SPEED_THRESHOLD_KMH);
            ESP_LOGD(TAG, "Motion[GPS]: %.2fkm/h -> %s",
                     gps_speed_kmh, gps_motion ? "MOVING" : "STOPPED");
        } else {
            ESP_LOGD(TAG, "Motion[GPS]: stale/invalid — using shunt current");
        }

        bool current_motion = (fabsf(snap) > MOTION_CURRENT_THRESHOLD_A);
        trike_in_motion = gps_valid ? gps_motion : current_motion;

        ESP_LOGI(TAG, "[SHUNT] cycle %" PRIu32 ": I=%.3fA, motion=%s",
                 s_snapshot_cycle_id, snap, trike_in_motion ? "YES" : "NO");

        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - cycle_start_us) / 1000);
        if (elapsed_ms < BMS_READ_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(BMS_READ_INTERVAL_MS - elapsed_ms));
        }
    }
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

jk_device_t *bms_monitor_get_devices(void)
{
    return jk_devices;
}

uint8_t bms_monitor_get_ok_count(void)
{
    return s_batteries_ok_count_final;
}

void bms_monitor_get_stats(bms_system_stats_t *stats)
{
    if (stats) {
        memcpy(stats, &system_stats, sizeof(bms_system_stats_t));
    }
}

void bms_monitor_reset_stats(void)
{
    memset(&system_stats, 0, sizeof(bms_system_stats_t));
}

bool bms_monitor_task_is_running(void)
{
    return task_running;
}

bool check_trike_motion(void)
{
    return trike_in_motion;
}

uint32_t bms_monitor_task_get_stack_hwm(void)
{
    return (bms_monitor_task_handle != NULL) ?
           uxTaskGetStackHighWaterMark(bms_monitor_task_handle) : 0;
}

bool bms_monitor_set_discharge_all(uint8_t state)
{
    ESP_LOGI(TAG, "bms_monitor_set_discharge_all(%d): bypassed in shunt-only mode", state);
    return true;
}

esp_err_t bms_monitor_task_init(struct modbus_rtu_interface_s *interface)
{
    s_snapshot_mutex = xSemaphoreCreateMutex();
    if (!s_snapshot_mutex) {
        ESP_LOGE(TAG, "Failed to create snapshot mutex");
        return ESP_FAIL;
    }

    if (interface && interface->read && interface->write) {
        for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
            jk_devices[i].modbus.device_addr = BMS_DEVICE_ADDRESS;
            jk_devices[i].modbus.interface   = *interface;
        }
    }

    /* Create LED status mutex and task */
    led_status_mutex = xSemaphoreCreateMutex();
    if (!led_status_mutex) {
        vSemaphoreDelete(s_snapshot_mutex);
        return ESP_FAIL;
    }

    BaseType_t led_created = xTaskCreatePinnedToCore(
        bms_led_status_task, "bms_led",
        BMS_LED_TASK_STACK, NULL,
        BMS_LED_TASK_PRIO, &bms_led_task_handle,
        BMS_LED_TASK_CORE
    );
    if (led_created != pdPASS) {
        vSemaphoreDelete(s_snapshot_mutex);
        vSemaphoreDelete(led_status_mutex);
        return ESP_FAIL;
    }

    /* Create battery monitor task */
    BaseType_t mon_created = xTaskCreatePinnedToCore(
        bms_monitor_task, "bms_monitor",
        BMS_MONITOR_TASK_STACK, NULL,
        BMS_MONITOR_TASK_PRIO, &bms_monitor_task_handle,
        BMS_MONITOR_TASK_CORE
    );
    if (mon_created != pdPASS) {
        vSemaphoreDelete(s_snapshot_mutex);
        vSemaphoreDelete(led_status_mutex);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Battery monitor (shunt-only) initialised on Core %d",
             BMS_MONITOR_TASK_CORE);
    return ESP_OK;
}

uint32_t bms_monitor_get_last_pass1_ms(void) { return s_last_pass1_ms; }
uint32_t bms_monitor_get_last_pass2_ms(void) { return s_last_pass2_ms; }

float bms_monitor_get_shunt_avg(void) { return s_shunt_snap_avg; }
float bms_monitor_get_shunt_max(void) { return s_shunt_snap_max; }
float bms_monitor_get_shunt_rms(void) { return s_shunt_snap_rms; }