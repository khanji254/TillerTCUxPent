/**
 * @file bms_monitor_task.c
 * @brief Battery Monitoring Task Implementation for KSC TCU V2.1 (Shunt-Only Mode)
 *
 * In this mode, BMS communication and RS-485 Modbus are disabled.
 * Battery current is sampled directly from the analog shunt sensor.
 */

#include "bms_monitor_task.h"
#include "bms_rs485_mux.h"
#include "jkbms_serial_storage.h"
#include "jk_bms.h"
#include "rgb_led.h"
#include "trike_sensors.h"
#include "board_config.h"
#include "lsm6dsox.h"
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

#define BMS_DEVICE_ADDRESS      0x01

/* =========================================================================
 * MOTION DETECTION
 * =========================================================================
 * Conservative multi-sensor fusion:
 *   1. GNSS Ground Speed:  gps_speed_kmh >= GPS_MOTION_SPEED_THRESHOLD_KMH (1.5 km/h)
 *   2. Shunt Current:      |I_shunt| > MOTION_CURRENT_THRESHOLD_A (2.0 A)
 *   3. 6-Axis IMU:         LSM6DSOX motion interrupt or vibration mag > 250 mg
 *
 * Safety Interlock: If ANY sensor detects motion, the vehicle is IN MOTION.
 * Stationary Debounce: Requires MOTION_STOP_DEBOUNCE_CYCLES consecutive cycles
 *                      of complete stillness across all sensors before declaring
 *                      the vehicle stationary.
 * ========================================================================= */

#define MOTION_CURRENT_THRESHOLD_A     2.0f
#define MOTION_IMU_VIB_THRESHOLD_MG    250.0f
#define MOTION_STOP_DEBOUNCE_CYCLES    3

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

static jk_device_t        jk_devices[BMS_BATTERY_COUNT];
static bms_system_stats_t system_stats            = {0};
static bool               task_running            = false;

static volatile bool      trike_in_motion            = false;
static uint8_t            s_stationary_cycle_count   = MOTION_STOP_DEBOUNCE_CYCLES;
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

        /* Motion detection: Multi-sensor conservative fusion */
        float gps_speed_kmh = 0.0f;
        bool  gps_valid     = gps_get_speed_cache(&gps_speed_kmh);
        bool  gps_motion    = false;

        if (gps_valid) {
            gps_motion = (gps_speed_kmh >= GPS_MOTION_SPEED_THRESHOLD_KMH);
            ESP_LOGD(TAG, "Motion[GPS]: %.2fkm/h -> %s",
                     gps_speed_kmh, gps_motion ? "MOVING" : "STOPPED");
        } else {
            ESP_LOGD(TAG, "Motion[GPS]: stale/invalid — relying on IMU & shunt current");
        }

        /* Read IMU acceleration for motion detection & vibration tracking */
        lsm6dsox_accel_mg_t imu_accel = {0};
        esp_err_t imu_ret = lsm6dsox_read_accel_mg(&imu_accel);
        float imu_mag = 0.0f;
        if (imu_ret == ESP_OK) {
            imu_mag = sqrtf(imu_accel.x_mg * imu_accel.x_mg +
                            imu_accel.y_mg * imu_accel.y_mg +
                            imu_accel.z_mg * imu_accel.z_mg);
        }
        bool imu_motion = lsm6dsox_is_in_motion();
        if (!imu_motion && imu_ret == ESP_OK && lsm6dsox_get_mode() == LSM6DSOX_MODE_PRODUCTION) {
            /* In PRODUCTION mode, HPF removes DC gravity; dynamic vibration > threshold indicates motion */
            if (imu_mag > MOTION_IMU_VIB_THRESHOLD_MG) {
                imu_motion = true;
            }
        }

        /* Shunt current detection: Traction or auxiliary current > threshold */
        bool current_motion = (fabsf(snap) > MOTION_CURRENT_THRESHOLD_A);

        /*
         * Safety-critical arbitration:
         * If ANY sensor modality detects active vehicle movement or load,
         * flag raw motion as true immediately. No sensor masks another.
         */
        bool raw_motion = (gps_valid && gps_motion) || current_motion || imu_motion;

        /*
         * Asymmetric hysteresis / debouncing:
         * - Motion transition to TRUE is instantaneous (1 cycle).
         * - Stillness transition to FALSE requires MOTION_STOP_DEBOUNCE_CYCLES
         *   consecutive cycles of stillness across all modalities before clearing
         *   motion flag. Prevents false-stop triggers during gear shifting,
         *   furrow turnaround, or zero-current coasting.
         */
        if (raw_motion) {
            s_stationary_cycle_count = 0;
            trike_in_motion = true;
        } else {
            if (s_stationary_cycle_count < MOTION_STOP_DEBOUNCE_CYCLES) {
                s_stationary_cycle_count++;
            }
            if (s_stationary_cycle_count >= MOTION_STOP_DEBOUNCE_CYCLES) {
                trike_in_motion = false;
            }
        }

        if (imu_ret == ESP_OK) {
            ESP_LOGI(TAG, "[SHUNT] cycle %" PRIu32 ": I=%.3fA, motion=%s [raw:%s, still_cnt:%d/%d | GPS:%s(%.1fkm/h), IMU:%s(Mag=%.1fmg), I:%s]",
                     s_snapshot_cycle_id, snap,
                     trike_in_motion ? "YES" : "NO",
                     raw_motion ? "YES" : "NO",
                     s_stationary_cycle_count, MOTION_STOP_DEBOUNCE_CYCLES,
                     gps_valid ? (gps_motion ? "YES" : "NO") : "N/A",
                     gps_speed_kmh,
                     imu_motion ? "YES" : "NO", imu_mag,
                     current_motion ? "YES" : "NO");
        } else {
            ESP_LOGI(TAG, "[SHUNT] cycle %" PRIu32 ": I=%.3fA, motion=%s [raw:%s, still_cnt:%d/%d | GPS:%s(%.1fkm/h), IMU:ERR(%s), I:%s]",
                     s_snapshot_cycle_id, snap,
                     trike_in_motion ? "YES" : "NO",
                     raw_motion ? "YES" : "NO",
                     s_stationary_cycle_count, MOTION_STOP_DEBOUNCE_CYCLES,
                     gps_valid ? (gps_motion ? "YES" : "NO") : "N/A",
                     gps_speed_kmh,
                     esp_err_to_name(imu_ret),
                     current_motion ? "YES" : "NO");
        }

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

    /* Create battery monitor task */
    BaseType_t mon_created = xTaskCreatePinnedToCore(
        bms_monitor_task, "bms_monitor",
        BMS_MONITOR_TASK_STACK, NULL,
        BMS_MONITOR_TASK_PRIO, &bms_monitor_task_handle,
        BMS_MONITOR_TASK_CORE
    );
    if (mon_created != pdPASS) {
        vSemaphoreDelete(s_snapshot_mutex);
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