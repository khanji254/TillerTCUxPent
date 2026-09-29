/**
 * @file bms_monitor_task.c
 * @brief JK-BMS Battery Monitoring Task Implementation for KSC TCU V1.1
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
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
 * SERIAL READ CONFIGURATION
 * ========================================================================= */

#define SERIAL_READ_MAX_RETRIES  5
#define SERIAL_READ_DELAY_MS     50

/* =========================================================================
 * MOTION DETECTION
 * =========================================================================
 * Primary:   LIS3DHTR hardware interrupt (current draw as fallback).
 * Threshold: >5 A on any battery indicates trike motion.
 * ========================================================================= */

#define MOTION_CURRENT_THRESHOLD_A  2

/* =========================================================================
 * MODULE STATE
 * =========================================================================
 * NOTE: bms_data_queue has been removed entirely.
 * Battery data is now exposed via the atomic snapshot API below.
 * ========================================================================= */
 
/* ── Atomic cycle snapshot (replaces FreeRTOS queue) ── */
static bms_queued_data_t  s_cycle_snapshot[BMS_BATTERY_COUNT];
static volatile bool      s_snapshot_ready    = false;
static volatile uint32_t  s_snapshot_cycle_id = 0;
static SemaphoreHandle_t  s_snapshot_mutex    = NULL;
 
/* ── Shunt ADC snapshot — captured at end of each BMS cycle ── */
static volatile float     s_shunt_current_snapshot_a = 0.0f;
static volatile float s_shunt_snap[3]   = {0.0f, 0.0f, 0.0f};
static volatile float s_shunt_snap_avg  = 0.0f;
static volatile float s_shunt_snap_max  = 0.0f;
static volatile float s_shunt_snap_rms  = 0.0f;
 
/* ── Cycle timing diagnostic ── */
static volatile uint32_t  s_last_cycle_ms = 0;

static TaskHandle_t   bms_monitor_task_handle = NULL;
static TaskHandle_t   bms_led_task_handle     = NULL;
static SemaphoreHandle_t led_status_mutex     = NULL;

static jk_device_t    jk_devices[BMS_BATTERY_COUNT];
static jk_data_t      jk_data_temp;
static bms_system_stats_t system_stats        = {0};
static bool           task_running            = false;

static volatile uint8_t  batteries_read_ok    = 0;
static volatile bool     led_status_updated   = false;
static volatile bool     trike_in_motion      = false;
static volatile uint8_t  s_batteries_ok_count, s_batteries_ok_count_final = 0;

/** Cached serial numbers for fallback when a battery is unreadable */
static char cached_serials[BMS_BATTERY_COUNT][16] = {{0}};

/* ── Per-pass timing diagnostics ── */
static volatile uint32_t  s_last_pass1_ms = 0;   /* serial + current, all 3 batteries */
static volatile uint32_t  s_last_pass2_ms = 0;   /* remaining registers, all 3 batteries */

/* =========================================================================
 * SERIAL VALIDATION STATE
 * =========================================================================
 * Prevents noisy/truncated Modbus frame reads from corrupting NVS.
 *
 * A new serial is only accepted (and considered for NVS write) after it has
 * been read identically for SERIAL_STABLE_THRESHOLD consecutive pass1 reads.
 * A read shorter than the current cached/NVS serial is rejected immediately
 * regardless of content — truncated frames are always shorter than valid ones.
 * ========================================================================= */

#define SERIAL_STABLE_THRESHOLD   3   /* consecutive identical reads required */
#define SERIAL_MIN_VALID_LEN      6   /* reject anything shorter than this    */

typedef struct {
    char     candidate[16];   /* serial currently being accumulated          */
    uint8_t  stable_count;    /* how many consecutive times we saw candidate */
    char     accepted[16];    /* last serial accepted after stability check  */
    bool     has_accepted;    /* true once at least one serial was accepted  */
} serial_validator_t;

static serial_validator_t s_serial_validators[BMS_BATTERY_COUNT];

/* =========================================================================
 * PUBLIC API — SNAPSHOT
 * ========================================================================= */
 
bool bms_monitor_get_snapshot(bms_queued_data_t out[BMS_BATTERY_COUNT],
                               uint32_t         *cycle_id_inout,
                               float            *shunt_a_out, float            *shunt_max_out)
{
    if (!s_snapshot_ready || !out || !cycle_id_inout || !shunt_a_out) {
        if (shunt_a_out) *shunt_a_out = 0.0f;
        return false;
    }
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    bool is_new = (*cycle_id_inout != s_snapshot_cycle_id);
    if (is_new) {
        memcpy(out, s_cycle_snapshot,
               BMS_BATTERY_COUNT * sizeof(bms_queued_data_t));
        *shunt_a_out    = s_shunt_current_snapshot_a; /* same mutex take */
        *cycle_id_inout = s_snapshot_cycle_id;
        *shunt_max_out	= s_shunt_snap_max;
    } else {
		*shunt_max_out	=  0.0f;
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

/**
 * @brief LED status task — mirrors BMS read health via the RGB LED system.
 *
 * Delegates actual LED control to rgb_led_set_alert() rather than driving
 * GPIO directly; this keeps LED logic in one place.
 *
 * @param[in] pvParameters  Unused.
 */
 /* In bms_led_status_task — add a consecutive-failure counter */
static void bms_led_status_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED status task started on Core %d", xPortGetCoreID());
    
    
    /* Wait for the first complete BMS read cycle before asserting any alert.
     * Without this, s_batteries_ok_count_final is 0 at boot even when
     * batteries are present, causing a false NO_BATTERIES alert. */
    vTaskDelay(pdMS_TO_TICKS(BMS_READ_INTERVAL_MS + 5000));    
    
    uint8_t no_battery_ticks = 0;   /* consecutive ticks with ok_count < 3 */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));

        uint8_t ok_count = s_batteries_ok_count_final;

        if (ok_count < BMS_BATTERY_COUNT) {
            no_battery_ticks++;
            /* Only assert NO_BATTERIES after 20 consecutive ticks = 1 second
             * This filters momentary read failures from racing with the
             * MQTT publish task's SOC/trike-state LED updates */
            if (no_battery_ticks >= 20) {
                rgb_led_set_alert(ALERT_NO_BATTERIES, false);
            }
        } else {
            no_battery_ticks = 0;
            /* Don't set NORMAL here — let the publish task own that state.
             * The BMS task only asserts fault; publish task clears it. */
        }
    }
}

/**
 * @brief Validate a freshly-read serial number against the stability window.
 *
 * Rules:
 *   1. Empty or shorter-than-minimum strings are rejected immediately.
 *   2. A string shorter than the currently accepted serial is rejected
 *      (truncated frame — noise always shortens, never lengthens).
 *   3. A string matching the current candidate increments stable_count.
 *      When stable_count reaches SERIAL_STABLE_THRESHOLD the serial is
 *      promoted to accepted[].
 *   4. A string that differs from the candidate resets the window.
 *
 * @param[in]  batt_id   Battery slot index.
 * @param[in]  raw       Freshly-read serial string from Modbus.
 * @param[out] out       Buffer to receive the validated serial (16 bytes).
 *                       Always populated — falls back to accepted or cached.
 * @return true if a stable, validated serial is available in out[].
 */
static bool bms_serial_validate(bms_battery_id_t batt_id,
                                 const char *raw,
                                 char out[16])
{
    serial_validator_t *v = &s_serial_validators[batt_id];
    size_t raw_len = strlen(raw);

    /* Rule 1: too short to be a real serial */
    if (raw_len < SERIAL_MIN_VALID_LEN) {
        ESP_LOGW(TAG, "B%d serial too short (%d chars) — rejected",
                 batt_id + 1, (int)raw_len);
        goto use_fallback;
    }

    /* Rule 2: shorter than the accepted serial → truncated frame */
    if (v->has_accepted && raw_len < strlen(v->accepted)) {
        ESP_LOGW(TAG, "B%d serial shorter than accepted (%d < %d) — rejected",
                 batt_id + 1, (int)raw_len, (int)strlen(v->accepted));
        goto use_fallback;
    }

    /* Rule 3/4: accumulate stability window */
    if (strcmp(raw, v->candidate) == 0) {
        v->stable_count++;
    } else {
        /* New candidate — reset window */
        strncpy(v->candidate, raw, 15);
        v->candidate[15] = '\0';
        v->stable_count  = 1;
    }

    if (v->stable_count >= SERIAL_STABLE_THRESHOLD) {
        /* Promote to accepted */
        if (!v->has_accepted ||
            strcmp(v->candidate, v->accepted) != 0) {
            strncpy(v->accepted, v->candidate, 15);
            v->accepted[15]  = '\0';
            v->has_accepted  = true;
            ESP_LOGI(TAG, "B%d serial stabilised: %s",
                     batt_id + 1, v->accepted);
        }
    }

    if (v->has_accepted) {
        strncpy(out, v->accepted, 15);
        out[15] = '\0';
        return true;
    }

use_fallback:
    /* No stable serial yet — use NVS cache */
    if (bms_serial_storage_get(batt_id, out) != ESP_OK) {
        snprintf(out, 16, "BMS%d_UNKNOWN", batt_id + 1);
    }
    return false;
}

/* =========================================================================
 * PASS 1: Read serial number and current only (time-sensitive)
 *
 * Called for all batteries first. Returns false if serial read fails
 * (battery unreachable), in which case pass 2 is also skipped.
 *
 * On success, queued_data->batt_i is populated and status remains
 * BMS_STATUS_NO_DATA until pass 2 completes successfully.
 * ========================================================================= */
static bool read_battery_pass1(bms_battery_id_t battery_id,
                                bms_queued_data_t *queued_data)
{
    memset(queued_data, 0, sizeof(bms_queued_data_t));
    queued_data->battery_id = battery_id;
    queued_data->timestamp  = (uint32_t)(esp_timer_get_time() / 1000);
    queued_data->status     = BMS_STATUS_NO_DATA;

    if (bms_rs485_switch_to_battery(battery_id) != ESP_OK) {
        ESP_LOGE(TAG, "P1 MUX switch failed for battery %d", battery_id + 1);
        system_stats.battery_stats[battery_id].failed_reads++;
        bms_serial_storage_get(battery_id, queued_data->serial_no);
        return false;
    }

	uart_flush_input(BOARD_RS485_UART_NUM);   /* flush any stale bytes from pass1 */
	vTaskDelay(pdMS_TO_TICKS(100));           /* longer settle for field cables */
	uart_flush_input(BOARD_RS485_UART_NUM);   /* flush again after settle */
	
    /* Serial number — up to SERIAL_READ_MAX_RETRIES attempts */
    int8_t ret = -1;
    bool serial_ok = false;
    for (uint8_t attempt = 0;
         attempt < SERIAL_READ_MAX_RETRIES && !serial_ok;
         attempt++) {
		uart_flush_input(BOARD_RS485_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(SERIAL_READ_DELAY_MS));
        ret = jk_bms_read_serial(&jk_devices[battery_id],
                                  queued_data->serial_no);
        if (ret == MODBUS_RTU_OK) serial_ok = true;
    }      

    /* ----------------------------------------------------------------
     * Serial validation — replaces the old direct NVS write.
     *
     * queued_data->serial_no holds the raw Modbus read at this point.
     * bms_serial_validate() runs it through the stability window and
     * length guard, returns the validated (or fallback) serial in
     * validated_serial[], and handles NVS writes internally only when
     * the serial has been stable for SERIAL_STABLE_THRESHOLD reads.
     * ---------------------------------------------------------------- */
    char validated_serial[16];
    bool serial_stable = bms_serial_validate(battery_id,
                                              queued_data->serial_no,
                                              validated_serial);

    /* Always use the validated (or fallback) serial going forward */
    strncpy(queued_data->serial_no, validated_serial, 15);
    queued_data->serial_no[15] = '\0';
    
    /*Terminate pass1 immediately if serial read fails after all retries*/
	if(!serial_ok) {
	    bms_rs485_disable_battery(battery_id);
	    vTaskDelay(pdMS_TO_TICKS(10));
	    return false;   /* serial failed */	
	}     

    /* NVS write — only when newly stable and not shorter than stored */
    if (serial_stable) {
        char nvs_serial[16] = {0};
        bool nvs_ok      = (bms_serial_storage_get(battery_id,
                                                    nvs_serial) == ESP_OK);
        bool nvs_differs = !nvs_ok ||
                           (strcmp(nvs_serial, validated_serial) != 0);
        bool nvs_longer  = nvs_ok &&
                           (strlen(nvs_serial) > strlen(validated_serial));

        if (nvs_differs && !nvs_longer) {
            bms_serial_storage_save(battery_id, validated_serial, true);
            ESP_LOGI(TAG, "B%d NVS serial updated: %s → %s",
                     battery_id + 1,
                     nvs_ok ? nvs_serial : "(none)",
                     validated_serial);
        }
    }

    strncpy(cached_serials[battery_id], validated_serial, 15);
    cached_serials[battery_id][15] = '\0';
    /* ----------------------------------------------------------------
     * End of serial validation block
     * ---------------------------------------------------------------- */

    /* Current — single attempt with one retry */
    uint8_t att = 0;   
    do {
		uart_flush_input(BOARD_RS485_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = jk_bms_read_batt_current(&jk_devices[battery_id],
                                        &queued_data->batt_i);
        if(ret != MODBUS_RTU_OK){
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    } while (ret != MODBUS_RTU_OK && att++ < 1);    

    if (ret != MODBUS_RTU_OK) {
        ESP_LOGE(TAG, "P1 current read failed battery %d", battery_id + 1);
        /* Non-fatal for pass 1 — batt_i stays 0, pass 2 will retry */
    }

    ESP_LOGI(TAG, "[P1] Battery %d: I=%" PRId32 "mA", 
             battery_id + 1, queued_data->batt_i);

    bms_rs485_disable_battery(battery_id);
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;   /* serial succeeded — proceed to pass 2 */
}

/* =========================================================================
 * PASS 2: Read all remaining registers (non-time-sensitive)
 *
 * Only called for batteries where pass 1 succeeded.
 * Picks up from serial already stored in queued_data.
 * ========================================================================= */
static bool read_battery_pass2(bms_battery_id_t battery_id,
                                bms_queued_data_t *queued_data)
{
    if (bms_rs485_switch_to_battery(battery_id) != ESP_OK) {
        ESP_LOGE(TAG, "P2 MUX switch failed for battery %d", battery_id + 1);
        return false;
    }

	uart_flush_input(BOARD_RS485_UART_NUM);   /* flush any stale bytes from pass1 */
	vTaskDelay(pdMS_TO_TICKS(100));           /* longer settle for field cables */
	uart_flush_input(BOARD_RS485_UART_NUM);   /* flush again after settle */

    memset(&jk_data_temp, 0, sizeof(jk_data_temp));
    int8_t ret = -1;

	// NEW (3 retries, UART flush on each retry):
	#define READ_REG(fn, dst, label)                                   \
	    do {                                                           \
	        uint8_t _att = 0;                                         \
	        do {                                                       \
	            if (_att > 0) {                                        \
	                uart_flush_input(BOARD_RS485_UART_NUM);           \
	                vTaskDelay(pdMS_TO_TICKS(30));                    \
	            }                                                      \
	            vTaskDelay(pdMS_TO_TICKS(50));                        \
	            ret = (fn);                                            \
	        } while (ret != MODBUS_RTU_OK && _att++ < 3);             \
	        if (ret != MODBUS_RTU_OK) {                               \
	            ESP_LOGE(TAG, "P2 failed to read " label " (err %d)", ret); \
	        }                                                          \
	    } while (0)    

    READ_REG(jk_bms_read_device_id(&jk_devices[battery_id],
                                    jk_data_temp.model_no),
             jk_data_temp.model_no, "Device ID");

    READ_REG(jk_bms_read_device_address(&jk_devices[battery_id],
                                         &jk_data_temp.device_address),
             jk_data_temp.device_address, "Device Address");

    READ_REG(jk_bms_read_cells_present(&jk_devices[battery_id],
                                        &jk_data_temp.cells_present),
             jk_data_temp.cells_present, "Cells Present");

    READ_REG(jk_bms_read_cell_count(&jk_devices[battery_id],
                                     &jk_data_temp.cell_count),
             jk_data_temp.cell_count, "Cell Count");

    READ_REG(jk_bms_read_cells_diff(&jk_devices[battery_id],
                                     &jk_data_temp.cells_diff),
             jk_data_temp.cells_diff, "Cells Diff");

    READ_REG(jk_bms_read_soc(&jk_devices[battery_id],
                               &jk_data_temp.soc),
             jk_data_temp.soc, "SOC");

    READ_REG(jk_bms_read_soh(&jk_devices[battery_id],
                               &jk_data_temp.soh),
             jk_data_temp.soh, "SOH");

    READ_REG(jk_bms_read_batt_voltage(&jk_devices[battery_id],
                                       &jk_data_temp.batt_v),
             jk_data_temp.batt_v, "Voltage");

    /* Current re-read in pass 2 — overwrites the pass 1 value.
     * Pass 1 current is stored in current_ma[] for motion detection
     * and offset calibration; the pass 2 value goes into the snapshot
     * for publishing. Both are useful but pass 1 is more time-aligned. */
    READ_REG(jk_bms_read_batt_current(&jk_devices[battery_id],
                                       &jk_data_temp.batt_i),
             jk_data_temp.batt_i, "Current");

    READ_REG(jk_bms_read_batt_power(&jk_devices[battery_id],
                                     &jk_data_temp.batt_power),
             jk_data_temp.batt_power, "Power");

    READ_REG(jk_bms_read_batt_remaining_capacity(
                 &jk_devices[battery_id],
                 &jk_data_temp.remaining_capacity),
             jk_data_temp.remaining_capacity, "Remaining Cap");

    READ_REG(jk_bms_read_charge_discharge_stat(
                 &jk_devices[battery_id],
                 &jk_data_temp.charge_stat,
                 &jk_data_temp.discharge_stat),
             jk_data_temp.charge_stat, "Charge/Discharge Stat");

    READ_REG(jk_bms_read_charge_cycles(&jk_devices[battery_id],
                                        &jk_data_temp.charge_cycles),
             jk_data_temp.charge_cycles, "Charge Cycles");

    READ_REG(jk_bms_read_alarms(&jk_devices[battery_id],
                                 &jk_data_temp.alarms),
             jk_data_temp.alarms, "Alarms");

#undef READ_REG

    /* Populate queued_data from pass 2 results.
     * serial_no already set in pass 1 — copy back from jk_data_temp
     * only if it differs (battery swap detection). */
    queued_data->device_address     = jk_data_temp.device_address;
    queued_data->batt_v             = jk_data_temp.batt_v;
	/* batt_i intentionally NOT overwritten here.
     * Pass 1 current is retained in the snapshot — it was captured
     * within the tight ~810ms window alongside all other batteries
     * and the shunt ADC. Pass 2 re-reads current only for the Modbus
     * register sequence completeness; that value is discarded. */
     
    /* queued_data->batt_i = jk_data_temp.batt_i; */
    queued_data->batt_power         = jk_data_temp.batt_power;
    queued_data->remaining_capacity = jk_data_temp.remaining_capacity;
    queued_data->cell_count         = jk_data_temp.cell_count;
    queued_data->cells_present      = jk_data_temp.cells_present;
    queued_data->cells_diff         = jk_data_temp.cells_diff;
    queued_data->soc                = jk_data_temp.soc;
    queued_data->soh                = jk_data_temp.soh;
    queued_data->charge_stat        = jk_data_temp.charge_stat;
    queued_data->discharge_stat     = jk_data_temp.discharge_stat;
    queued_data->charge_cycles      = jk_data_temp.charge_cycles;
    queued_data->alarms             = jk_data_temp.alarms;
    memcpy(queued_data->model_no, jk_data_temp.model_no,
           sizeof(queued_data->model_no));

    /* Serial already validated and written by pass 1 — preserve only.
     * The old serial change detection block is removed entirely.
     * Pass 1 owns all serial validation and NVS writes via bms_serial_validate(). */
    strncpy(cached_serials[battery_id], queued_data->serial_no, 15);
    cached_serials[battery_id][15] = '\0';

    queued_data->status = BMS_STATUS_OK;
    system_stats.battery_stats[battery_id].successful_reads++;
    system_stats.battery_stats[battery_id].last_success_timestamp =
        queued_data->timestamp;

    ESP_LOGI(TAG, "[P2] Battery %d: V=%" PRIu32 "mV I=%" PRId32 "mA SOC=%u%%",
             battery_id + 1, queued_data->batt_v,
             queued_data->batt_i, queued_data->soc);

    bms_rs485_disable_battery(battery_id);
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

/* =========================================================================
 * SINGLE BATTERY READ
 * ========================================================================= */

/**
 * @brief Read all registers from one JK-BMS battery.
 *
 * Serial number is read first (up to SERIAL_READ_MAX_RETRIES attempts).
 * If the serial read fails all remaining registers are skipped and the
 * cached serial is used so the queue entry carries identification data.
 *
 * @param[in]  battery_id   Which battery to read.
 * @param[out] queued_data  Output structure to populate.
 * @return true on complete success, false if serial read failed.
 */
static bool read_single_battery(bms_battery_id_t battery_id,
                                 bms_queued_data_t *queued_data)
{
    memset(queued_data, 0, sizeof(bms_queued_data_t));
    queued_data->battery_id = battery_id;
    queued_data->timestamp  = (uint32_t)(esp_timer_get_time() / 1000);
    queued_data->status     = BMS_STATUS_NO_DATA;

    if (bms_rs485_switch_to_battery(battery_id) != ESP_OK) {
        ESP_LOGE(TAG, "MUX switch failed for battery %d", battery_id + 1);
        system_stats.battery_stats[battery_id].failed_reads++;
        bms_serial_storage_get(battery_id, queued_data->serial_no);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); 			//settling time for proper bus performance

    memset(&jk_data_temp, 0, sizeof(jk_data_temp));

    //ESP_LOGI(TAG, "--- Reading Battery %d ---", battery_id + 1);

    /* ----------------------------------------------------------------
     * Serial number — up to SERIAL_READ_MAX_RETRIES attempts
     * ---------------------------------------------------------------- */
    int8_t ret = -1;
    bool serial_ok = false;

    for (uint8_t attempt = 0;
         attempt < SERIAL_READ_MAX_RETRIES && !serial_ok;
         attempt++) {
        vTaskDelay(pdMS_TO_TICKS(SERIAL_READ_DELAY_MS));
        ret = jk_bms_read_serial(&jk_devices[battery_id],
                                  jk_data_temp.serial_no);
        if (ret == MODBUS_RTU_OK) {
            serial_ok = true;
        }
    }

    if (!serial_ok) {
        //ESP_LOGE(TAG, "Serial read failed after %d attempts",
                 //SERIAL_READ_MAX_RETRIES);
        if (bms_serial_storage_get(battery_id,
                                    queued_data->serial_no) != ESP_OK) {
            snprintf(queued_data->serial_no, sizeof(queued_data->serial_no),
                     "BMS%d_UNKNOWN", battery_id + 1);
        }
        system_stats.battery_stats[battery_id].failed_reads++;
        return false;
    }

    /* ----------------------------------------------------------------
     * Helper macro: read a register with one retry on failure
     * ---------------------------------------------------------------- */
#define READ_REG(fn, dst, label)                                   \
    do {                                                           \
        uint8_t _att = 0;                                         \
        do {                                                       \
            vTaskDelay(pdMS_TO_TICKS(50));                        \
            ret = (fn);                                            \
        } while (ret != MODBUS_RTU_OK && _att++ < 1);             \
        if (ret != MODBUS_RTU_OK) {                               \
            ESP_LOGE(TAG, "Failed to read " label " (err %d)", ret); \
        }                                                          \
    } while (0)

    READ_REG(jk_bms_read_device_id(&jk_devices[battery_id],
                                    jk_data_temp.model_no),
             jk_data_temp.model_no, "Device ID");

    READ_REG(jk_bms_read_device_address(&jk_devices[battery_id],
                                         &jk_data_temp.device_address),
             jk_data_temp.device_address, "Device Address");

    READ_REG(jk_bms_read_cells_present(&jk_devices[battery_id],
                                        &jk_data_temp.cells_present),
             jk_data_temp.cells_present, "Cells Present");

    READ_REG(jk_bms_read_cell_count(&jk_devices[battery_id],
                                     &jk_data_temp.cell_count),
             jk_data_temp.cell_count, "Cell Count");

    READ_REG(jk_bms_read_cells_diff(&jk_devices[battery_id],
                                     &jk_data_temp.cells_diff),
             jk_data_temp.cells_diff, "Cells Diff");

    READ_REG(jk_bms_read_soc(&jk_devices[battery_id],
                               &jk_data_temp.soc),
             jk_data_temp.soc, "SOC");

    READ_REG(jk_bms_read_soh(&jk_devices[battery_id],
                               &jk_data_temp.soh),
             jk_data_temp.soh, "SOH");

    READ_REG(jk_bms_read_batt_voltage(&jk_devices[battery_id],
                                       &jk_data_temp.batt_v),
             jk_data_temp.batt_v, "Voltage");

    READ_REG(jk_bms_read_batt_current(&jk_devices[battery_id],
                                       &jk_data_temp.batt_i),
             jk_data_temp.batt_i, "Current");

    READ_REG(jk_bms_read_batt_power(&jk_devices[battery_id],
                                     &jk_data_temp.batt_power),
             jk_data_temp.batt_power, "Power");

    READ_REG(jk_bms_read_batt_remaining_capacity(
                 &jk_devices[battery_id],
                 &jk_data_temp.remaining_capacity),
             jk_data_temp.remaining_capacity, "Remaining Cap");

    READ_REG(jk_bms_read_charge_discharge_stat(
                 &jk_devices[battery_id],
                 &jk_data_temp.charge_stat,
                 &jk_data_temp.discharge_stat),
             jk_data_temp.charge_stat, "Charge/Discharge Stat");

    READ_REG(jk_bms_read_charge_cycles(&jk_devices[battery_id],
                                        &jk_data_temp.charge_cycles),
             jk_data_temp.charge_cycles, "Charge Cycles");

    READ_REG(jk_bms_read_alarms(&jk_devices[battery_id],
                                 &jk_data_temp.alarms),
             jk_data_temp.alarms, "Alarms");

#undef READ_REG

    /* ----------------------------------------------------------------
     * Copy data to queue structure
     * ---------------------------------------------------------------- */
    queued_data->device_address     = jk_data_temp.device_address;
    queued_data->batt_v             = jk_data_temp.batt_v;
    queued_data->batt_i             = jk_data_temp.batt_i;
    queued_data->batt_power         = jk_data_temp.batt_power;
    queued_data->remaining_capacity = jk_data_temp.remaining_capacity;
    queued_data->cell_count         = jk_data_temp.cell_count;
    queued_data->cells_present      = jk_data_temp.cells_present;
    queued_data->cells_diff         = jk_data_temp.cells_diff;
    queued_data->soc                = jk_data_temp.soc;
    queued_data->soh                = jk_data_temp.soh;
    queued_data->charge_stat        = jk_data_temp.charge_stat;
    queued_data->discharge_stat     = jk_data_temp.discharge_stat;
    queued_data->charge_cycles      = jk_data_temp.charge_cycles;
    queued_data->alarms             = jk_data_temp.alarms;
    memcpy(queued_data->model_no, jk_data_temp.model_no,
           sizeof(queued_data->model_no));
    memcpy(queued_data->serial_no, jk_data_temp.serial_no,
           sizeof(queued_data->serial_no));

    /* Serial change detection (battery swap) */
    char old_serial[16];
    bool force_nvs = false;
    if (bms_serial_storage_get(battery_id, old_serial) == ESP_OK) {
        if (strcmp(old_serial, jk_data_temp.serial_no) != 0) {
            force_nvs = true;
            ESP_LOGW(TAG, "Battery %d serial changed: %s → %s",
                     battery_id + 1, old_serial, jk_data_temp.serial_no);
        }
    }
    bms_serial_storage_save(battery_id, jk_data_temp.serial_no, force_nvs);
    strncpy(cached_serials[battery_id], jk_data_temp.serial_no, 15);
    cached_serials[battery_id][15] = '\0';

    queued_data->status = BMS_STATUS_OK;
    system_stats.battery_stats[battery_id].successful_reads++;
    system_stats.battery_stats[battery_id].last_success_timestamp =
        queued_data->timestamp;

    ESP_LOGI(TAG, "Battery %d: V=%" PRIu32 "mV I=%" PRId32 "mA SOC=%u%%",
             battery_id + 1, queued_data->batt_v,
             queued_data->batt_i, queued_data->soc);

    return true;
}

/* =========================================================================
 * MAIN BMS MONITORING TASK
 * ========================================================================= */

/**
 * @brief Cyclic battery read task running on Core 0.
 *
 * Each cycle reads all BMS_BATTERY_COUNT batteries sequentially and
 * enqueues the results for the MQTT publish task.  If the queue is full
 * the oldest entry is discarded (overwrite semantics).
 *
 * @param[in] pvParameters  Unused.
 */
static void bms_monitor_task(void *pvParameters)
{
    /*
     * CHANGED: bms_data_queue removed.
     * Battery readings are staged locally and committed to s_cycle_snapshot
     * only when all BMS_BATTERY_COUNT batteries are read in the same cycle.
     * This guarantees the publish task never mixes readings across cycles.
     */
    bms_queued_data_t staging[BMS_BATTERY_COUNT];
 
    s_batteries_ok_count = s_batteries_ok_count_final = 0;
 
    ESP_LOGI(TAG, "BMS monitor task started on Core %d", xPortGetCoreID());
    task_running = true;
 
    int32_t current_ma[BMS_BATTERY_COUNT] = {0};
    
    // Register with watchdog 
    esp_task_wdt_add(NULL);
 
    while (1) {
		
		esp_task_wdt_reset();
		
        /* ── Cycle timing: start ── */
        int64_t cycle_start_us = esp_timer_get_time();
 
 /*       s_batteries_ok_count = 0;
 
        for (bms_battery_id_t batt_id = BMS_BATTERY_1;
             batt_id < BMS_BATTERY_COUNT;
             batt_id++) {
 
            
             * CHANGED: write into staging[], NOT into a queue.
             * The staging buffer is local to this task — no locking needed
             * during the read loop. It is only copied to s_cycle_snapshot
             * under mutex after all three reads complete successfully.
             
            bool ok = read_single_battery(batt_id, &staging[batt_id]);
 
            if (ok) {
                s_batteries_ok_count++;
            } else {
                system_stats.battery_stats[batt_id].last_failure_timestamp =
                    staging[batt_id].timestamp;
                ESP_LOGE(TAG, "Failed to read battery %d", batt_id + 1);
            }
 
            current_ma[batt_id] = staging[batt_id].batt_i;
 
            bms_rs485_disable_battery(batt_id);
            vTaskDelay(pdMS_TO_TICKS(10));
        }*/
        
        
        
		s_batteries_ok_count = 0;

        /* ── PASS 1: Serial + current for all batteries ──────────────────
         * All three current readings are captured within a ~660ms window.
         * pass1_ok[] tracks which batteries are alive for pass 2.
         * current_ma[] is populated here for motion detection and offset cal.
         */
		/* ── PASS 1: Serial + current for all batteries ── */
		int64_t pass1_start_us = esp_timer_get_time();
		bool  pass1_ok[BMS_BATTERY_COUNT];
		float pass1_snaps[BMS_BATTERY_COUNT];
		
		/* Zero-initialise both arrays regardless of count */
		memset(pass1_ok,    0, sizeof(pass1_ok));
		memset(pass1_snaps, 0, sizeof(pass1_snaps));
		trike_sensor_data_t shunt_tmp = {0};
		
		for (bms_battery_id_t batt_id = BMS_BATTERY_1;
		     batt_id < BMS_BATTERY_COUNT; batt_id++) {
		
		    pass1_ok[batt_id] = read_battery_pass1(batt_id, &staging[batt_id]);
		    current_ma[batt_id] = staging[batt_id].batt_i;
		
		    /* Snap shunt immediately after this battery's current read (~1ms, no delays) */
		    float snap = 0.0f;
		    if (trike_sensors_read_current(&shunt_tmp, &snap) == ESP_OK) {
		        pass1_snaps[batt_id] = snap;
		    } else {
		        /* Failed: carry forward last good snap for this slot */
		        pass1_snaps[batt_id] = s_shunt_snap[(int)batt_id];
		        ESP_LOGW(TAG, "[P1] Shunt snap failed for B%d — using last", batt_id + 1);
		    }
		}
		
		/*1 BATTERY SETUP*/
		/*Compute statistics from the 1 snap */
/*		float avg = pass1_snaps[0];
		float mx  = pass1_snaps[0];
		float rms = sqrtf((pass1_snaps[0]*pass1_snaps[0]) / 1.0f);
		
		s_last_pass1_ms = (uint32_t)((esp_timer_get_time() - pass1_start_us) / 1000);
		
		ESP_LOGI(TAG, "[P1] B1=%" PRId32 "mA "
		         "shunt: B1=%.3fA  avg=%.3fA  max=%.3fA  rms=%.3fA  "
		         "pass1=%" PRIu32 "ms",
		         current_ma[0],
		         pass1_snaps[0],
		         avg, mx, rms,
		         s_last_pass1_ms);*/
		 
		/*2 BATTERY SETUP*/         
		/*Compute statistics from the 2 snaps */
/*		float avg = (pass1_snaps[0] + pass1_snaps[1] ) / 2.0f;
		float mx  = pass1_snaps[0];
		if (pass1_snaps[1] > mx) mx = pass1_snaps[1];
		
		float rms = sqrtf((pass1_snaps[0]*pass1_snaps[0] +
		                   pass1_snaps[1]*pass1_snaps[1]) / 2.0f);
		
		s_last_pass1_ms = (uint32_t)((esp_timer_get_time() - pass1_start_us) / 1000);
		
		ESP_LOGI(TAG, "[P1] B1=%" PRId32 "mA B2=%" PRId32 "mA"
		         "shunt: B1=%.3fA B2=%.3fA avg=%.3fA  max=%.3fA  rms=%.3fA  "
		         "pass1=%" PRIu32 "ms",
		         current_ma[0], current_ma[1],
		         pass1_snaps[0], pass1_snaps[1], 
		         avg, mx, rms,
		         s_last_pass1_ms);	*/
		         		         
		/*3 BATTERY SETUP*/         
		// Compute statistics from the 3 snaps 
		float avg = (pass1_snaps[0] + pass1_snaps[1] + pass1_snaps[2]) / 3.0f;
		float mx  = pass1_snaps[0];
		if (pass1_snaps[1] > mx) mx = pass1_snaps[1];
		if (pass1_snaps[2] > mx) mx = pass1_snaps[2];
		float rms = sqrtf((pass1_snaps[0]*pass1_snaps[0] +
		                   pass1_snaps[1]*pass1_snaps[1] +
		                   pass1_snaps[2]*pass1_snaps[2]) / 3.0f);
		
		s_last_pass1_ms = (uint32_t)((esp_timer_get_time() - pass1_start_us) / 1000);
		
		ESP_LOGI(TAG, "[P1] B1=%" PRId32 "mA B2=%" PRId32 "mA B3=%" PRId32 "mA  "
		         "shunt: B1=%.3fA B2=%.3fA B3=%.3fA  avg=%.3fA  max=%.3fA  rms=%.3fA  "
		         "pass1=%" PRIu32 "ms",
		         current_ma[0], current_ma[1], current_ma[2],
		         pass1_snaps[0], pass1_snaps[1], pass1_snaps[2],
		         avg, mx, rms,
		         s_last_pass1_ms);		         

        /* ── PASS 2: All remaining registers for each alive battery ──────
         * Non-time-sensitive. Voltage, SOC, SOH, capacity etc.
         */
        int64_t pass2_start_us = esp_timer_get_time();
        
        for (bms_battery_id_t batt_id = BMS_BATTERY_1;
             batt_id < BMS_BATTERY_COUNT;
             batt_id++) {
            if (!pass1_ok[batt_id]) {
                system_stats.battery_stats[batt_id].last_failure_timestamp =
                    staging[batt_id].timestamp;
                ESP_LOGE(TAG, "Battery %d pass 1 failed — skipping pass 2",
                         batt_id + 1);
                continue;
            }
            bool ok2 = read_battery_pass2(batt_id, &staging[batt_id]);
            if (ok2) {
                s_batteries_ok_count++;
				/*Update current_ma with pass 2 re-read for motion/offset cal 
                current_ma[batt_id] = staging[batt_id].batt_i;*/
            } else {
                system_stats.battery_stats[batt_id].last_failure_timestamp =
                    staging[batt_id].timestamp;
                ESP_LOGE(TAG, "Battery %d pass 2 failed", batt_id + 1);
            }
        }
        
        s_last_pass2_ms = (uint32_t)((esp_timer_get_time() - pass2_start_us) / 1000);        
 
		s_batteries_ok_count_final = s_batteries_ok_count;
        system_stats.total_readings++;

        s_last_cycle_ms = (uint32_t)((esp_timer_get_time() - cycle_start_us) / 1000);

        /* ── Commit snapshot — battery data AND shunt written atomically ──
         *
         * s_shunt_current_snapshot_a is written HERE, inside the mutex,
         * not during pass 1. This guarantees that any caller holding the
         * mutex (i.e. bms_monitor_get_snapshot()) reads a shunt value
         * that is from exactly the same sub-second pass-1 window as the
         * battery currents in s_cycle_snapshot[].
         *
         * The next BMS cycle cannot overwrite s_shunt_current_snapshot_a
         * until it completes its own pass 1 (~810ms from now) AND takes
         * this mutex to commit. By that time the publish task has already
         * copied both values out under the same mutex take and holds its
         * own frozen local copies for the entire 3.9s publish sequence.
         */        
		xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
		memcpy(s_cycle_snapshot, staging, sizeof(staging));
		/* Commit all shunt statistics atomically with battery data */
		/*1 BATTERY SETUP*/
		//s_shunt_snap[0]            = pass1_snaps[0];
		
		/*2 BATTERY SETUP*/
		//s_shunt_snap[0]            = pass1_snaps[0];
		//s_shunt_snap[1]            = pass1_snaps[1];

		/*3 BATTERY SETUP*/
		s_shunt_snap[0]            = pass1_snaps[0];
		s_shunt_snap[1]            = pass1_snaps[1];
		s_shunt_snap[2]            = pass1_snaps[2];

		s_shunt_current_snapshot_a = avg;   /* backward-compatible: avg is best single value */
		s_shunt_snap_avg           = avg;
		s_shunt_snap_max           = mx;
		s_shunt_snap_rms           = rms;
		s_snapshot_cycle_id++;
		s_snapshot_ready = true;
		xSemaphoreGive(s_snapshot_mutex);        

        ESP_LOGI(TAG,
                 "[TIMING] cycle %" PRIu32 ": total=%" PRIu32 "ms  "
                 "pass1=%" PRIu32 "ms  pass2=%" PRIu32 "ms  "
                 "shunt=%.3fA  ok=%d/%d",
                 s_snapshot_cycle_id, s_last_cycle_ms,
                 s_last_pass1_ms, s_last_pass2_ms,
                 avg,
                 s_batteries_ok_count_final, BMS_BATTERY_COUNT);
 
/*         ── Snap shunt ADC immediately after BMS reads ──
         *
         * CHANGED: shunt current is captured here, NOT in
         * create_metadata_payload(). This aligns the shunt reading
         * temporally with the BMS cycle data that will be published.
         * create_metadata_payload() must call bms_monitor_get_shunt_snapshot()
         * instead of doing a live trike_sensors_read_current() call.
         
        trike_sensor_data_t shunt_snap = {0};
        float snap_i = 0.0f;
        if (trike_sensors_read_current(&shunt_snap, &snap_i) == ESP_OK) {
            s_shunt_current_snapshot_a = snap_i;
            ESP_LOGI(TAG, "Shunt snapshot: %.3f A", snap_i);
        } else {
            ESP_LOGW(TAG, "Shunt ADC read failed — snapshot unchanged");
        }*/                 
                 
        if (s_batteries_ok_count_final == BMS_BATTERY_COUNT) {
 
            /* ── BMS total current for offset calibration ── */
            float bms_total_a = 0.0f;
            for (int b = 0; b < BMS_BATTERY_COUNT; b++) {
                bms_total_a += (float)current_ma[b] / 1000.0f;
            }
 
        } 
 
        /* ── Motion detection (unchanged logic) ── */
        float gps_speed_kmh = 0.0f;
        bool  gps_valid     = gps_get_speed_cache(&gps_speed_kmh);
        bool  gps_motion    = false;
 
        if (gps_valid) {
            gps_motion = (gps_speed_kmh >= GPS_MOTION_SPEED_THRESHOLD_KMH);
            ESP_LOGD(TAG, "Motion[GPS]: %.2fkm/h → %s",
                     gps_speed_kmh, gps_motion ? "MOVING" : "STOPPED");
        } else {
            ESP_LOGD(TAG, "Motion[GPS]: stale/invalid — using current");
        }
 
        bool current_motion = false;
        
        /*1 BATTERY SETUP*/
/*       	if (!gps_valid) {
            current_motion =
                (llabs(current_ma[0]) > MOTION_CURRENT_THRESHOLD_A * 1000);
        }*/
        
        /*2 BATTERY SETUP*/
/*        if (!gps_valid) {
            current_motion =
                (llabs(current_ma[0]) > MOTION_CURRENT_THRESHOLD_A * 1000)||
                (llabs(current_ma[1]) > MOTION_CURRENT_THRESHOLD_A * 1000);
        }*/
        
        /*3 BATTERY SETUP*/
        if (!gps_valid) {
            current_motion =
                (llabs(current_ma[0]) > MOTION_CURRENT_THRESHOLD_A * 1000)||
                (llabs(current_ma[1]) > MOTION_CURRENT_THRESHOLD_A * 1000) ||
                (llabs(current_ma[2]) > MOTION_CURRENT_THRESHOLD_A * 1000);
        }                

		/*Do not allow trike power off if less than all configured batteries are being read and GPS is unavailable*/        
/*		if (!gps_valid && s_batteries_ok_count_final < BMS_BATTERY_COUNT) {
			current_motion = true;
		}  */
		/* ── Partial-battery motion detection retry ──────────────────────────
		 *
		 * If GPS is unavailable AND fewer than all batteries were read, we
		 * cannot rely on current_ma[] for the missing slots.  Instead, retry
		 * current reads on the batteries that DID succeed in pass 1 — if the
		 * pack is under load, the live batteries will show it.
		 *
		 * Up to 3 quick current reads per reachable battery.  If any reading
		 * exceeds the motion threshold we declare motion=true.  Only if every
		 * retry on every reachable battery fails do we force motion=true as a
		 * safe fallback.
		 */
		if (!gps_valid && s_batteries_ok_count_final < BMS_BATTERY_COUNT) {
		
		    bool retry_motion_detected = false;
		    bool any_retry_succeeded   = false;
		
		    for (bms_battery_id_t batt_id = BMS_BATTERY_1;
		         batt_id < BMS_BATTERY_COUNT;
		         batt_id++) {
		
		        /* Only retry batteries that SUCCEEDED in pass 1 */
		        if (!pass1_ok[batt_id]) continue;
		
		        if (bms_rs485_switch_to_battery(batt_id) != ESP_OK) {
		            ESP_LOGW(TAG, "[MotionRetry] MUX switch failed B%d", batt_id + 1);
		            bms_rs485_disable_battery(batt_id);
		            continue;
		        }
		
		        uart_flush_input(BOARD_RS485_UART_NUM);
		        vTaskDelay(pdMS_TO_TICKS(50));
		
		        int32_t retry_i_ma = 0;
		        bool    got_current = false;
		
		        for (uint8_t attempt = 0; attempt < 3 && !got_current; attempt++) {
		            uart_flush_input(BOARD_RS485_UART_NUM);
		            vTaskDelay(pdMS_TO_TICKS(50));
		            int8_t ret = jk_bms_read_batt_current(&jk_devices[batt_id],
		                                                   &retry_i_ma);
		            if (ret == MODBUS_RTU_OK) {
		                got_current = true;
		                any_retry_succeeded = true;
/*		                ESP_LOGI(TAG,
		                         "[MotionRetry] B%d attempt %d: I=%" PRId32 "mA",
		                         batt_id + 1, attempt + 1, retry_i_ma);*/
		            } else {
/*		                ESP_LOGW(TAG,
		                         "[MotionRetry] B%d attempt %d failed (err %d)",
		                         batt_id + 1, attempt + 1, ret);*/
		            }
		        }
		
		        bms_rs485_disable_battery(batt_id);
		        vTaskDelay(pdMS_TO_TICKS(10));
		
		        if (got_current &&
		            llabs(retry_i_ma) > MOTION_CURRENT_THRESHOLD_A * 1000) {
		            retry_motion_detected = true;
/*		            ESP_LOGI(TAG,
		                     "[MotionRetry] B%d over threshold "
		                     "(%" PRId32 "mA > %dmA) — motion",
		                     batt_id + 1, retry_i_ma,
		                     MOTION_CURRENT_THRESHOLD_A * 1000);*/
		        }
		    }
		
		    if (any_retry_succeeded) {
		        current_motion = current_motion || retry_motion_detected;
/*		        ESP_LOGI(TAG, "[MotionRetry] result: motion=%s",
		                 current_motion ? "YES" : "NO");*/
		        
		    } else {
		        /* All reachable batteries also failed retry — safe fallback */
		        current_motion = true;

		       /* ESP_LOGW(TAG, "[MotionRetry] all retries failed — forcing motion=true");*/
		    }
		}		            
 
        trike_in_motion = gps_valid ? gps_motion : current_motion;
 
        /* ── Interval floor (no-op in practice: read always > 1000ms) ── */
        uint32_t elapsed = (uint32_t)(
            (esp_timer_get_time() - cycle_start_us) / 1000);
        if (elapsed < BMS_READ_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(BMS_READ_INTERVAL_MS - elapsed));
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
    //return true;
}

uint32_t bms_monitor_task_get_stack_hwm(void)
{
    return (bms_monitor_task_handle != NULL) ?
           uxTaskGetStackHighWaterMark(bms_monitor_task_handle) : 0;
}

bool bms_monitor_set_discharge_all(uint8_t state)
{
    bool all_ok = true;
    const char *label = (state == JK_MOSFET_ENABLE) ? "ENABLE" : "DISABLE";

    for (bms_battery_id_t id = BMS_BATTERY_1;
         id < BMS_BATTERY_COUNT; id++) {

        if (bms_rs485_switch_to_battery(id) != ESP_OK) {
            all_ok = false;
            bms_rs485_disable_battery(id);
            continue;
        }

        int ret = -1;
        for (int retry = 0; retry < 2; retry++) {
            if (retry > 0) vTaskDelay(pdMS_TO_TICKS(100));
            ret = jk_bms_toggle_discharge(&jk_devices[id], state);
            if (ret == MODBUS_RTU_OK) break;
        }

        if (ret != MODBUS_RTU_OK) {
            ESP_LOGE(TAG, "Discharge %s failed for battery %d",
                     label, id + 1);
            all_ok = false;
        }

        bms_rs485_disable_battery(id);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return all_ok;
}

esp_err_t bms_monitor_task_init(struct modbus_rtu_interface_s *interface)
{
    if (!interface || !interface->read || !interface->write) {
        ESP_LOGE(TAG, "Invalid Modbus interface");
        return ESP_ERR_INVALID_ARG;
    }

    bms_rs485_mux_init();

    esp_err_t ret = bms_serial_storage_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Serial storage init failed — continuing");
    }
    bms_serial_storage_load(cached_serials);

    /*
     * CHANGED: no bms_data_queue created here.
     * Create snapshot mutex instead.
     */
    s_snapshot_mutex = xSemaphoreCreateMutex();
    if (!s_snapshot_mutex) {
        ESP_LOGE(TAG, "Failed to create snapshot mutex");
        return ESP_FAIL;
    }

    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
        jk_devices[i].modbus.device_addr = BMS_DEVICE_ADDRESS;
        jk_devices[i].modbus.interface   = *interface;
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

    // Create BMS monitor task 
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

    ESP_LOGI(TAG, "BMS monitor initialised —  Core %d",
              BMS_MONITOR_TASK_CORE);
    return ESP_OK;
}

uint32_t bms_monitor_get_last_pass1_ms(void) { return s_last_pass1_ms; }
uint32_t bms_monitor_get_last_pass2_ms(void) { return s_last_pass2_ms; }

float bms_monitor_get_shunt_avg(void) { return s_shunt_snap_avg; }
float bms_monitor_get_shunt_max(void) { return s_shunt_snap_max; }
float bms_monitor_get_shunt_rms(void) { return s_shunt_snap_rms; }