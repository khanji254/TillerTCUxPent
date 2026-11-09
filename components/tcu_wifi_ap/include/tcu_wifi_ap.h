/**
 * @file tcu_wifi_ap.h
 * @brief ESP32-S3 Wi-Fi SoftAP and Live Telemetry Web Portal for KSC TCU V1.1
 *
 * Provides a standalone Wi-Fi Access Point (SoftAP) with captive portal DNS,
 * an embedded HTTP server, and an interactive real-time dashboard.
 *
 * Allows field technicians, engineers, or operators to:
 *   - Connect to the TCU via Wi-Fi (SSID: TCU_<serial> or custom)
 *   - View live streaming metrics at 1Hz (GPS, Battery, Shunt Current,
 *     Power, Temperature, SOC, Trike Relay, IMU Accel/RMS, Motion, GSM signal)
 *   - Download full recorded telemetry history as a CSV file (.csv)
 *   - Download instant telemetry state as a JSON file (.json)
 *   - Clear rolling buffer for fresh test sessions
 *
 * Zero external dependencies (all HTML, CSS, JS, and SVG icons are embedded).
 */

#ifndef TCU_WIFI_AP_H_
#define TCU_WIFI_AP_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * DEFAULT CONFIGURATION
 * ========================================================================= */

#define TCU_WIFI_AP_DEFAULT_SSID_PREFIX "TCU_"
#define TCU_WIFI_AP_DEFAULT_PASSWORD    "12345678"  /**< Minimum 8 chars for WPA2, or "" for open */
#define TCU_WIFI_AP_DEFAULT_CHANNEL     1
#define TCU_WIFI_AP_MAX_CONNECTIONS     4
#define TCU_WIFI_AP_IP                  "192.168.4.1"

/**
 * Maximum records stored in RAM circular buffer for CSV download.
 * At ~188 bytes/record, 200 records consumes ~37.6 KB of RAM (~3.3 minutes at 1 Hz).
 * Reduced from 1,000 to avoid DRAM exhaustion on ESP32-S3 without PSRAM.
 */
#define TCU_TELEMETRY_LOG_MAX_RECORDS   200

/* =========================================================================
 * TELEMETRY RECORD STRUCTURE
 * ========================================================================= */

typedef struct {
    uint32_t timestamp_ms;       /**< FreeRTOS tick timestamp (ms)              */
    uint32_t uptime_s;           /**< System uptime in seconds                  */
    char     device_serial[16];  /**< ESP32-S3 MAC serial (e.g. "A4CF12B3E501") */
    char     hub_name[32];       /**< Configured device hub name                */

    /* Power & Battery (ADC / Shunt / Trike) */
    float    pack_voltage_v;     /**< Voltage measurement divider (V)          */
    float    shunt_current_a;    /**< INA240A1PWR instantaneous current (A)     */
    float    shunt_max_a;        /**< Peak current observed in cycle (A)        */
    float    power_w;            /**< Calculated instantaneous power (W)        */
    float    temp_c;             /**< Board/NTC temperature (°C)                */
    uint8_t  avg_soc_pct;        /**< Average battery SOC (0–100%)              */
    bool     trike_pwr_on;       /**< Trike downstream relay/power active       */

    /* Motion & Dynamics (LSM6DSOX 6-axis IMU) */
    bool     motion_detected;    /**< True when vehicle motion detected         */
    bool     impact_detected;    /**< True when impact threshold exceeded       */
    float    accel_x_mg;         /**< Accel X (mg)                              */
    float    accel_y_mg;         /**< Accel Y (mg)                              */
    float    accel_z_mg;         /**< Accel Z (mg)                              */
    float    accel_mag_mg;       /**< 1-second RMS vibration magnitude (mg)     */

    /* GPS & Navigation (Quectel EG915N) */
    bool     gps_valid;          /**< True when valid 2D/3D fix is active       */
    double   latitude;           /**< Latitude (decimal degrees)                */
    double   longitude;          /**< Longitude (decimal degrees)               */
    float    altitude_m;         /**< Altitude above sea level (m)              */
    float    speed_kmh;          /**< Speed over ground (km/h)                  */
    uint8_t  satellites;         /**< Satellites in use                         */
    char     utc_time[16];       /**< UTC time string (hhmmss.ss)               */
    char     utc_date[12];       /**< UTC date string (ddmmyy)                  */

    /* Cellular & Modem */
    int16_t  gsm_rssi;           /**< GSM RSSI (0–31, 99=none)                  */
    int16_t  gsm_dbm;            /**< GSM signal strength in dBm                */
    bool     mqtt_connected;     /**< Cloud MQTT connection status              */

    /* System Health */
    uint32_t free_heap_bytes;    /**< Internal free heap memory (bytes)         */
    char     fw_version[16];     /**< Firmware version string                   */
} tcu_telemetry_record_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize the Wi-Fi SoftAP, DNS server, and HTTP web server.
 *
 * @param ssid_override If NULL or empty, defaults to "TCU_<serial>".
 * @param password      If NULL or shorter than 8 chars (and non-empty), defaults
 *                      to TCU_WIFI_AP_DEFAULT_PASSWORD ("12345678"). Set to "" for open network.
 * @return ESP_OK on success.
 */
esp_err_t tcu_wifi_ap_init(const char *ssid_override, const char *password);

/**
 * @brief Update the current live telemetry state and append to rolling log buffer.
 *
 * Thread-safe. Designed to be called once per second (e.g. from diag_log_task).
 *
 * @param record Pointer to populated telemetry snapshot.
 */
void tcu_wifi_ap_update_telemetry(const tcu_telemetry_record_t *record);

/**
 * @brief Retrieve a copy of the latest live telemetry record.
 *
 * @param[out] out Destination struct to copy into.
 * @return true if data was retrieved, false if no record logged yet.
 */
bool tcu_wifi_ap_get_latest(tcu_telemetry_record_t *out);

/**
 * @brief Clear the rolling in-memory CSV log buffer.
 */
void tcu_wifi_ap_clear_log(void);

/**
 * @brief Get the count of records currently in the rolling log buffer.
 *
 * @return Number of stored records (0 up to TCU_TELEMETRY_LOG_MAX_RECORDS).
 */
size_t tcu_wifi_ap_get_log_count(void);

/**
 * @brief Get the number of Wi-Fi clients currently connected to the SoftAP.
 *
 * @return Connected client count.
 */
uint8_t tcu_wifi_ap_get_client_count(void);

#ifdef __cplusplus
}
#endif

#endif /* TCU_WIFI_AP_H_ */
