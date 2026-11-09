/**
 * @file Quectel_gps.h
 * @brief Quectel EG915N GPS Driver — Continuous NMEA Streaming via UART0
 *
 * On the TCU V1.1 board the Quectel EG915N debug UART streams continuous
 * NMEA-0183 sentences to the ESP32-S3 UART0 (GPIO43 TX / GPIO44 RX).
 *
 * This module:
 *   - Enables the Quectel GPS engine via AT+QGPS=1 on the main UART
 *   - Configures the debug UART output port for NMEA sentences
 *   - Provides a background task that reads GPRMC/GPGGA sentences from
 *     UART0 and updates an internal position cache
 *   - Exposes a non-blocking getter that returns the latest valid fix
 *
 * Only valid GPS fixes are returned.  If no valid fix is available the
 * caller receives ESP_FAIL and should skip the GPS fields in the payload.
 * Stale/cached coordinates from previous fixes are not published.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef QUECTEL_GPS_H_
#define QUECTEL_GPS_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
/** Speed above which the trike is considered in motion (km/h). */
//#define GPS_MOTION_SPEED_THRESHOLD_KMH   1.5f
#define GPS_MOTION_SPEED_THRESHOLD_KMH   5.0f

/** Maximum age of a cached GPS speed reading before falling back
 *  to current-based detection (ms).
 *  diag_log_task polls every 1s; 5s allows 4 missed polls. */
#define GPS_MOTION_SPEED_MAX_AGE_MS      5000
/* =========================================================================
 * DATA TYPES
 * ========================================================================= */

/**
 * @brief GPS fix quality as reported in GPGGA sentence field 6.
 */
typedef enum {
    GPS_FIX_NONE    = 0,   /**< No fix / invalid                             */
    GPS_FIX_INVALID = 1,   /**< Position reported but fix quality invalid    */
    GPS_FIX_2D      = 2,   /**< 2-D fix (altitude may be unreliable)         */
    GPS_FIX_3D      = 3,   /**< 3-D fix (full position valid)                */
} gps_fix_quality_t;

/**
 * @brief Parsed GPS position snapshot.
 */
typedef struct {
    double             latitude;      /**< Decimal degrees, + = North        */
    double             longitude;     /**< Decimal degrees, + = East         */
    float              altitude;      /**< Metres above sea level             */
    float              speed_kmh;     /**< Speed over ground (km/h)           */
    float              hdop;          /**< Horizontal dilution of precision   */
    uint8_t            satellites;    /**< Number of satellites in use        */
    gps_fix_quality_t  fix;           /**< Fix quality indicator              */
    bool               valid;         /**< true only for 2D or 3D fix        */
    char               utc_time[12];  /**< UTC time string (hhmmss.ss)        */
    char               utc_date[8];   /**< UTC date string (ddmmyy)           */
    uint32_t           timestamp_ms;  /**< System tick when sentence was parsed */
} gps_position_t;

/**
 * @brief GPS subsystem configuration.
 */
typedef struct {
    bool    agps_enabled;     /**< Enable AGPS for faster first fix           */
    uint8_t min_satellites;   /**< Minimum satellites required for valid fix  */
} gps_config_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise the GPS subsystem.
 *
 * Configures the Quectel GPS engine via AT commands on the main modem UART
 * (AT+QGPS=1, output port = UART, NMEA format), then starts the background
 * NMEA parsing task on UART0.
 *
 * Must be called after the modem UART is initialised and the MQTT connection
 * is established (so the main UART mutex is available).
 *
 * @param[in] config  GPS configuration, or NULL for defaults.
 * @return ESP_OK on success.
 */
esp_err_t gps_init(const gps_config_t *config);

/**
 * @brief Enable GPS engine (send AT+QGPS=1 on main modem UART).
 *
 * @return ESP_OK on success.
 */
esp_err_t gps_enable(void);

/**
 * @brief Disable GPS engine (send AT+QGPSEND on main modem UART).
 *
 * @return ESP_OK on success.
 */
esp_err_t gps_disable(void);

/**
 * @brief Get the latest valid GPS position from the streaming parser.
 *
 * Non-blocking.  Returns the most recently parsed valid fix.
 * Returns ESP_FAIL (and sets position->valid = false) if no valid
 * fix has been obtained since the last call or since boot.
 *
 * @param[out] position  Pointer to structure to receive the fix.
 * @return ESP_OK if a valid fix is available, ESP_FAIL otherwise.
 */
esp_err_t gps_get_latest_position(gps_position_t *position);

/**
 * @brief Fill default GPS configuration values.
 *
 * @param[out] config  Config structure to populate with defaults.
 */
void gps_get_default_config(gps_config_t *config);

/**
 * @brief Return true if at least one valid GPS fix has been received.
 *
 * @return true if a fix is available, false otherwise.
 */
bool gps_has_fix(void);

/**
 * @brief Update the internal GPS speed cache.
 *
 * Called after any successful gps_get_latest_position() call.
 * Thread-safe.  Does not touch the modem UART.
 *
 * @param[in] speed_kmh  Speed over ground in km/h from the RMC sentence.
 * @param[in] fix_valid  true if the GPS fix was valid when speed was read.
 */
void gps_update_speed_cache(float speed_kmh, bool fix_valid);

/**
 * @brief Read the cached GPS speed without polling the modem.
 *
 * Reads the value last written by gps_update_speed_cache().
 * Returns false if the cache is stale or fix was invalid.
 * Safe to call from any task — no UART mutex taken.
 *
 * @param[out] speed_kmh_out  Populated with cached speed if return is true.
 * @return true if a fresh valid speed is available.
 */
bool gps_get_speed_cache(float *speed_kmh_out);


#ifdef __cplusplus
}
#endif

#endif /* QUECTEL_GPS_H_ */