/**
 * @file Quectel_gps.c
 * @brief Quectel EG915N GPS — Continuous NMEA Streaming Implementation
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "Quectel_gps.h"
#include "Quectel_mqtt.h"   /* send_at_command(), uart_modem_mutex           */
#include "board_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TAG_GPS "GPS"

/* =========================================================================
 * INTERNAL CONSTANTS
 * ========================================================================= */

#define GPS_TASK_STACK_SIZE   3072
#define GPS_TASK_PRIORITY     4
#define GPS_TASK_CORE         0
#define GPS_NMEA_BUF_SIZE     256

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static struct {
    gps_config_t   config;
    gps_position_t position;
    bool           has_fix;
    bool           initialized;
    SemaphoreHandle_t mutex;
} gps_state = {
    .has_fix     = false,
    .initialized = false,
};

/* =========================================================================
 * GPS SPEED CACHE — written by diag_log_task, read by bms_monitor_task
 * No modem UART involved. Protected by its own mutex.
 * ========================================================================= */

static struct {
    float             speed_kmh;
    bool              fix_valid;
    uint32_t          timestamp_ms;
    SemaphoreHandle_t mutex;
} s_speed_cache = {
    .speed_kmh    = 0.0f,
    .fix_valid    = false,
    .timestamp_ms = 0,
    .mutex        = NULL,
};

/* =========================================================================
 * NMEA PARSING HELPERS
 * ========================================================================= */

/**
 * @brief Convert NMEA lat/lon raw string to decimal degrees.
 *
 * NMEA format: DDDMM.MMMMM (degrees + decimal minutes).
 * Decimal degrees = D + M/60.
 *
 * @param[in]  raw      NMEA position field (e.g. "0136.12345").
 * @param[in]  dir      Direction character ('N','S','E','W').
 * @return Decimal degrees value (negative for S or W).
 */
static double nmea_to_decimal_degrees(const char *raw, char dir)
{
    if (!raw || raw[0] == '\0') {
        return 0.0;
    }

    double raw_val = atof(raw);

    /* Integer part is degrees × 100; fractional part is minutes */
    int    degrees = (int)(raw_val / 100);
    double minutes = raw_val - (degrees * 100.0);
    double decimal = degrees + minutes / 60.0;

    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/**
 * @brief Extract a comma-delimited field from an NMEA sentence.
 *
 * @param[in]  sentence  Full NMEA sentence string.
 * @param[in]  field_num Zero-based field index (0 = sentence type).
 * @param[out] out       Buffer to receive the field.
 * @param[in]  out_size  Buffer size.
 */
static void nmea_get_field(const char *sentence, int field_num,
                            char *out, size_t out_size)
{
    out[0] = '\0';
    int field = 0;
    const char *p = sentence;

    while (*p && field < field_num) {
        if (*p == ',') {
            field++;
        }
        p++;
    }

    /* Copy until next comma, asterisk, or end */
    size_t idx = 0;
    while (*p && *p != ',' && *p != '*' && idx < out_size - 1) {
        out[idx++] = *p++;
    }
    out[idx] = '\0';
}

/**
 * @brief Parse a GPRMC sentence and update the position structure.
 *
 * GPRMC format:
 *   $GPRMC,hhmmss.ss,A,DDMM.MMMMM,N,DDDMM.MMMMM,E,knots,COG,DDMMYY,...
 *   Field 2 = 'A' (active/valid) or 'V' (void/invalid)
 *
 * @param[in]  sentence  Full GPRMC sentence.
 * @param[out] pos       Position to update.
 * @return true if sentence was valid and position updated.
 */
static bool parse_gprmc(const char *sentence, gps_position_t *pos)
{
    char field[32];

    /* Field 1: UTC time */
    nmea_get_field(sentence, 1, field, sizeof(field));
    strncpy(pos->utc_time, field, sizeof(pos->utc_time) - 1);

    /* Field 2: Status — 'A' = valid, 'V' = void */
    nmea_get_field(sentence, 2, field, sizeof(field));
    if (field[0] != 'A') {
        pos->valid = false;
        return false;
    }

    /* Field 3: Latitude */
    char lat_str[16], lat_dir[4], lon_str[16], lon_dir[4];
    nmea_get_field(sentence, 3, lat_str, sizeof(lat_str));
    nmea_get_field(sentence, 4, lat_dir, sizeof(lat_dir));
    nmea_get_field(sentence, 5, lon_str, sizeof(lon_str));
    nmea_get_field(sentence, 6, lon_dir, sizeof(lon_dir));

    pos->latitude  = nmea_to_decimal_degrees(lat_str, lat_dir[0]);
    pos->longitude = nmea_to_decimal_degrees(lon_str, lon_dir[0]);

    /* Field 7: Speed over ground (knots → km/h) */
    nmea_get_field(sentence, 7, field, sizeof(field));
    if (field[0] != '\0') {
        pos->speed_kmh = (float)atof(field) * 1.852f;
    }

    /* Field 9: Date (DDMMYY) */
    nmea_get_field(sentence, 9, field, sizeof(field));
    strncpy(pos->utc_date, field, sizeof(pos->utc_date) - 1);

    pos->valid        = true;
    pos->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    return true;
}

/**
 * @brief Parse a GPGGA sentence for altitude, satellites, HDOP and fix type.
 *
 * GPGGA format:
 *   $GPGGA,hhmmss.ss,lat,N,lon,E,fix,sats,HDOP,alt,M,...
 *   Field 6 = fix quality (0=invalid, 1=GPS, 2=DGPS)
 *
 * @param[in]  sentence  Full GPGGA sentence.
 * @param[out] pos       Position to update.
 */
static void parse_gpgga(const char *sentence, gps_position_t *pos)
{
    char field[32];

    /* Field 6: Fix quality */
    nmea_get_field(sentence, 6, field, sizeof(field));
    int fix_q = atoi(field);
    if (fix_q >= 1) {
        pos->fix = GPS_FIX_3D;   /* Treat any quality ≥ 1 as 3D fix */
    } else {
        pos->fix   = GPS_FIX_NONE;
        pos->valid = false;
        return;
    }

    /* Field 7: Satellites in use */
    nmea_get_field(sentence, 7, field, sizeof(field));
    pos->satellites = (uint8_t)atoi(field);

    /* Field 8: HDOP */
    nmea_get_field(sentence, 8, field, sizeof(field));
    pos->hdop = (float)atof(field);

    /* Field 9: Altitude (metres above sea level) */
    nmea_get_field(sentence, 9, field, sizeof(field));
    pos->altitude = (float)atof(field);

    /* Apply minimum satellite gate */
    if (pos->satellites < gps_state.config.min_satellites) {
        pos->valid = false;
        pos->fix   = GPS_FIX_NONE;
    }
}

/* =========================================================================
 * NMEA READER TASK
 * ========================================================================= */

/**
 * @brief Background task that reads NMEA sentences from UART0 and
 *        maintains the latest GPS position in the module state.
 *
 * @param[in] pvParameters  Unused.
 */
static void gps_nmea_task(void *pvParameters)
{
    ESP_LOGI(TAG_GPS, "NMEA streaming task started on Core %d",
             xPortGetCoreID());

    char     line_buf[GPS_NMEA_BUF_SIZE];
    int      line_len = 0;
    uint8_t  byte;

    while (1) {
        /* Read one byte at a time with a 10 ms timeout */
        int n = uart_read_bytes(BOARD_GPS_UART_NUM, &byte, 1,
                                pdMS_TO_TICKS(10));
        if (n <= 0) {
            continue;
        }

        if (byte == '$') {
            /* Start of new sentence — reset buffer */
            line_len        = 0;
            line_buf[0]     = '$';
            line_len        = 1;
        } else if (byte == '\n' || byte == '\r') {
            /* End of sentence */
            if (line_len > 6) {
                line_buf[line_len] = '\0';

                gps_position_t tmp;

                /* Take snapshot of current state to update */
                if (xSemaphoreTake(gps_state.mutex,
                                   pdMS_TO_TICKS(5)) == pdTRUE) {
                    memcpy(&tmp, &gps_state.position, sizeof(tmp));
                    xSemaphoreGive(gps_state.mutex);
                } else {
                    line_len = 0;
                    continue;
                }

                bool updated = false;

                if (strncmp(line_buf, "$GPRMC", 6) == 0 ||
                    strncmp(line_buf, "$GNRMC", 6) == 0) {
                    updated = parse_gprmc(line_buf, &tmp);
                } else if (strncmp(line_buf, "$GPGGA", 6) == 0 ||
                           strncmp(line_buf, "$GNGGA", 6) == 0) {
                    parse_gpgga(line_buf, &tmp);
                    updated = true;
                }

                if (updated) {
                    if (xSemaphoreTake(gps_state.mutex,
                                       pdMS_TO_TICKS(5)) == pdTRUE) {
                        memcpy(&gps_state.position, &tmp, sizeof(tmp));
                        if (tmp.valid) {
                            gps_state.has_fix = true;
                        }
                        xSemaphoreGive(gps_state.mutex);
                    }
                }

                ESP_LOGD(TAG_GPS, "NMEA: %s", line_buf);
            }
            line_len = 0;
        } else {
            /* Accumulate sentence bytes */
            if (line_len < GPS_NMEA_BUF_SIZE - 2) {
                line_buf[line_len++] = (char)byte;
            } else {
                /* Buffer overflow — discard and wait for next '$' */
                line_len = 0;
            }
        }
    }
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

void gps_get_default_config(gps_config_t *config)
{
    if (!config) return;
    config->agps_enabled   = true;
    config->min_satellites = 3;
}

esp_err_t gps_enable(void)
{
	char raw_buf[128];
    memset(raw_buf, 0, sizeof(raw_buf));
    
    /* Enable GPS engine */
    bool ok = send_at_command("AT+QGPS=1", "OK",
                              3, 1000, raw_buf, sizeof(raw_buf));

    if (!ok) {
        /* Check if CME error is acceptable */
        if (strstr(raw_buf, "503") ||
            strstr(raw_buf, "504")) {
            /* 503 = busy (already running), 504 = session ongoing — both OK */
            return ESP_OK;
        }
        if (strstr(raw_buf, "+CME ERROR")) {
            /* Any other CME error = hardware or module fault */
            return ESP_FAIL;
        }
        return ESP_FAIL;;
    }
    /* Poll for a GGA sentence — validates NMEA path */
    memset(raw_buf, 0, sizeof(raw_buf));
    ESP_LOGI(TAG_GPS, "GPS engine enabled");
    
	char nmea_buf[256] = {0};
	vTaskDelay(pdMS_TO_TICKS(2000));  // let engine get first fix attempt
	send_at_command("AT+QGPSGNMEA=\"GGA\"", "QGPSGNMEA", 2, 3000,
	                nmea_buf, sizeof(nmea_buf));
	ESP_LOGI(TAG_GPS, "GNMEA poll: %s", nmea_buf);    
    return ESP_OK;
}

esp_err_t gps_disable(void)
{
    if (!send_at_command("AT+QGPSEND", "OK", 3, 1000, NULL, 0)) {
        ESP_LOGW(TAG_GPS, "GPS engine disable command failed (may already be off)");
    }
    return ESP_OK;
}

esp_err_t gps_init(const gps_config_t *config)
{
    if (gps_state.initialized) {
        return ESP_OK;
    }
    
    /* Flush any stale modem responses before sending GPS AT commands.
     * The factory test AT+COPS / AT+CPIN responses may have arrived
     * late and are sitting in the UART RX buffer, causing a one-command
     * pipeline shift that makes every GPS config command read the wrong response. */
    uart_flush_input(MODEM_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(200));    

    /* Apply configuration */
    if (config) {
        memcpy(&gps_state.config, config, sizeof(gps_config_t));
    } else {
        gps_get_default_config(&gps_state.config);
    }

    memset(&gps_state.position, 0, sizeof(gps_state.position));
    gps_state.has_fix = false;

    /* Create mutex for position access */
    gps_state.mutex = xSemaphoreCreateMutex();
    if (!gps_state.mutex) {
        ESP_LOGE(TAG_GPS, "Failed to create GPS mutex");
        return ESP_FAIL;
    }

    /*
     * Configure Quectel debug UART to output NMEA sentences.
     * AT+QGPSCFG="outport","uartdebug"  — route NMEA to debug UART port.
     * AT+QGPSCFG="nmeasrc",1          — enable NMEA from GPS engine.
     * AT+QGPSCFG="gpsnmeatype",31     — GGA+RMC+GSV+GSA+VTG sentences.
     * AT+QGPSCFG = "gnssconfig", 1	- enable streaming of GPS data from Beidou, Galileo and glonass satelites
     */
    send_at_command("AT+QGPSCFG=\"outport\",\"uartdebug\"", "OK",
                    3, 500, NULL, 0);
    send_at_command("AT+QGPSCFG=\"nmeasrc\",1", "OK",
                    3, 500, NULL, 0);
    send_at_command("AT+QGPSCFG=\"gpsnmeatype\",31", "OK",
                    3, 500, NULL, 0);
    send_at_command("AT+QGPSCFG=\"gnssconfig\",5", "OK",
                    3, 500, NULL, 0);

    if (gps_state.config.agps_enabled) {
        send_at_command("AT+QAGPS=1", "OK", 3, 500, NULL, 0);
    }

    /* Enable GPS engine */
    esp_err_t ret = gps_enable();
    if (ret != ESP_OK) {
        return ret;
    }

    /* Install UART0 driver for NMEA streaming */
    uart_config_t uart_cfg = {
        .baud_rate  = BOARD_GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BOARD_GPS_UART_NUM,
                                        BOARD_GPS_BUF_SIZE * 2,
                                        0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_GPS_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_GPS_UART_NUM,
                                  BOARD_GPS_TX_PIN,
                                  BOARD_GPS_RX_PIN,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));

    /* Start background NMEA parsing task */
    BaseType_t created = xTaskCreatePinnedToCore(
        gps_nmea_task,
        "gps_nmea",
        GPS_TASK_STACK_SIZE,
        NULL,
        GPS_TASK_PRIORITY,
        NULL,
        GPS_TASK_CORE
    );

    if (created != pdPASS) {
        ESP_LOGE(TAG_GPS, "Failed to create GPS NMEA task");
        return ESP_FAIL;
    }

    gps_state.initialized = true;
    ESP_LOGI(TAG_GPS, "GPS initialised — NMEA streaming on UART%d "
             "(RX=IO%d TX=IO%d)",
             BOARD_GPS_UART_NUM, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);
    return ESP_OK;
}

esp_err_t gps_get_latest_position(gps_position_t *position)
{
    if (!position || !gps_state.initialized) {
		if(!gps_state.initialized){
			ESP_LOGE(TAG_GPS, "GPS not initialized!");
			gps_config_t cfg;
	    	gps_get_default_config(&cfg);
	    	if (gps_init(&cfg) != ESP_OK){return ESP_FAIL;};
		}
		
		else if(!position){
			return ESP_ERR_INVALID_ARG;
		}
    }

/*     Try the streaming cache first 
    if (gps_state.has_fix) {
        if (xSemaphoreTake(gps_state.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(position, &gps_state.position, sizeof(gps_position_t));
            xSemaphoreGive(gps_state.mutex);
            if (position->valid) {
                return ESP_OK;
            }
        }
    }
*/
    /*
     * REQ 2: Streaming data not available — fall back to AT+QGPSGNMEA
     * polling on the main UART.
     *
     * Per GNSS App Note §3.3: AT+QGPSCFG="nmeasrc",1 must be set first
     * (done in gps_init).  AT+QGPSGNMEA="GGA" returns the latest GGA
     * sentence; +CME ERROR: 516 = no fix yet (engine running but no
     * satellite fix available).
     *
     * We parse the GGA sentence directly here to populate the position
     * struct.  This is a best-effort fallback — if neither streaming
     * nor polling returns valid data, we return ESP_FAIL with valid=false.
     */
    char nmea_buf[256] = {0};
    bool poll_ok = send_at_command("AT+QGPSGNMEA=\"GGA\"",
                                    "QGPSGNMEA",
                                    2, 600,
                                    nmea_buf, sizeof(nmea_buf));

    if (!poll_ok) {
        /* Check for +CME ERROR: 516 (not fixed) — engine is alive */
        if (strstr(nmea_buf, "+CME ERROR: 516")) {
            ESP_LOGD(TAG_GPS, "GPS polled: no fix yet (CME 516)");
        }
        else if(strstr(nmea_buf,"+CME ERROR: 505")){
			ESP_LOGE(TAG_GPS, "GPS not initialized!");
			gps_config_t cfg;
	    	gps_get_default_config(&cfg);
	    	if (gps_init(&cfg) != ESP_OK){return ESP_FAIL;};
		} else {
            ESP_LOGW(TAG_GPS, "GPS poll failed: %s", nmea_buf);
        }
        memset(position, 0, sizeof(gps_position_t));
        position->fix   = GPS_FIX_NONE;
        position->valid = false;
        return ESP_FAIL;
    }

    /* Find the $GPGGA or $GNGGA sentence in the AT response */
    char *gga = strstr(nmea_buf, "$GPGGA");
    if (!gga) gga = strstr(nmea_buf, "$GNGGA");

    if (!gga) {
        memset(position, 0, sizeof(gps_position_t));
        position->fix   = GPS_FIX_NONE;
        position->valid = false;
        return ESP_FAIL;
    }

    /* Terminate the sentence at CR/LF */
    char sentence[128] = {0};
    size_t i = 0;
    while (gga[i] && gga[i] != '\r' && gga[i] != '\n' &&
           i < sizeof(sentence) - 1) {
        sentence[i] = gga[i];
        i++;
    }
    sentence[i] = '\0';

    /* Parse using the existing GGA parser */
    gps_position_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    parse_gpgga(sentence, &tmp);

    /* Also try to get lat/lon from a polled RMC */
    char rmc_buf[256] = {0};
    if (send_at_command("AT+QGPSGNMEA=\"RMC\"", "QGPSGNMEA",
                        2, 600, rmc_buf, sizeof(rmc_buf))) {
        char *rmc = strstr(rmc_buf, "$GPRMC");
        if (!rmc) rmc = strstr(rmc_buf, "$GNRMC");
        if (rmc) {
            char rmc_sent[128] = {0};
            size_t j = 0;
            while (rmc[j] && rmc[j] != '\r' && rmc[j] != '\n' &&
                   j < sizeof(rmc_sent) - 1) {
                rmc_sent[j] = rmc[j]; j++;
            }
            rmc_sent[j] = '\0';
            parse_gprmc(rmc_sent, &tmp);
        }
    }

    memcpy(position, &tmp, sizeof(gps_position_t));

    /* Update the streaming cache too */
    if (xSemaphoreTake(gps_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&gps_state.position, &tmp, sizeof(tmp));
        if (tmp.valid) gps_state.has_fix = true;
        xSemaphoreGive(gps_state.mutex);
    }

    ESP_LOGI(TAG_GPS, "GPS fallback poll: fix=%d sats=%d lat=%.5f lon=%.5f",
             tmp.fix, tmp.satellites, tmp.latitude, tmp.longitude);

    return tmp.valid ? ESP_OK : ESP_FAIL;
}

bool gps_has_fix(void)
{
    return gps_state.has_fix;
}

void gps_update_speed_cache(float speed_kmh, bool fix_valid)
{
    /* Lazy-init the mutex on first call — safe because diag_log_task
     * starts after scheduler is running and this is called from one task. */
    if (s_speed_cache.mutex == NULL) {
        s_speed_cache.mutex = xSemaphoreCreateMutex();
        if (!s_speed_cache.mutex) {
            ESP_LOGE(TAG_GPS, "Failed to create speed cache mutex");
            return;
        }
    }

    if (xSemaphoreTake(s_speed_cache.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_speed_cache.speed_kmh    = speed_kmh;
        s_speed_cache.fix_valid    = fix_valid;
        s_speed_cache.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        xSemaphoreGive(s_speed_cache.mutex);
    }
}

bool gps_get_speed_cache(float *speed_kmh_out)
{
    if (!speed_kmh_out || s_speed_cache.mutex == NULL) return false;

    float    spd   = 0.0f;
    bool     valid = false;
    uint32_t ts    = 0;

    if (xSemaphoreTake(s_speed_cache.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        spd   = s_speed_cache.speed_kmh;
        valid = s_speed_cache.fix_valid;
        ts    = s_speed_cache.timestamp_ms;
        xSemaphoreGive(s_speed_cache.mutex);
    } else {
        return false;
    }

    if (!valid || ts == 0) return false;

    uint32_t age_ms = (uint32_t)(esp_timer_get_time() / 1000) - ts;
    if (age_ms > GPS_MOTION_SPEED_MAX_AGE_MS) {
        ESP_LOGD(TAG_GPS, "Speed cache stale: age=%" PRIu32 "ms", age_ms);
        return false;
    }

    *speed_kmh_out = spd;
    return true;
}