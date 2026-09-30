/**
 * @file factory_test.c
 * @brief Factory / Startup Peripheral Test Suite Implementation
 *
 * Modem test extended per Quectel EG915N AT Commands Manual v1.2:
 *   - Four AT command stages: AT / ATI / AT+CGMI / AT+GSN
 *   - Power-cycle retry: up to MODEM_POWERCYCLE_RETRIES full cycles
 *     before declaring FAILED (each cycle: OFF → 1 s wait → ON)
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "factory_test.h"
#include "board_config.h"
#include "version.h"
#include "lis3dhtr.h"
#include "rgb_led.h"
#include "trike_sensors.h"
#include "bms_rs485_mux.h"
#include "Quectel_mqtt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define TAG "FACTORY_TEST"

/* =========================================================================
 * INTERNAL CONSTANTS
 * ========================================================================= */

#define TEST_SEPARATOR       "============================================\r\n"
#define LED_USER_TIMEOUT_MS  5000   /**< LED sequence auto-complete window   */
#define RS485_HELLO_REPEATS  5      /**< Transmit count per battery bus      */

/* =========================================================================
 * REPORTING HELPERS
 * ========================================================================= */

/** Print a result line to USB console. */
static void print_result(const factory_test_result_t *r)
{
    if (r->passed) {
        printf("  [PASS] %-44s  %s\r\n", r->name, r->detail);
    } else {
        printf("  [FAIL] %-44s  -> %s\r\n", r->name, r->detail);
    }
}

/* =========================================================================
 * INTERNAL MODEM HELPERS
 * ========================================================================= */

/**
 * @brief Strip leading/trailing whitespace and control characters from a
 *        modem response field extracted by the test, writing the cleaned
 *        string into @p out (max @p out_size bytes including null).
 *
 * Used to produce clean display strings from raw modem response buffers.
 */
static void strip_response(const char *src, char *out, size_t out_size)
{
    if (!src || !out || out_size == 0) return;

    /* Advance past leading whitespace / CR / LF */
    while (*src && (isspace((unsigned char)*src))) {
        src++;
    }

    size_t i = 0;
    while (*src && i < out_size - 1) {
        /* Stop at the first CR or LF — take only the first line */
        if (*src == '\r' || *src == '\n') break;
        out[i++] = *src++;
    }
    out[i] = '\0';
}

/**
 * @brief Extract the IMEI number from a raw AT+GSN / AT+CGSN response.
 *
 * The EG915N responds:
 *   \r\n<15-digit IMEI>\r\n\r\nOK\r\n
 *
 * This helper scans for the first run of exactly 15 decimal digits.
 *
 * @param[in]  raw      Raw modem response buffer.
 * @param[out] imei_out Buffer to receive the IMEI string (≥16 bytes).
 * @return true if a 15-digit sequence was found.
 */
static bool extract_imei(const char *raw, char *imei_out)
{
    if (!raw || !imei_out) return false;

    const char *p = raw;
    while (*p) {
        /* Look for the start of a digit run */
        if (isdigit((unsigned char)*p)) {
            const char *start = p;
            int count = 0;
            while (isdigit((unsigned char)*p)) {
                p++;
                count++;
            }
            /* IMEI is exactly 15 digits */
            if (count == 15) {
                memcpy(imei_out, start, 15);
                imei_out[15] = '\0';
                return true;
            }
        } else {
            p++;
        }
    }
    return false;
}

/**
 * @brief Attempt a single modem power cycle: power off, wait, power on.
 *
 * Calls hardware_poweroff() then gsm_poweron() which is the same sequence
 * used by poweron_modem().  The caller must re-verify AT communication
 * after this returns.
 *
 * @param[in] cycle_num  1-based cycle number, used for console logging only.
 */
static void modem_power_cycle(int cycle_num)
{
    printf("  [MODEM] Power cycle %d/%d — OFF...\r\n",
           cycle_num, MODEM_POWERCYCLE_RETRIES);
    hardware_poweroff();
    //vTaskDelay(pdMS_TO_TICKS(MODEM_POWERCYCLE_WAIT_MS));
    vTaskDelay(pdMS_TO_TICKS(3000));   /* was 1000 — give PSU time to recover */
    printf("  [MODEM] Power cycle %d/%d — ON...\r\n",
           cycle_num, MODEM_POWERCYCLE_RETRIES);
    /* Set PWRKEY HIGH before calling gsm_poweron() */
    gpio_set_level(PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));           
    gsm_poweron();
    
   	uart_flush_input(MODEM_UART_NUM);   // Clear UART of noise
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* -----------------------------------------------------------------
 * TEST 6 — GPS engine initialization (fail-fast on CME error)
 * ----------------------------------------------------------------- */

/**
 * @brief Verify GPS engine can be started without CME error.
 *
 * Per GNSS Application Note §2.3.2: AT+QGPS=1 max response 300 ms.
 * CME ERROR 503 = GNSS subsystem busy (engine already running — OK).
 * CME ERROR 516 = Not fixed currently (engine started, no fix yet — OK).
 * Any other CME ERROR = hardware fault → FAIL.
 *
 * AT+QGPSCFG="nmeasrc",1 enables AT-port NMEA polling (§2.3.1.2).
 * AT+QGPSGNMEA="GGA" polls for a GGA sentence (§2.3.5, max 300 ms).
 */
static void test_gps_init(factory_test_result_t *r)
{
    r->name   = "GPS ENGINE INIT (AT+QGPS=1 no CME error)";
    r->passed = false;

    char raw_buf[128];
    memset(raw_buf, 0, sizeof(raw_buf));

    /* Enable NMEA source on AT port */
    send_at_command("AT+QGPSCFG=\"nmeasrc\",1", "OK", 3, 600, NULL, 0);

    /* Enable GPS engine */
    bool ok = send_at_command("AT+QGPS=1", "OK",
                              3, 600, raw_buf, sizeof(raw_buf));

    if (!ok) {
        /* Check if CME error is acceptable */
        if (strstr(raw_buf, "503") ||
            strstr(raw_buf, "504")) {
            /* 503 = busy (already running), 504 = session ongoing — both OK */
            snprintf(r->detail, sizeof(r->detail),
                     "GPS already active (CME 503/504) — engine OK");
            r->passed = true;
            return;
        }
        if (strstr(raw_buf, "+CME ERROR")) {
            /* Any other CME error = hardware or module fault */
            snprintf(r->detail, sizeof(r->detail),
                     "AT+QGPS=1 CME ERROR — GPS hardware fault: %.60s",
                     raw_buf);
            return;
        }
        snprintf(r->detail, sizeof(r->detail),
                 "AT+QGPS=1 no response — modem/GPS unresponsive");
        return;
    }

    /* Poll for a GGA sentence — validates NMEA path */
    memset(raw_buf, 0, sizeof(raw_buf));
    vTaskDelay(pdMS_TO_TICKS(1000));
    bool gga_ok = send_at_command("AT+QGPSGNMEA=\"GGA\"",
                                   "QGPSGNMEA", 2, 600,
                                   raw_buf, sizeof(raw_buf));

    if (gga_ok) {
        /* CME 516 = not fixed yet — engine running but no satellite fix */
        snprintf(r->detail, sizeof(r->detail),
                 "GPS engine started, NMEA path OK: %.60s", raw_buf);
    } else if (strstr(raw_buf, "+CME ERROR: 516")) {
        snprintf(r->detail, sizeof(r->detail),
                 "GPS engine started — no fix yet (CME 516), normal");
    } else {
        snprintf(r->detail, sizeof(r->detail),
                 "GPS engine started — NMEA poll inconclusive");
    }
    r->passed = true;
}

/* =========================================================================
 * INDIVIDUAL TEST FUNCTIONS
 * ========================================================================= */

/* -----------------------------------------------------------------
 * TEST 1 — LIS3DHTR accelerometer
 * ----------------------------------------------------------------- */

/**
 * @brief Verify I2C communication with the LIS3DHTR and read one sample.
 *
 * Checks WHO_AM_I register (expected 0x33 per datasheet) then reads
 * a calibrated acceleration sample.  A Z-axis magnitude near 1000 mg
 * confirms the sensor is responding correctly when the board is flat.
 *
 * @param[out] r  Test result record.
 */
static void test_lis3dhtr(factory_test_result_t *r)
{
    r->name   = "LIS3DHTR ACCELEROMETER";
    r->passed = false;

    uint8_t who = 0;
    esp_err_t ret = lis3dhtr_check_who_am_i(&who);
    if (ret != ESP_OK) {
        snprintf(r->detail, sizeof(r->detail),
                 "I2C read failed (%s)", esp_err_to_name(ret));
        return;
    }

    if (who != LIS3DHTR_WHO_AM_I_VAL) {
        snprintf(r->detail, sizeof(r->detail),
                 "WHO_AM_I=0x%02X expected 0x%02X",
                 who, LIS3DHTR_WHO_AM_I_VAL);
        return;
    }

    lis3dhtr_accel_mg_t accel;
    ret = lis3dhtr_read_accel_mg(&accel);
    if (ret != ESP_OK) {
        snprintf(r->detail, sizeof(r->detail),
                 "Accel read failed (%s)", esp_err_to_name(ret));
        return;
    }

    snprintf(r->detail, sizeof(r->detail),
             "WHO_AM_I=0x33  X=%.0fmg Y=%.0fmg Z=%.0fmg",
             accel.x_mg, accel.y_mg, accel.z_mg);
    r->passed = true;
}

static void test_modem(factory_test_result_t *r)
{
    r->name   = "QUECTEL MODEM (AT/ATI/CGMI/GSN/CSQ/CREG/CPIN)";
    r->passed = false;

    /*
     * AT command stages with per-stage timeouts from Quectel EG915N
     * AT Commands Manual v1.3:
     *
     * Stage 0 — AT       : basic comms. Max resp 300 ms.
     * Stage 1 — ATI      : product ID ("Quectel"). Max resp 300 ms.
     * Stage 2 — ATE0      : Switch off echo mode on the module. Max resp 300 ms.
     * Stage 3 — AT+CGMI  : manufacturer ("Quectel"). Max resp 300 ms.
     * Stage 4 — AT+GSN   : IMEI (15 digits). Max resp 300 ms.
     * Stage 5 — AT+CSQ   : signal quality. Max resp 300 ms.
     *                       rssi=99 = invalid — retry with power cycle.
     * Stage 6 — AT+CREG? : network registration. Max resp 300 ms.
     *                       STRICT: only accept stat=1 (home) or stat=5 (roaming).
     *                       New modules may take 60–180 s to register.
     *                       Retry up to MODEM_CREG_WAIT_RETRIES times with
     *                       MODEM_CREG_POLL_MS between attempts.
     * Stage 7 — AT+CPIN? : SIM card. Max resp 5000 ms.
     *                       Accept "+CPIN: READY" only. Any CME ERROR = FAIL.
     */
    typedef struct {
        const char *cmd;
        const char *expected;
        const char *label;
        uint32_t    timeout_ms;
        bool        fail_fast;  /* true = abort sequence on first failure */
    } stage_def_t;

    static const stage_def_t stages[] = {
        { "AT",      "OK",      "AT basic",     600,   false },
        { "ATI",     "Quectel", "ATI ident",    600,   false },
        { "ATE0",     "OK", "ATE0 Echo mode off",    600,   false },
        { "AT+CGMI", "Quectel", "AT+CGMI mfr",  600,   false },
        { "AT+GSN",  "OK",      "AT+GSN IMEI",  600,   false },
        { "AT+CSQ",  "OK",      "AT+CSQ signal",600,   true  },       
        { "AT+CPIN?","READY",   "AT+CPIN? SIM", 5000,  true  },
        { "AT+CREG?","OK",      "AT+CREG? reg", 600,   true  },
    };
    const int NUM_STAGES = (int)(sizeof(stages) / sizeof(stages[0]));

    char ati_line[48]  = {0};
    char cgmi_line[32] = {0};
    char imei_str[20]  = {0};
    char csq_str[32]   = {0};
    char creg_str[32]  = {0};
    //char raw_buf[BOARD_MODEM_BUF_SIZE];
    static char raw_buf[BOARD_MODEM_BUF_SIZE];  /* static = BSS, not stack */

    printf("  [MODEM] Powering on modem...\r\n");
    poweron_modem();

    for (int cycle = 1; cycle <= MODEM_POWERCYCLE_RETRIES; cycle++) {
	    /* Quick silence check — if modem gives no response at all,
	     * it has browned out. Don't waste retry budget on dead AT sends. */
	    bool modem_alive = false;
	    for (int ping = 0; ping < 3 && !modem_alive; ping++) {
	        memset(raw_buf, 0, sizeof(raw_buf));
	        if (send_at_command("AT", "OK", 2, 800, raw_buf, sizeof(raw_buf))) {
	            modem_alive = true;
	        } else {
	            vTaskDelay(pdMS_TO_TICKS(500));
	        }
	    }
	    if (!modem_alive) {
	        printf("  [MODEM] Modem silent (brownout?) on cycle %d — power cycling\r\n",
	               cycle);
	        if (cycle < MODEM_POWERCYCLE_RETRIES) {
	            modem_power_cycle(cycle);
	        }
	        continue;  /* restart the stage loop after power cycle */
	    }

        bool all_stages_ok = true;

        for (int s = 0; s < NUM_STAGES; s++) {

            bool stage_ok = false;
            memset(raw_buf, 0, sizeof(raw_buf));

            /* --- Stage 5: AT+CSQ with rssi=99 invalid check --- */
            if (s == 5) {
                for (int attempt = 1;
                		//attempt <= MODEM_AT_RETRIES && !stage_ok;
                     attempt <= 8 && !stage_ok;
                     attempt++) {
                    printf("  [MODEM] Cycle %d  Stage %d (%s) attempt %d...\r\n",
                           cycle, s + 1, stages[s].label, attempt);
/*                    bool ok = send_at_command(stages[s].cmd,
                                              stages[s].expected,
                                              2,
                                              stages[s].timeout_ms,
                                              raw_buf,
                                              sizeof(raw_buf));*/
                    bool ok = send_at_command(stages[s].cmd,
                                              stages[s].expected,
                                              40,
                                              stages[s].timeout_ms,
                                              raw_buf,
                                              sizeof(raw_buf));
                    if (ok) {
                        /* Parse rssi value — rssi=99 means "not detectable" */
                        int rssi = 99, ber = 99;
						char *csq_ptr = strstr(raw_buf, "+CSQ:");
						if (csq_ptr && sscanf(csq_ptr, "+CSQ: %d,%d", &rssi, &ber) == 2) {
                            if (rssi == 99) {
                                ESP_LOGW(TAG, "AT+CSQ: rssi=99 (no signal) — retrying");
                                vTaskDelay(pdMS_TO_TICKS(2000));
                                /* Don't set stage_ok — force retry */
                            } else {
                                /* Valid signal: 0–31 or 100–191 (LTE extended) */
                                int dbm = (rssi <= 31) ? (-113 + 2 * rssi) :
                                          (-113 + 2 * (rssi - 100));
                                snprintf(csq_str, sizeof(csq_str),
                                         "rssi=%d(%ddBm)", rssi, dbm);
                                stage_ok = true;
                            }
                        } else {
                            /* Could not parse — treat as invalid */
                            ESP_LOGW(TAG, "AT+CSQ: parse failed, raw: %s", raw_buf);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                    }
                }
                if (!stage_ok) {
                    snprintf(r->detail, sizeof(r->detail),
                             "AT+CSQ failed/invalid signal after retries — cycle %d/%d",
                             cycle, MODEM_POWERCYCLE_RETRIES);
                    all_stages_ok = false;
                    break;
                }
                continue;
            }

            /* --- Stage 7: AT+CREG? — poll until registered (1 or 5) --- */
            if (s == 7) {
                const int   CREG_MAX_POLLS  = 60;   /* 60 × 3 s = 180 s max */
                const int   CREG_POLL_MS    = 3000;
                bool creg_registered = false;

                printf("  [MODEM] Waiting for network registration (up to 180s)...\r\n");

                for (int poll = 0; poll < CREG_MAX_POLLS && !creg_registered; poll++) {
					
				    /* Every 10 polls (~30s), probe the modem is still responding */
				    if (poll > 0 && (poll % 10) == 0) {
				        if (!send_at_command("AT", "OK", 2, 800, NULL, 0)) {
				            printf("  [CREG] Modem went silent during registration wait — aborting\r\n");
				            all_stages_ok = false;
				            goto stage_loop_break;
				        }
				    }					
				    memset(raw_buf, 0, sizeof(raw_buf));
                    bool ok = send_at_command("AT+CREG?", "+CREG:",
                                              2, 600,
                                              raw_buf, sizeof(raw_buf));
                    if (ok) {
                        int n = -1, stat = -1;
                        /* Parse "+CREG: <n>,<stat>" or "+CREG: <stat>" */
                        char *p = strstr(raw_buf, "+CREG:");
                        if (p) {
                            if (sscanf(p, "+CREG: %d,%d", &n, &stat) == 2) {
                                /* two-param form */
                            } else if (sscanf(p, "+CREG: %d", &n) == 1) {
                                stat = n; n = 0;
                            }
                        }
                        printf("  [CREG] poll %d: n=%d stat=%d\r\n",
                               poll + 1, n, stat);
                        if (stat == 1 || stat == 5) {
                            /* 1 = registered home, 5 = roaming */
                            snprintf(creg_str, sizeof(creg_str),
                                     "stat=%d(%s)", stat,
                                     stat == 1 ? "home" : "roaming");
                            creg_registered = true;
                        }
                    }
                    if (!creg_registered) {
                        vTaskDelay(pdMS_TO_TICKS(CREG_POLL_MS));
                    }
                }

                if (!creg_registered) {
                    snprintf(r->detail, sizeof(r->detail),
                             "AT+CREG: not registered after 180s — cycle %d/%d",
                             cycle, MODEM_POWERCYCLE_RETRIES);
                    all_stages_ok = false;
                    break;
                }
                stage_ok = true;
                continue;
            }

            /* --- Stage 6: AT+CPIN? — SIM present --- */
            if (s == 6) {
                for (int attempt = 1;
                     attempt <= MODEM_AT_RETRIES && !stage_ok;
                     attempt++) {
                    printf("  [MODEM] Cycle %d  Stage %d (%s) attempt %d...\r\n",
                           cycle, s + 1, stages[s].label, attempt);
                    bool ok = send_at_command("AT+CPIN?", "+CPIN:",
                                              2, 5000,
                                              raw_buf, sizeof(raw_buf));
                    if (ok) {
                        if (strstr(raw_buf, "+CPIN: READY")) {
                            stage_ok = true;
                        } else if (strstr(raw_buf, "+CME ERROR") ||
                                   strstr(raw_buf, "ERROR")) {
                            /* CME error = SIM not present or not responding */
                            snprintf(r->detail, sizeof(r->detail),
                                     "AT+CPIN: CME ERROR — SIM absent/fault");
                            all_stages_ok = false;
                            goto stage_loop_break;
                        } else {
                            /* SIM present but needs PIN or PUK */
                            snprintf(r->detail, sizeof(r->detail),
                                     "AT+CPIN: SIM locked");
                            all_stages_ok = false;
                            goto stage_loop_break;
                        }
                    }
                }
                if (!stage_ok) {
                    snprintf(r->detail, sizeof(r->detail),
                             "AT+CPIN? failed after retries — cycle %d/%d",
                             cycle, MODEM_POWERCYCLE_RETRIES);
                    all_stages_ok = false;
                    break;
                }
                continue;
            }

            /* --- General stages (0–4) --- */
            for (int attempt = 1;
                 attempt <= MODEM_AT_RETRIES && !stage_ok;
                 attempt++) {
                printf("  [MODEM] Cycle %d/%d  Stage %d/%d (%s) attempt %d/%d\r\n",
                       cycle, MODEM_POWERCYCLE_RETRIES,
                       s + 1, NUM_STAGES, stages[s].label,
                       attempt, MODEM_AT_RETRIES);

                memset(raw_buf, 0, sizeof(raw_buf));
                bool ok = send_at_command(stages[s].cmd,
                                          stages[s].expected,
                                          6,
                                          stages[s].timeout_ms,
                                          raw_buf, sizeof(raw_buf));
                if (ok) {
                    stage_ok = true;
                    if (s == 1) {
                        /* ATI — extract objectID (second line) */
                        const char *p = raw_buf;
                        int line_num = 0;
                        while (*p) {
                            while (*p == '\r' || *p == '\n') p++;
                            if (!*p) break;
                            const char *ls = p;
                            while (*p && *p != '\r' && *p != '\n') p++;
                            size_t ll = (size_t)(p - ls);
                            line_num++;
                            if (line_num == 2 && ll > 0 &&
                                ll < sizeof(ati_line) - 1) {
                                memcpy(ati_line, ls, ll);
                                ati_line[ll] = '\0';
                                break;
                            }
                        }
                    } else if (s == 3) {
                        strip_response(raw_buf, cgmi_line, sizeof(cgmi_line));
                    } else if (s == 4) {
                        if (!extract_imei(raw_buf, imei_str)) {
                            stage_ok = false;
                        }
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }
            }

            if (!stage_ok) {
                snprintf(r->detail, sizeof(r->detail),
                         "Stage %d (%s) failed — cycle %d/%d",
                         s + 1, stages[s].label,
                         cycle, MODEM_POWERCYCLE_RETRIES);
                all_stages_ok = false;
                break;
            }
        } /* stage loop */

stage_loop_break:
        if (all_stages_ok) {
			snprintf(r->detail, sizeof(r->detail),
			         "%.16s IMEI:%.15s %s %s SIM:OK",
			         ati_line[0]  ? ati_line  : "?",
			         imei_str[0]  ? imei_str  : "?",
			         csq_str[0]   ? csq_str   : "?",
			         creg_str[0]  ? creg_str  : "?");
            r->passed = true;
            return;
        }

        if (cycle < MODEM_POWERCYCLE_RETRIES) {
            modem_power_cycle(cycle);
        }

    } /* cycle loop */

    ESP_LOGE(TAG, "Modem test FAILED after %d power cycles",
             MODEM_POWERCYCLE_RETRIES);
             
    while (1) {
    	vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/* -----------------------------------------------------------------
 * TEST 3 — RS-485 MUX transmit on all three battery channels
 * ----------------------------------------------------------------- */

/**
 * @brief Verify the MC74HC4052ADG MUX can select each battery bus and
 *        transmit data.  Tester observes TX activity LEDs on each bus.
 *
 * Sends RS485_HELLO_REPEATS × "Hello World\r\n" per battery channel.
 * The MUX select pins are driven by bms_rs485_switch_to_battery() which
 * sets GPIO8 (SEL_A) and GPIO9 (SEL_B) as documented in board_config.h.
 *
 * Channel mapping (board_config.h §RS-485 MUX):
 *   Battery 0: A=0, B=0
 *   Battery 1: A=1, B=0
 *   Battery 2: A=0, B=1
 *
 * @param[out] r  Test result record.
 */
static void __attribute__((unused)) test_rs485_mux(factory_test_result_t *r)
{
    r->name   = "RS-485 MUX TX (2 batteries)";
    r->passed = false;

    const char  *msg     = "Hello World\r\n";
    const size_t msg_len = strlen(msg);

    for (int batt = 0; batt < BMS_BATTERY_COUNT; batt++) {
        esp_err_t ret = bms_rs485_switch_to_battery(
            (bms_battery_id_t)batt);
        if (ret != ESP_OK) {
            snprintf(r->detail, sizeof(r->detail),
                     "MUX switch failed for Battery %d (%s)",
                     batt + 1, esp_err_to_name(ret));
            bms_rs485_disable_all();
            return;
        }

        printf("  RS-485 Battery %d: transmitting %d x 'Hello World'...\r\n",
               batt + 1, RS485_HELLO_REPEATS);

        for (int i = 0; i < RS485_HELLO_REPEATS; i++) {
            uart_write_bytes(BOARD_RS485_UART_NUM, msg, msg_len);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        bms_rs485_disable_battery((bms_battery_id_t)batt);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    snprintf(r->detail, sizeof(r->detail),
             "Tx complete on 3 buses — verify TX LEDs lit per bus");
    r->passed = true;
}

/* -----------------------------------------------------------------
 * TEST 4 — RGB LED visual confirmation
 * ----------------------------------------------------------------- */

/**
 * @brief Cycle RGB LEDs through RED → GREEN → BLUE → WHITE.
 *
 * Each colour is held for approximately 200 ms.  The full sequence
 * completes within LED_USER_TIMEOUT_MS.  The tester visually confirms
 * each LED illuminates in turn.  This test always auto-passes because
 * GPIO write success cannot be verified in software alone — the tester
 * must record any LED that failed to illuminate.
 *
 * Uses rgb_led_set_raw() to bypass the alert state machine so the
 * LED task does not interfere with the test sequence.
 *
 * @param[out] r  Test result record.
 */
static void test_rgb_leds(factory_test_result_t *r)
{
    r->name   = "RGB LEDs (visual — RED/GREEN/BLUE/WHITE)";
    r->passed = true;   /* Auto-pass — visual check only */

    static const struct {
        const char *colour;
        bool red;
        bool green;
        bool blue;
    } sequence[] = {
        { "RED",   true,  false, false },
        { "GREEN", false, true,  false },
        { "BLUE",  false, false, true  },
        { "WHITE", true,  true,  true  },
    };
    const int NUM_COLOURS = (int)(sizeof(sequence) / sizeof(sequence[0]));

    TickType_t start   = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(LED_USER_TIMEOUT_MS);

    for (int i = 0; i < NUM_COLOURS; i++) {
        if ((xTaskGetTickCount() - start) >= timeout) {
            break;
        }
        printf("  LED: %s ON\r\n", sequence[i].colour);
        rgb_led_set_raw(sequence[i].red,
                        sequence[i].green,
                        sequence[i].blue);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Restore Red-only (power indicator state) */
    rgb_led_set_raw(true, false, false);

    snprintf(r->detail, sizeof(r->detail),
             "Cycled RED/GREEN/BLUE/WHITE — confirm each lit");
}

/* -----------------------------------------------------------------
 * TEST 5 — ADC sanity check (voltage / current / temperature)
 * ----------------------------------------------------------------- */

/**
 * @brief Read all three ADC channels and verify values are in range.
 *
 * Pass criteria:
 *   Voltage    : < 1.0 V  (no trike connected, acceptable)
 *                OR  40.0 – 100.0 V  (trike pack connected)
 *   Current    : -10.0 A to +10.0 A  (near-zero at rest)
 *   Temperature: -40.0 °C to +120.0 °C
 *
 * The -99.0 °C sentinel from trike_sensors_read_temperature() (NTC ADC
 * out of range) causes the temperature check to fail, flagging an open
 * or shorted NTC circuit.
 *
 * @param[out] r  Test result record.
 */
static void test_adcs(factory_test_result_t *r)
{
    r->name   = "ADC (voltage / current / temperature)";
    r->passed = false;

    trike_sensor_data_t sensors;
    esp_err_t ret = trike_sensors_read_all(&sensors);

    if (!sensors.valid) {
        snprintf(r->detail, sizeof(r->detail),
                 "ADC read failed (%s)", esp_err_to_name(ret));
        return;
    }

    printf("  ADC: V=%.2fV  I=%.3fA  T=%.1f deg C\r\n",
           sensors.voltage_v, sensors.current_a, sensors.temperature_c);

    /* Voltage: 0 V (no pack) or 40–100 V (pack connected) */
    bool volt_ok = (sensors.voltage_v < 1.0f) ||
                   (sensors.voltage_v >= 40.0f &&
                    sensors.voltage_v <= 100.0f);

    /* Current: within ±10 A at rest */
    bool curr_ok = (sensors.current_a >= -10.0f &&
                    sensors.current_a <=  10.0f);

    /* Temperature: -40 to +120 °C (rejects -99 sentinel) */
    bool temp_ok = (sensors.temperature_c > -40.0f &&
                    sensors.temperature_c <  120.0f);

    if (!volt_ok) {
        snprintf(r->detail, sizeof(r->detail),
                 "Voltage %.2fV out of range (expect <1V or 40-100V)",
                 sensors.voltage_v);
        return;
    }
    if (!curr_ok) {
        snprintf(r->detail, sizeof(r->detail),
                 "Current %.3fA out of range at rest (expect +-10A max)",
                 sensors.current_a);
        return;
    }
    if (!temp_ok) {
        snprintf(r->detail, sizeof(r->detail),
                 "Temperature %.1f deg C out of range (NTC open/shorted?)",
                 sensors.temperature_c);
        return;
    }

    snprintf(r->detail, sizeof(r->detail),
             "V=%.2fV  I=%.3fA  T=%.1f deg C",
             sensors.voltage_v, sensors.current_a, sensors.temperature_c);
    r->passed = true;
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

esp_err_t factory_test_run(void)
{
    printf(TEST_SEPARATOR);
    printf("  KSC TCU V1.1 Factory Self-Test\r\n");
    printf("  HW: %-8s  FW: %-8s  Built: %s %s\r\n",
           HW_VERSION_STR, FW_VERSION_STR,
           FW_BUILD_DATE, FW_BUILD_TIME);
    printf(TEST_SEPARATOR);
    printf("\r\n");

    factory_test_result_t results[6];
    int total  = 0;
    int passed = 0;

    test_lis3dhtr  (&results[total++]);
    test_modem     (&results[total++]);
    //test_rs485_mux (&results[total++]);
    test_rgb_leds  (&results[total++]);
    test_adcs      (&results[total++]);
    //test_gps_init  (&results[total++]);

    printf("\r\n--- Test Results ---\r\n");
    for (int i = 0; i < total; i++) {
        print_result(&results[i]);
        if (results[i].passed) passed++;
    }

    printf(TEST_SEPARATOR);
    if (passed == total) {
        printf("  Result: %d/%d tests PASSED\r\n", passed, total);
        printf(TEST_SEPARATOR);
        printf("\r\n");
        return ESP_OK;
    }

    printf("  Result: %d/%d tests PASSED  (%d FAILED)\r\n",
           passed, total, total - passed);
    printf(TEST_SEPARATOR);

	/*    
     * REQ 5: Halt firmware execution until all tests pass.
     * Blink RED LED at 500 ms to indicate blocked state.
     * Tester must power-cycle to retry after fixing the fault.
     
    printf("\r\n  *** FIRMWARE HALTED — Fix failures and power-cycle ***\r\n\r\n");
    while (1) {
        rgb_led_set_raw(true,  false, false);
        vTaskDelay(pdMS_TO_TICKS(250));
        rgb_led_set_raw(false, false, false);
        vTaskDelay(pdMS_TO_TICKS(250));
    }*/

    return ESP_FAIL; /* Never reached */
}