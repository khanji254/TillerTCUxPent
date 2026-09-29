/*
 * @file factory_test.h
 * @brief Factory / Startup Peripheral Test Suite for KSC TCU V1.1
 *
 * Runs a sequence of peripheral self-tests at boot before entering normal
 * operation.  Results are reported via USB CDC (UART0 in console mode)
 * as human-readable PASS/FAIL lines.
 *
 * Tests performed:
 *   1. LIS3DHTR I2C communication and WHO_AM_I verification
 *   2. Quectel modem — four-stage AT command verification:
 *        Stage 1: AT          → "OK"          (basic UART comms)
 *        Stage 2: ATI         → "Quectel"     (product identification)
 *        Stage 3: AT+CGMI     → "Quectel"     (manufacturer identification)
 *        Stage 4: AT+GSN      → 15-digit IMEI (IMEI readout)
 *      Each stage is attempted up to MODEM_AT_RETRIES times.  On failure
 *      the modem is power-cycled (OFF → 1 s wait → ON) and the stage is
 *      retried up to MODEM_POWERCYCLE_RETRIES full cycles before the
 *      entire modem test is declared FAILED.
 *   3. RS-485 MUX transmit test (5× "Hello World" on each battery bus)
 *   4. RGB LED visual test (cycles RED/GREEN/BLUE/WHITE, 200 ms each,
 *      auto-completes within LED_USER_TIMEOUT_MS)
 *   5. ADC sanity check (voltage, current, temperature rails in range)
 *
 * Modem AT command reference (Quectel EG915N AT Commands Manual v1.2):
 *   AT     — Basic attention command. Response: OK            (§ implicit)
 *   ATI    — Product identification. Response: Quectel\r\n
 *             <objectID>\r\nRevision: <revision>\r\nOK       (§ 2.1)
 *   AT+CGMI— Manufacturer identification. Response: Quectel\r\nOK (§ 2.5)
 *   AT+GSN — IMEI request. Response: <15-digit IMEI>\r\nOK   (§ 2.8)
 *
 * @author  Mary Mbugua
 * @date    2026-04-02*/
 

#ifndef FACTORY_TEST_H_
#define FACTORY_TEST_H_

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * MODEM TEST CONFIGURATION
 * ========================================================================= */

/**
 * Number of AT command attempts per stage before triggering a power cycle.
 * Each attempt uses send_at_command() which has its own internal retry loop
 * (RETRIES = 6 in Quectel_mqtt.h); this constant controls how many times
 * the entire per-stage attempt is made before the modem is power-cycled.
 */
#define MODEM_AT_RETRIES          2

/**
 * Number of full power-cycle attempts before declaring modem FAILED.
 * Total maximum attempts per stage = MODEM_AT_RETRIES × MODEM_POWERCYCLE_RETRIES.
 */
#define MODEM_POWERCYCLE_RETRIES  3

/** Delay between modem power-off and power-on during a cycle (ms). */
#define MODEM_POWERCYCLE_WAIT_MS  1000

/** Maximum number of AT+CREG? polls before declaring registration FAILED.
 *  60 polls × 3 s = 180 s max — per Quectel datasheet recommendation for
 *  new modules registering to a network for the first time. */
#define MODEM_CREG_MAX_POLLS      60

/** Poll interval between AT+CREG? attempts (ms) */
#define MODEM_CREG_POLL_MS        3000

/** Increase detail buffer for extended test results */
/* Note: factory_test_result_t.detail is already char detail[128] — no change needed */

/* =========================================================================
 * RESULT RECORD
 * ========================================================================= */

/**
 * @brief Result record for a single test.
 */
typedef struct {
    const char *name;       /**< Test name string (points to literal)        */
    bool        passed;     /**< true = PASSED, false = FAILED               */
    char        detail[128]; /**< Detail message — values on pass, error on fail */
} factory_test_result_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Run the complete factory test suite.
 *
 * Blocks until all tests complete.  Writes PASS/FAIL output to the USB
 * serial console.  The modem test includes power-cycle retry logic and
 * four-stage AT command verification (AT / ATI / AT+CGMI / AT+GSN).
 * LED visual tests auto-complete within LED_USER_TIMEOUT_MS so the suite
 * can run unattended on a bench.
 *
 * @return ESP_OK if all tests passed, ESP_FAIL if any test failed.
 *         The caller may choose to halt or continue on failure.
 */
esp_err_t factory_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* FACTORY_TEST_H_ */