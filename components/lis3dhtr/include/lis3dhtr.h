/**
 * @file lis3dhtr.h
 * @brief LIS3DHTR Accelerometer Driver for KSC TCU V1.1 (ESP32-S3, I2C)
 *
 * Provides full accelerometer functionality relevant to a three-wheeled
 * electric motorcycle (trike):
 *
 *   - Motion detection (vehicle moving vs. stationary)
 *   - Tilt / roll / pitch angle estimation
 *   - Impact / shock event detection via hardware interrupt on INT1
 *   - Free-fall detection (e.g. trike tipped over)
 *   - Continuous raw acceleration sampling
 *
 * Hardware wiring (TCU V1.1 board):
 *   SCL  → GPIO47  (4.7 kΩ external pull-up to 3.3 V)
 *   SDA  → GPIO48  (4.7 kΩ external pull-up to 3.3 V)
 *   SA0  → GND     → I2C address = 0x18
 *   CS   → 3.3 V via 4.7 kΩ (I2C mode selected)
 *   INT1 → GPIO7   (hardware interrupt — click and activity detection)
 *   INT2 → not connected
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef LIS3DHTR_H_
#define LIS3DHTR_H_

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * DEVICE CONSTANTS
 * ========================================================================= */

/** I2C address with SA0 tied to GND */
#define LIS3DHTR_I2C_ADDR           0x18

/** Expected value of WHO_AM_I register — confirms device identity */
#define LIS3DHTR_WHO_AM_I_VAL       0x33

/* =========================================================================
 * REGISTER MAP (all registers used by this driver)
 * ========================================================================= */

#define LIS3DHTR_REG_WHO_AM_I       0x0F
#define LIS3DHTR_REG_CTRL_REG1      0x20  /**< ODR, power mode, axes        */
#define LIS3DHTR_REG_CTRL_REG2      0x21  /**< High-pass filter             */
#define LIS3DHTR_REG_CTRL_REG3      0x22  /**< INT1 pin function selection  */
#define LIS3DHTR_REG_CTRL_REG4      0x23  /**< Full-scale, HR mode, BDU     */
#define LIS3DHTR_REG_CTRL_REG5      0x24  /**< FIFO enable, latch INT       */
#define LIS3DHTR_REG_CTRL_REG6      0x25  /**< INT2 pad control             */
#define LIS3DHTR_REG_REFERENCE      0x26  /**< Reference/DC offset          */
#define LIS3DHTR_REG_STATUS_REG     0x27  /**< Data-ready flags             */
#define LIS3DHTR_REG_OUT_X_L        0x28  /**< X low byte (multi-read base) */
#define LIS3DHTR_REG_OUT_X_H        0x29
#define LIS3DHTR_REG_OUT_Y_L        0x2A
#define LIS3DHTR_REG_OUT_Y_H        0x2B
#define LIS3DHTR_REG_OUT_Z_L        0x2C
#define LIS3DHTR_REG_OUT_Z_H        0x2D
#define LIS3DHTR_REG_INT1_CFG       0x30  /**< INT1 axis/dir enable         */
#define LIS3DHTR_REG_INT1_SRC       0x31  /**< INT1 source (read to clear)  */
#define LIS3DHTR_REG_INT1_THS       0x32  /**< INT1 threshold               */
#define LIS3DHTR_REG_INT1_DURATION  0x33  /**< INT1 minimum duration        */
#define LIS3DHTR_REG_CLICK_CFG      0x38  /**< Click axis enable            */
#define LIS3DHTR_REG_CLICK_SRC      0x39  /**< Click source (read to clear) */
#define LIS3DHTR_REG_CLICK_THS      0x3A  /**< Click threshold              */
#define LIS3DHTR_REG_TIME_LIMIT     0x3B  /**< Click time limit             */
#define LIS3DHTR_REG_TIME_LATENCY   0x3C  /**< Double-click latency         */
#define LIS3DHTR_REG_TIME_WINDOW    0x3D  /**< Double-click window          */

/* =========================================================================
 * FIFO REGISTERS (datasheet §7, Table 21)
 * ========================================================================= */
#define LIS3DHTR_REG_FIFO_CTRL     0x2E  /**< FIFO_CTRL_REG — mode & watermark */
#define LIS3DHTR_REG_FIFO_SRC      0x2F  /**< FIFO_SRC_REG  — status & count   */

/**
 * FIFO_CTRL_REG bit patterns (datasheet §8.19, Table 50)
 *   FM[1:0] in bits [7:6]:
 *     00 = Bypass  (FIFO disabled, reset default)
 *     01 = FIFO mode  (fills to 32 then stops)
 *     10 = Stream mode  (fills to 32 then overwrites oldest) ← use this
 *     11 = Stream-to-FIFO
 */
#define LIS3DHTR_FIFO_BYPASS       0x00   /**< FM=00 — FIFO off (reset)      */
#define LIS3DHTR_FIFO_STREAM       0x80   /**< FM=10 — stream mode           */

/**
 * FIFO_SRC_REG bit masks (datasheet §8.20, Table 52)
 */
#define LIS3DHTR_FIFO_SRC_WTM      0x80   /**< Watermark exceeded            */
#define LIS3DHTR_FIFO_SRC_OVRN    0x40   /**< Overrun (expected in stream)  */
#define LIS3DHTR_FIFO_SRC_EMPTY    0x20   /**< FIFO empty                    */
#define LIS3DHTR_FIFO_SRC_FSS_MASK 0x1F   /**< FSS[4:0] — unread count       */

/**
 * Hardware FIFO depth — fixed in silicon (datasheet §3.6, §5.1)
 * At 200 Hz ODR: 32 samples = 160 ms window
 */
#define LIS3DHTR_FIFO_DEPTH        32

/**
 * Accumulator ring-buffer sizes
 *   PER_SECOND_BUF: one entry per diag_log_task tick (1 Hz)
 *   At 60 s publish interval → 60 entries per publish window
 */
//#define LIS3DHTR_ACC_BUF_SIZE      60    /**< One entry per second, 60s window */

// CHANGE: was 60 (one entry per second at 25Hz)
// At 200Hz: 6–7 drains/second × 60 seconds = ~400 windows per publish interval
#define LIS3DHTR_ACC_BUF_SIZE      400

/* =========================================================================
 * FIFO WINDOW STATISTICS (computed per 1-second drain)
 * ========================================================================= */

/**
 * @brief Statistics from one FIFO drain (~32 samples, ~160 ms window).
 *
 * Called once per second by diag_log_task.
 * Stored in a 60-slot ring buffer; published once per minute.
 */
typedef struct {
    float x_rms_mg;         /**< RMS of X samples in this window           */
    float y_rms_mg;         /**< RMS of Y samples in this window           */
    float y_rms_hpf_mg;		/**< RMS of Y samples in this window with HPF   */
    float z_rms_mg;         /**< RMS of Z samples in this window           */
    float x_mean_mg;         /**< Mean of X samples in this window           */
    float y_mean_mg;         /**< Mean of Y samples in this window           */
    float z_mean_mg;         /**< Mean of Z samples in this window           */
    float x_max_abs_mg;     /**< Largest |X| sample in window              */
    float y_max_abs_mg;     /**< Largest |Y| sample in window              */
    float z_max_abs_mg;     /**< Largest |Z| sample in window              */
    float x_max_signed_mg;  /**< Signed X with largest |X| (preserves dir) */
    float y_max_signed_mg;  /**< Signed Y with largest |Y|                 */
    float z_max_signed_mg;  /**< Signed Z with largest |Z|                 */
    uint8_t n_samples;      /**< Samples actually read (0–32)              */    
} lis3dhtr_window_t;

/* =========================================================================
 * PUBLISH-INTERVAL STATISTICS (computed from 60 stored windows)
 * ========================================================================= */

/**
 * @brief One-minute summary ready to publish to ThingsBoard.
 *
 * Retrieved by create_metadata_payload() at each 60-second publish tick.
 * All values are gravity-free (HPF active in production mode).
 */
typedef struct {
    /* Per-axis RMS-of-RMS across 60 one-second windows */
    float x_rms60_mg;          /**< RMS of 60 per-second X RMS values      */
    float y_rms60_mg;          /**< RMS of 60 per-second Y RMS values       */
    float y_rms60_hpf_mg;      /**< RMS of 60 per-second Y RMS values with HPF */
    float z_rms60_mg;          /**< RMS of 60 per-second Z RMS values       */

    /* Vector magnitude of the three RMS-of-RMS values */
    float magnitude_rms60_mg;  /**< sqrt(x²+y²+z²) of rms60 triplet        */

    /* Per-axis peak across all 60 windows */
    float x_peak_abs_mg;       /**< Max |X| across all 60 windows          */
    float y_peak_abs_mg;       /**< Max |Y| across all 60 windows          */
    float z_peak_abs_mg;       /**< Max |Z| across all 60 windows          */
    float x_peak_signed_mg;    /**< Signed X for max |X| event             */
    float y_peak_signed_mg;    /**< Signed Y for max |Y| event             */
    float z_peak_signed_mg;    /**< Signed Z for max |Z| event             */

    uint16_t windows_captured;  /**< How many 1-second windows were stored  */
} lis3dhtr_publish_stats_t;

/* =========================================================================
 * CTRL_REG1 bit values
 * ========================================================================= */

/** 100 Hz ODR, normal mode, X/Y/Z enabled: 0x57 = 0101_0111 */
#define LIS3DHTR_CTRL1_100HZ_NORMAL  0x57

/** 25 Hz ODR, normal mode, X/Y/Z enabled: 0x37 = 0011_0111
 *  Sufficient for motion/tilt on a vehicle; saves ~75% current vs 100 Hz 
 * 	25 Hz ODR, normal power, X/Y/Z enabled: 0x37 = 0011_0111
 *  ODR[3:0]=0011 → 25 Hz per datasheet Table 31.
 *  FIFO fills in 32/25 = 1280 ms — diag_log_task drains every 1s
 *  so ~25 samples per drain, OVRN never fires. */
#define LIS3DHTR_CTRL1_25HZ_NORMAL   0x37

/** 200 Hz ODR, normal power, X/Y/Z enabled: 0x67 = 0110_0111 */
#define LIS3DHTR_CTRL1_200HZ_NORMAL  0x67

/** 50 Hz ODR: 0x47 = 0100_0111, ODR[3:0]=0100 per Table 31 */
#define LIS3DHTR_CTRL1_50HZ_NORMAL   0x47

/*
 * INT1_THS: Motion detection threshold
 *
 * Per LIS3DH datasheet Table 59:
 *   1 LSB = 16 mg  @ FS = ±2g
 *   1 LSB = 32 mg  @ FS = ±4g
 *   1 LSB = 62 mg  @ FS = ±8g
 *   1 LSB = 186 mg @ FS = ±16g   ← active
 *
 * Register: 8 bits, bit7=0 (unused), THS[6:0] = 7-bit value (0–127).
 *
 * With HP_IA1=1 (HPF on INT1), gravity is removed before comparison.
 * Threshold represents pure dynamic acceleration.
 *
 * At ±16g, 1 LSB = 186 mg:
 *   5  LSB =  930 mg ≈ 0.9g  (gentle motion, risk of vibration false trigger)
 *   8  LSB = 1488 mg ≈ 1.5g  (recommended — detects clear vehicle movement)
 *   16 LSB = 2976 mg ≈ 3.0g  (only strong acceleration triggers)
 *   26 LSB = 4836 mg ≈ 4.8g  (previous value — far too high for motion detect)
 *
 * Recommendation: use 8 LSB (1.5g) for trike motion detection.
 */
#define LIS3DHTR_MOTION_THS_DEFAULT  8    /* 8 × 186 mg = 1488 mg ≈ 1.5g  */

/*
 * INT1_DURATION is an 8-bit register (values 0-127).
 * At 200 Hz ODR: 1 measurement cycle = 1/200 s = 5 ms per LSB.
 * Required debounce: 200 ms / 5 ms per LSB = 40 LSB.
 */
//#define LIS3DHTR_MOTION_DUR_DEFAULT  40

/* At 50Hz ODR: Debounce: 1 LSB = 1/50 = 20 ms. 10 LSB × 20 ms = 200 ms */
//#define LIS3DHTR_MOTION_DUR_DEFAULT  10

/* Update debounce for 25 Hz: 1 LSB = 1/25 = 40 ms.
 * 5 LSB × 40 ms = 200 ms debounce — same physical duration as before. */
//#define LIS3DHTR_MOTION_DUR_DEFAULT   		/*At 25Hz*/
#define LIS3DHTR_MOTION_DUR_DEFAULT   40		/*At 200Hz*/	

/* =========================================================================
 * IMPACT DETECTION THRESHOLDS — all full-scale ranges at 200 Hz ODR
 * =========================================================================
 * CLICK_THS register: 7-bit value, 1 LSB = FS/128.
 * LIR_Click (bit 7) = 1 to latch the interrupt.
 * TIME_LIMIT: max click pulse duration in ODR steps (1 step = 5 ms at 200 Hz).
 *
 * Automotive impact target: ≥5 g instantaneous shock (crash/severe pothole).
 *
 *  FS=±2g : 1 LSB=15.625mg, 5g=320LSB → exceeds 7-bit max(127)=1.98g.
 *            At ±2g the max settable threshold is 127 LSB (~2g). A true
 *            5g crash saturates the ADC so impact is still detected.
 *  FS=±4g : 1 LSB=31.25mg,  5g=160LSB → exceeds max. Use 127=~3.97g.
 *  FS=±8g : 1 LSB=62.5mg,   5g= 80LSB ✓  Use 80.
 *  FS=±16g: 1 LSB=125mg,    5g= 40LSB ✓  Use 40.  ← ACTIVE (16g mode)
 *
 * TIME_LIMIT=2 → 10 ms max pulse duration. Genuine impacts are short
 * transients; road vibration sustains longer and won't false-trigger.
 * ========================================================================= */

#define LIS3DHTR_IMPACT_THS_2G    127   /**< ~1.98 g (max at ±2g)           */
#define LIS3DHTR_IMPACT_THS_4G    127   /**< ~3.97 g (max at ±4g)           */
#define LIS3DHTR_IMPACT_THS_8G     80   /**< ~5.0 g  at ±8g                 */

#define LIS3DHTR_IMPACT_THS_16G    40   /**< ~5 g  at ±16g  ← current     */
//#define LIS3DHTR_IMPACT_TIME_LIMIT  2   /**< 2 × 5 ms = 10 ms max pulse at 200Hz     */

/* Click: 1 LSB = 20 ms. TIME_LIMIT=1 → 20 ms max pulse */
//#define LIS3DHTR_IMPACT_TIME_LIMIT    1  /**< at 50Hz						*/

/* TIME_LIMIT at 25 Hz: 1 LSB = 40 ms minimum.
 * 1 LSB = 40 ms max click pulse — accepts genuine short impacts,
 * still filters sustained vibration (which lasts >40 ms). */
//#define LIS3DHTR_IMPACT_TIME_LIMIT    1		/*At 25Hz*/
#define LIS3DHTR_IMPACT_TIME_LIMIT    2			/*At 200Hz*/

/* =========================================================================
 * CTRL_REG4 values for each full-scale range (HR=1, BDU=1)
 * ========================================================================= */
#define LIS3DHTR_CTRL4_2G    0x88  /**< BDU=1, FS=±2g,  HR=1              */
#define LIS3DHTR_CTRL4_4G    0x98  /**< BDU=1, FS=±4g,  HR=1              */
#define LIS3DHTR_CTRL4_8G    0xA8  /**< BDU=1, FS=±8g,  HR=1              */
#define LIS3DHTR_CTRL4_16G   0xB8  /**< BDU=1, FS=±16g, HR=1              */

/* =========================================================================
 * SENSITIVITY per LSB after >>4 (12-bit HR mode)
 * ========================================================================= */
#define LIS3DHTR_SENS_MG_2G    1.0f   /**< mg/LSB at ±2g  HR               */
#define LIS3DHTR_SENS_MG_4G    2.0f   /**< mg/LSB at ±4g  HR               */
#define LIS3DHTR_SENS_MG_8G    4.0f   /**< mg/LSB at ±8g  HR               */
#define LIS3DHTR_SENS_MG_16G  12.0f /**< mg/LSB at ±16g HR (16000/1365)  */
/* =========================================================================
 * DATA STRUCTURES
 * ========================================================================= */

/**
 * @brief Raw 16-bit acceleration counts from all three axes.
 */
typedef struct {
    int16_t x;   /**< Raw X-axis counts (2's complement) */
    int16_t y;   /**< Raw Y-axis counts                  */
    int16_t z;   /**< Raw Z-axis counts                  */
} lis3dhtr_raw_t;

/**
 * @brief Calibrated acceleration values in milli-g.
 *
 * At ±2 g full-scale with 16-bit output (high-resolution mode):
 *   sensitivity = 1 mg/digit
 */
typedef struct {
    float x_mg;  /**< X acceleration (mg) */
    float y_mg;  /**< Y acceleration (mg) */
    float z_mg;  /**< Z acceleration (mg) */
} lis3dhtr_accel_mg_t;

/**
 * @brief Derived trike orientation and motion state.
 */
typedef struct {
    float    roll_deg;          /**< Roll angle — retained, not published    */
    float    pitch_deg;         /**< Pitch angle — retained, not published   */
    float    accel_mag_mg;      /**< Total vector magnitude (mg)             */
    float    accel_rms_mg;      /**< RMS of X²+Y²+Z² — for vibration        */
    float    x_mg;              /**< X-axis acceleration (mg) — published    */
    float    y_mg;              /**< Y-axis acceleration (mg) — published    */
    float    z_mg;              /**< Z-axis acceleration (mg) — published    */
    bool     in_motion;			/**< true if vehicle is moving (HW interrupt set)*/
    bool     impact_detected;	/**< true if a shock/click event was latched      */
    bool     free_fall;			/**< true if free-fall condition detected         */
} lis3dhtr_motion_t;

/* =========================================================================
 * OPERATING MODE — switchable between production (HPF, ±2g) and test
 * (no HPF, ±16g, gravity visible) per req 4
 * ========================================================================= */

typedef enum {
    LIS3DHTR_MODE_PRODUCTION = 0,  /**< HPF on, ±2g, vibration measurement  */
    LIS3DHTR_MODE_TEST       = 1,  /**< HPF off, ±16g, gravity component visible */
} lis3dhtr_mode_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise the I2C bus and configure the LIS3DHTR.
 *
 * Installs the I2C master driver, verifies WHO_AM_I, configures ODR,
 * full-scale range, high-resolution mode, and sets up the hardware
 * interrupt on INT1 for activity / inactivity detection.
 *
 * @return ESP_OK on success.
 *         ESP_ERR_NOT_FOUND   if WHO_AM_I does not match 0x33.
 *         ESP_ERR_INVALID_STATE if I2C driver installation fails.
 */
esp_err_t lis3dhtr_init(void);

/**
 * @brief Read raw 16-bit acceleration counts from all three axes.
 *
 * @param[out] raw  Pointer to receive raw counts.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_read_raw(lis3dhtr_raw_t *raw);

/**
 * @brief Read calibrated acceleration in milli-g from all three axes.
 *
 * Converts raw counts using the ±2 g sensitivity constant.
 *
 * @param[out] accel  Pointer to receive mg values.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_read_accel_mg(lis3dhtr_accel_mg_t *accel);

/**
 * @brief Read derived trike motion state.
 *
 * Computes roll, pitch, total acceleration magnitude, and checks the
 * INT1_SRC register to determine if the hardware motion-detection
 * interrupt has fired since the last call.
 *
 * @param[out] motion  Pointer to receive motion state.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_read_motion(lis3dhtr_motion_t *motion);

/**
 * @brief Check whether the vehicle is currently in motion.
 *
 * Uses the hardware activity-detection interrupt result latched in
 * INT1_SRC.  Reading INT1_SRC clears the latch automatically.
 *
 * @return true if motion is detected, false if stationary.
 */
bool lis3dhtr_is_in_motion(void);

/**
 * @brief Check and clear any latched impact event.
 *
 * @return true if an impact was detected since the last call.
 */
bool lis3dhtr_check_clear_impact(void);

/**
 * @brief Read WHO_AM_I register and verify the expected value 0x33.
 *
 * @param[out] value  Byte read from the register.
 * @return ESP_OK if read and value == 0x33.
 *         ESP_ERR_INVALID_RESPONSE if value is wrong.
 */
esp_err_t lis3dhtr_check_who_am_i(uint8_t *value);

/**
 * @brief Reconfigure the accelerometer operating mode at runtime.
 *
 * PRODUCTION mode: HPF on (CTRL_REG2=0x39), ±2g, 200 Hz. Output
 * registers show zero when stationary; only vibration is measured.
 * Impact detection: CLICK_THS threshold for ±16g.
 *
 * TEST mode: HPF off (CTRL_REG2=0x00), ±16g, 200 Hz. Gravity
 * component is visible (Z≈1000mg when flat). Useful for verifying
 * sensor orientation and confirming 1g component on each axis.
 * Impact detection: CLICK_THS threshold for ±16g.
 *
 * Both modes retain impact detection (CLICK_CFG) and motion detection
 * (INT1_CFG) interrupt configuration.
 *
 * @param[in] mode  LIS3DHTR_MODE_PRODUCTION or LIS3DHTR_MODE_TEST.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_configure_mode(lis3dhtr_mode_t mode);

/**
 * @brief Get the currently active operating mode.
 * @return Current lis3dhtr_mode_t.
 */
lis3dhtr_mode_t lis3dhtr_get_mode(void);

/**
 * @brief Scan the I2C bus and log all responding addresses.
 *
 * Useful during factory test to confirm device is wired correctly.
 */
void lis3dhtr_i2c_scan(void);

/**
 * @brief Check if an impact has occurred since the last call to this function.
 * 
 * This function consumes the pending impact flag, i.e., it returns true
 * once for each detected impact and then clears the internal flag.
 * Reading CLICK_SRC (to clear the hardware latch) is handled by
 * lis3dhtr_read_motion(), which should be called regularly.
 *
 * @return true if an impact was detected since the last consume call,
 *         false otherwise.
 */
bool lis3dhtr_consume_impact(void);

/**
 * @brief Enable FIFO stream mode.
 *
 * Switches FIFO from Bypass to Stream mode per datasheet §5.1.3.
 * Must be called after lis3dhtr_init().  Safe to call multiple times.
 * Impact (CLICK) and motion (INT1_CFG) interrupts are unaffected —
 * they operate on the raw ODR pipeline before FIFO buffering (§5.1).
 *
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_fifo_enable(void);

/**
 * @brief Drain FIFO and update the 60-slot accumulator ring buffer.
 *
 * Call once per second from diag_log_task.
 * Reads FIFO_SRC_REG (§8.20) for sample count, burst-reads all
 * available samples from OUT_X_L(28h)–OUT_Z_H(2Dh) (§5.1.5),
 * computes per-window RMS and signed peak for each axis, and stores
 * the result in the internal ring buffer.
 *
 * FIFO overrun (OVRN bit in FIFO_SRC_REG) is logged at DEBUG level
 * only — it is expected and harmless in stream mode (§5.1.3).
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if FIFO not enabled.
 */
esp_err_t lis3dhtr_fifo_drain_accumulate(void);

/**
 * @brief Compute and return the 60-second publish statistics.
 *
 * Iterates over all captured windows in the ring buffer and computes:
 *   - RMS-of-RMS per axis (mathematically equivalent to true RMS over
 *     the full window when windows are contiguous and non-overlapping)
 *   - Vector magnitude of the RMS triplet
 *   - Max absolute peak and corresponding signed value per axis
 *
 * Does NOT clear the ring buffer — call lis3dhtr_accumulator_reset()
 * after retrieving stats to start a fresh window.
 *
 * @param[out] stats  Publish-ready statistics.
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_get_publish_stats(lis3dhtr_publish_stats_t *stats);

/**
 * @brief Clear the accumulator ring buffer.
 *
 * Call immediately after lis3dhtr_get_publish_stats() to start the
 * next 60-second accumulation window cleanly.
 */
void lis3dhtr_accumulator_reset(void);

/**
 * @brief Public getter function for s_acc_count.
 */
uint16_t get_s_acc_count();

// New function declaration for 1-second snapshot RMS (non-destructive)
// Returns RMS computed from the last N windows without resetting the buffer.
// Call from diag_log_task at 1Hz for diagnostic logging.
esp_err_t lis3dhtr_get_snapshot_rms(lis3dhtr_publish_stats_t *stats, uint16_t last_n_windows);

/**
 * @brief Calibrate gravity offset by averaging raw samples at boot.
 *
 * Call once after lis3dhtr_init() and lis3dhtr_configure_mode() but
 * BEFORE accel_drain_task starts. Reads n_samples raw readings,
 * computes per-axis mean, stores as gravity offset for all subsequent
 * FIFO drain computations.
 *
 * NOT stored in NVS — recalibrated on every boot.
 * Assumes sensor is stationary during calibration.
 *
 * @param[in] n_samples  Number of samples to average (200 recommended).
 * @return ESP_OK on success.
 */
esp_err_t lis3dhtr_calibrate_gravity(uint16_t n_samples);

/**
 * @brief Return the currently stored gravity offsets (for diagnostics).
 */
void lis3dhtr_get_gravity_offsets(float *x_mg, float *y_mg, float *z_mg);
#ifdef __cplusplus
}
#endif

#endif /* LIS3DHTR_H_ */