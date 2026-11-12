/**
 * @file lsm6dsox.h
 * @brief LSM6DSOX 6-axis IMU Driver for KSC TCU V2.1 (ESP32-S3, I2C)
 *
 * Provides full accelerometer functionality for three-wheeled electric motorcycles:
 *   - Motion detection (vehicle moving vs. stationary via WAKE_UP)
 *   - Tilt / roll / pitch angle estimation
 *   - Impact / shock event detection via hardware interrupt and thresholding
 *   - Hardware FIFO batching (208 Hz stream mode) and 1 Hz RMS accumulation
 *   - Gravity calibration and ThingsBoard telemetry statistics
 *
 * Hardware wiring (TCU V2.1 board):
 *   SCL  → GPIO47 (external pull-up)
 *   SDA  → GPIO48 (external pull-up)
 *   SA0  → GND (0x6A) or VDD (0x6B) [auto-detected]
 *   INT1 → GPIO7
 */

#ifndef LSM6DSOX_H_
#define LSM6DSOX_H_

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * DEVICE CONSTANTS & I2C ADDRESSES
 * ========================================================================= */

#define LSM6DSOX_I2C_ADDR_LOW        0x6A  /**< SA0 tied to GND             */
#define LSM6DSOX_I2C_ADDR_HIGH       0x6B  /**< SA0 tied to VDD             */
#define LSM6DSOX_WHO_AM_I_VAL        0x6C  /**< Expected WHO_AM_I value     */

/* =========================================================================
 * REGISTER MAP
 * ========================================================================= */

#define LSM6DSOX_REG_FIFO_CTRL1      0x07  /**< FIFO watermark threshold LSB */
#define LSM6DSOX_REG_FIFO_CTRL2      0x08  /**< FIFO watermark threshold MSB */
#define LSM6DSOX_REG_FIFO_CTRL3      0x09  /**< FIFO batching rates          */
#define LSM6DSOX_REG_FIFO_CTRL4      0x0A  /**< FIFO mode configuration      */
#define LSM6DSOX_REG_WHO_AM_I        0x0F  /**< Device identification        */
#define LSM6DSOX_REG_CTRL1_XL        0x10  /**< Accel ODR, full-scale, filter */
#define LSM6DSOX_REG_CTRL2_G         0x11  /**< Gyro ODR, full-scale         */
#define LSM6DSOX_REG_CTRL3_C         0x12  /**< Control reg 3: BDU, IF_INC, reset */
#define LSM6DSOX_REG_CTRL4_C         0x13  /**< Control reg 4                */
#define LSM6DSOX_REG_CTRL5_C         0x14  /**< Control reg 5                */
#define LSM6DSOX_REG_CTRL6_C         0x15  /**< Control reg 6                */
#define LSM6DSOX_REG_CTRL7_G         0x16  /**< Gyro filtering               */
#define LSM6DSOX_REG_CTRL8_XL        0x17  /**< Accel filtering (HPF, LPF2)  */
#define LSM6DSOX_REG_CTRL9_XL        0x18  /**< Accel config                 */
#define LSM6DSOX_REG_CTRL10_C        0x19  /**< Control reg 10               */
#define LSM6DSOX_REG_ALL_INT_SRC     0x1A  /**< Source of all interrupts     */
#define LSM6DSOX_REG_WAKE_UP_SRC     0x1B  /**< Wake-up interrupt source     */
#define LSM6DSOX_REG_TAP_SRC         0x1C  /**< Tap interrupt source         */
#define LSM6DSOX_REG_STATUS_REG      0x1E  /**< Data-ready flags             */
#define LSM6DSOX_REG_OUTX_L_G        0x22  /**< Gyro X low byte              */
#define LSM6DSOX_REG_OUTX_L_A        0x28  /**< Accel X low byte             */
#define LSM6DSOX_REG_OUTX_H_A        0x29  /**< Accel X high byte            */
#define LSM6DSOX_REG_OUTY_L_A        0x2A  /**< Accel Y low byte             */
#define LSM6DSOX_REG_OUTY_H_A        0x2B  /**< Accel Y high byte            */
#define LSM6DSOX_REG_OUTZ_L_A        0x2C  /**< Accel Z low byte             */
#define LSM6DSOX_REG_OUTZ_H_A        0x2D  /**< Accel Z high byte            */
#define LSM6DSOX_REG_FIFO_STATUS1    0x3A  /**< FIFO words unread LSB        */
#define LSM6DSOX_REG_FIFO_STATUS2    0x3B  /**< FIFO words MSB, overrun, full*/
#define LSM6DSOX_REG_TAP_CFG0        0x56  /**< Latch interrupt config       */
#define LSM6DSOX_REG_TAP_CFG1        0x57  /**< Tap axes config              */
#define LSM6DSOX_REG_TAP_CFG2        0x58  /**< Basic interrupt enable       */
#define LSM6DSOX_REG_TAP_THS_6D      0x59  /**< Tap threshold                */
#define LSM6DSOX_REG_INT_DUR2        0x5A  /**< Tap duration                 */
#define LSM6DSOX_REG_WAKE_UP_THS     0x5B  /**< Wake-up threshold            */
#define LSM6DSOX_REG_WAKE_UP_DUR     0x5C  /**< Wake-up duration             */
#define LSM6DSOX_REG_FREE_FALL       0x5D  /**< Free-fall config             */
#define LSM6DSOX_REG_MD1_CFG         0x5E  /**< Routing to INT1              */
#define LSM6DSOX_REG_MD2_CFG         0x5F  /**< Routing to INT2              */
#define LSM6DSOX_REG_FIFO_DATA_TAG   0x78  /**< FIFO data output tag         */
#define LSM6DSOX_REG_FIFO_DATA_OUT   0x79  /**< FIFO data output X_L         */

/* =========================================================================
 * SENSITIVITY (mg/LSB) - 16-bit Full Scale
 * ========================================================================= */
#define LSM6DSOX_SENS_MG_2G          0.061f  /**< ±2g:  0.061 mg/digit       */
#define LSM6DSOX_SENS_MG_4G          0.122f  /**< ±4g:  0.122 mg/digit       */
#define LSM6DSOX_SENS_MG_8G          0.244f  /**< ±8g:  0.244 mg/digit       */
#define LSM6DSOX_SENS_MG_16G         0.488f  /**< ±16g: 0.488 mg/digit       */

/* =========================================================================
 * FIFO CONSTANTS
 * ========================================================================= */
#define LSM6DSOX_FIFO_DEPTH          32
#define LSM6DSOX_ACC_BUF_SIZE        400
#define LSM6DSOX_WITMOTION_SCALE     1.32f

/* FIFO TAG for Accelerometer NC */
#define LSM6DSOX_FIFO_TAG_ACCEL      0x02

/* =========================================================================
 * DATA STRUCTURES
 * ========================================================================= */

typedef struct {
    int16_t x;   /**< Raw X-axis counts (2's complement) */
    int16_t y;   /**< Raw Y-axis counts                  */
    int16_t z;   /**< Raw Z-axis counts                  */
} lsm6dsox_raw_t;

typedef struct {
    float x_mg;  /**< X acceleration (mg) */
    float y_mg;  /**< Y acceleration (mg) */
    float z_mg;  /**< Z acceleration (mg) */
} lsm6dsox_accel_mg_t;

typedef struct {
    float x_rms_mg;         /**< RMS of X samples in this window           */
    float y_rms_mg;         /**< RMS of Y samples in this window           */
    float y_rms_hpf_mg;     /**< RMS of Y samples in this window with HPF   */
    float z_rms_mg;         /**< RMS of Z samples in this window           */
    float x_mean_mg;        /**< Mean of X samples in this window          */
    float y_mean_mg;        /**< Mean of Y samples in this window          */
    float z_mean_mg;        /**< Mean of Z samples in this window          */
    float x_max_abs_mg;     /**< Largest |X| sample in window              */
    float y_max_abs_mg;     /**< Largest |Y| sample in window              */
    float z_max_abs_mg;     /**< Largest |Z| sample in window              */
    float x_max_signed_mg;  /**< Signed X with largest |X|                 */
    float y_max_signed_mg;  /**< Signed Y with largest |Y|                 */
    float z_max_signed_mg;  /**< Signed Z with largest |Z|                 */
    uint8_t n_samples;      /**< Samples actually read in window           */
} lsm6dsox_window_t;

typedef struct {
    float x_rms60_mg;          /**< RMS of per-second X RMS values         */
    float y_rms60_mg;          /**< RMS of per-second Y RMS values         */
    float y_rms60_hpf_mg;      /**< RMS of per-second Y RMS values with HPF*/
    float z_rms60_mg;          /**< RMS of per-second Z RMS values         */
    float magnitude_rms60_mg;  /**< sqrt(x²+y²+z²) of rms60 triplet        */
    float x_peak_abs_mg;       /**< Max |X| across all windows             */
    float y_peak_abs_mg;       /**< Max |Y| across all windows             */
    float z_peak_abs_mg;       /**< Max |Z| across all windows             */
    float x_peak_signed_mg;    /**< Signed X for max |X| event             */
    float y_peak_signed_mg;    /**< Signed Y for max |Y| event             */
    float z_peak_signed_mg;    /**< Signed Z for max |Z| event             */
    uint16_t windows_captured; /**< Number of valid windows in buffer      */
} lsm6dsox_publish_stats_t;

typedef struct {
    float roll_deg;          /**< Roll angle                             */
    float pitch_deg;         /**< Pitch angle                            */
    float accel_mag_mg;      /**< Total vector magnitude (mg)            */
    float accel_rms_mg;      /**< RMS of vector magnitude                */
    float x_mg;              /**< X-axis acceleration (mg)               */
    float y_mg;              /**< Y-axis acceleration (mg)               */
    float z_mg;              /**< Z-axis acceleration (mg)               */
    bool  in_motion;         /**< True if vehicle is in motion           */
    bool  impact_detected;   /**< True if shock/impact event latched     */
    bool  free_fall;         /**< True if free-fall detected             */
} lsm6dsox_motion_t;

typedef enum {
    LSM6DSOX_MODE_PRODUCTION = 0,  /**< HPF enabled, ±2g (vibration tracking)  */
    LSM6DSOX_MODE_TEST       = 1,  /**< HPF disabled, ±16g (gravity visible)   */
} lsm6dsox_mode_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize I2C and configure LSM6DSOX.
 * Auto-detects whether SA0 is grounded (0x6A) or high (0x6B).
 */
esp_err_t lsm6dsox_init(void);

/**
 * @brief Read raw 16-bit acceleration counts from all 3 axes.
 */
esp_err_t lsm6dsox_read_raw(lsm6dsox_raw_t *raw);

/**
 * @brief Read calibrated acceleration in mg from all 3 axes.
 */
esp_err_t lsm6dsox_read_accel_mg(lsm6dsox_accel_mg_t *accel);

/**
 * @brief Read derived trike motion and orientation state.
 */
esp_err_t lsm6dsox_read_motion(lsm6dsox_motion_t *motion);

/**
 * @brief Fast check whether vehicle is currently in motion.
 */
bool lsm6dsox_is_in_motion(void);

/**
 * @brief Check and clear latched impact event.
 */
bool lsm6dsox_check_clear_impact(void);

/**
 * @brief Consumes the pending impact flag (returns true once per event).
 */
bool lsm6dsox_consume_impact(void);

/**
 * @brief Verify device identity (reads 0x0F, expects 0x6C).
 */
esp_err_t lsm6dsox_check_who_am_i(uint8_t *value);

/**
 * @brief Configure operating mode (PRODUCTION vs TEST).
 */
esp_err_t lsm6dsox_configure_mode(lsm6dsox_mode_t mode);

/**
 * @brief Get currently active operating mode.
 */
lsm6dsox_mode_t lsm6dsox_get_mode(void);

/**
 * @brief Scan the I2C bus and log all detected devices.
 */
void lsm6dsox_i2c_scan(void);

/**
 * @brief Enable hardware FIFO stream/continuous mode at 208 Hz.
 */
esp_err_t lsm6dsox_fifo_enable(void);

/**
 * @brief Drain FIFO samples, compute RMS/peaks, and store in ring buffer.
 */
esp_err_t lsm6dsox_fifo_drain_accumulate(void);

/**
 * @brief Return publish-interval RMS and peak statistics.
 */
esp_err_t lsm6dsox_get_publish_stats(lsm6dsox_publish_stats_t *stats);

/**
 * @brief Clear the accumulator ring buffer.
 */
void lsm6dsox_accumulator_reset(void);

/**
 * @brief Non-destructive snapshot of the last N windows RMS.
 */
esp_err_t lsm6dsox_get_snapshot_rms(lsm6dsox_publish_stats_t *stats, uint16_t last_n_windows);

/**
 * @brief Calibrate gravity offsets by averaging stationary samples at boot.
 */
esp_err_t lsm6dsox_calibrate_gravity(uint16_t n_samples);

/**
 * @brief Get stored gravity offsets.
 */
void lsm6dsox_get_gravity_offsets(float *x_mg, float *y_mg, float *z_mg);

/* =========================================================================
 * BACKWARD COMPATIBILITY ALIASES (LIS3DHTR Drop-in)
 * ========================================================================= */
typedef lsm6dsox_raw_t            lis3dhtr_raw_t;
typedef lsm6dsox_accel_mg_t       lis3dhtr_accel_mg_t;
typedef lsm6dsox_window_t         lis3dhtr_window_t;
typedef lsm6dsox_publish_stats_t  lis3dhtr_publish_stats_t;
typedef lsm6dsox_motion_t         lis3dhtr_motion_t;
typedef lsm6dsox_mode_t           lis3dhtr_mode_t;

#define LIS3DHTR_MODE_PRODUCTION  LSM6DSOX_MODE_PRODUCTION
#define LIS3DHTR_MODE_TEST        LSM6DSOX_MODE_TEST
#define LIS3DHTR_WHO_AM_I_VAL     LSM6DSOX_WHO_AM_I_VAL

#define lis3dhtr_init                    lsm6dsox_init
#define lis3dhtr_read_raw                lsm6dsox_read_raw
#define lis3dhtr_read_accel_mg           lsm6dsox_read_accel_mg
#define lis3dhtr_read_motion             lsm6dsox_read_motion
#define lis3dhtr_is_in_motion            lsm6dsox_is_in_motion
#define lis3dhtr_check_clear_impact      lsm6dsox_check_clear_impact
#define lis3dhtr_consume_impact          lsm6dsox_consume_impact
#define lis3dhtr_check_who_am_i          lsm6dsox_check_who_am_i
#define lis3dhtr_configure_mode          lsm6dsox_configure_mode
#define lis3dhtr_get_mode                lsm6dsox_get_mode
#define lis3dhtr_i2c_scan                lsm6dsox_i2c_scan
#define lis3dhtr_fifo_enable             lsm6dsox_fifo_enable
#define lis3dhtr_fifo_drain_accumulate   lsm6dsox_fifo_drain_accumulate
#define lis3dhtr_get_publish_stats       lsm6dsox_get_publish_stats
#define lis3dhtr_accumulator_reset       lsm6dsox_accumulator_reset
#define lis3dhtr_get_snapshot_rms        lsm6dsox_get_snapshot_rms
#define lis3dhtr_calibrate_gravity       lsm6dsox_calibrate_gravity
#define lis3dhtr_get_gravity_offsets     lsm6dsox_get_gravity_offsets

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSOX_H_ */
