/**
 * @file lis3dhtr.c
 * @brief LIS3DHTR Accelerometer Driver Implementation for KSC TCU V1.1
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "lis3dhtr.h"
#include "board_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

#define TAG "LIS3DHTR"

/* =========================================================================
 * SENSITIVITY CONSTANT
 * =========================================================================
 * At ±2 g full-scale in high-resolution (16-bit) mode:
 *   sensitivity = 1 mg / digit
 *
 * Output is left-justified in 16-bit register; actual resolution is
 * 12-bit (bits [15:4]).  Right-shift 4 then multiply by 1.0 mg/digit.
 * ========================================================================= */

/* Active sensitivity — updated when mode changes */
static float s_sensitivity_mg = LIS3DHTR_SENS_MG_16G;
static lis3dhtr_mode_t s_mode = LIS3DHTR_MODE_PRODUCTION;

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static bool s_initialized = false;

static bool s_impact_pending = false;

/* =========================================================================
 * FIFO & ACCUMULATOR STATE
 * ========================================================================= */

static bool s_fifo_enabled = false;

/**
 * Ring buffer storing per-second FIFO drain results.
 * Written by lis3dhtr_fifo_drain_accumulate() at 1 Hz.
 * Read by lis3dhtr_get_publish_stats() at each 60-second publish tick.
 * No mutex needed — only diag_log_task writes, only mqtt_publish_task
 * reads (at publish time, after the most recent drain has completed).
 */
static lis3dhtr_window_t s_acc_buf[LIS3DHTR_ACC_BUF_SIZE];
static uint16_t           s_acc_head   = 0;   /* next write slot (0–59) */
static uint16_t           s_acc_count  = 0;   /* valid entries (0–60)   */

/* =========================================================================
 * GRAVITY CALIBRATION STATE
 * =========================================================================
 * Computed once at boot by lis3dhtr_calibrate_gravity().
 * Not stored in NVS — recalibrated on every power-on.
 * Subtracted from each raw sample before RMS computation in
 * lis3dhtr_fifo_drain_accumulate() to remove the DC gravity component.
 * ========================================================================= */
static float s_gravity_x_mg = 0.0f;
static float s_gravity_y_mg = 0.0f;
static float s_gravity_z_mg = 0.0f;
static bool  s_gravity_calibrated = false;

/* Scale factor applied after gravity removal to align with Witmotion data */
#define LIS3DHTR_WITMOTION_SCALE  1.32f

/* =========================================================================
 * LOW-LEVEL I2C PRIMITIVES
 * ========================================================================= */

/**
 * @brief Write a single byte to a register.
 *
 * @param[in] reg    Register address.
 * @param[in] value  Byte to write.
 * @return esp_err_t
 */
static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read one or more consecutive bytes starting at reg.
 *
 * Sets bit 7 of the register address to enable the LIS3DHTR
 * auto-increment (SUB bit 7 = 1) for multi-byte reads.
 *
 * @param[in]  reg  Starting register address.
 * @param[out] dst  Buffer to receive bytes.
 * @param[in]  len  Number of bytes to read.
 * @return esp_err_t
 */
static esp_err_t read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    uint8_t reg_addr = (len > 1) ? (reg | 0x80) : reg;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);   /* Repeated start */
    i2c_master_write_byte(cmd,
        (LIS3DHTR_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, dst, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dst + (len - 1), I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd,
                                          pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

esp_err_t lis3dhtr_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* Install I2C master driver (uses board_config.h pin definitions) */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BOARD_I2C_SDA_PIN,
        .scl_io_num       = BOARD_I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,  /* External pull-ups fitted */
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = BOARD_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(BOARD_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(BOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify device identity */
    uint8_t who = 0;
    ret = lis3dhtr_check_who_am_i(&who);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I check failed — check wiring");
        return ret;
    }

    /*
     * CTRL_REG1: 200 Hz ODR, normal power, X/Y/Z enabled
     *   0x67 = 0011_0111
     *   [7:4] ODR = 0110 → 200 Hz
     *   [3]   LPen = 0   → Normal mode
     *   [2:0] Zen/Yen/Xen = 111
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG1, LIS3DHTR_CTRL1_200HZ_NORMAL);
    //ret = write_reg(LIS3DHTR_REG_CTRL_REG1, LIS3DHTR_CTRL1_50HZ_NORMAL);
    //ret = write_reg(LIS3DHTR_REG_CTRL_REG1, LIS3DHTR_CTRL1_25HZ_NORMAL);
    if (ret != ESP_OK) { goto init_fail; }
    
	/* 
	 * CTRL_REG2: Enable high-pass filter on output data registers and INT1 output
	 * CTRL_REG2 = 0x39 = 0011 1001
	 * Bit 7 = 0  → HPM1    = 0  ┐  HPF mode = Normal
	 * Bit 6 = 0  → HPM0    = 0  ┘
	 * Bit 5 = 1  → HPCF2   = 1  ┐  Cutoff = 1 Hz at 200 Hz ODR
	 * Bit 4 = 1  → HPCF1   = 1  ┘
	 * Bit 3 = 1  → FDS     = 1  ← HPF filtered data to OUT registers
	 * Bit 2 = 0  → HPCLICK = 0  ← HPF NOT applied to click (raw data for impact)
	 * Bit 1 = 0  → HP_IA2  = 0  ← HPF NOT applied to INT2
	 * Bit 0 = 1  → HP_IA1  = 1  ← HPF applied to INT1 (gravity-free motion detect)
	 */
	ret = write_reg(LIS3DHTR_REG_CTRL_REG2, 0x39);     
	if (ret != ESP_OK) { goto init_fail; }
    /*
     * CTRL_REG4: ±16 g, high-resolution mode, BDU enabled
     *   0xB8 = 1011_1000
     *   [7]   BDU = 1   → Block data update (avoids reading high/low
     *                       byte from two different samples)
     *   [6]   BLE = 0   → Little-endian (LSB at lower address)
     *   [5:4] FS  = 11  → ±16 g full-scale
     *   [3]   HR  = 1   → High-resolution (12-bit effective)
     *   [1:0] SIM = 00  → SPI 4-wire (irrelevant in I2C mode)
     * 	 ±16g chosen to prevent saturation during automotive impacts.
     * 	 Resolution is 11.718 mg/LSB — sufficient for vibration at 200 Hz.
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG4, LIS3DHTR_CTRL4_16G);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * CTRL_REG5: Latch interrupt on INT1
     *   0x08 = 0000_1000
     *   [3] LIR_INT1 = 1 → Interrupt latched until INT1_SRC is read
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG5, 0x08);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * CTRL_REG3: Route both activity interrupt (IA1) AND click interrupt
     * to INT1 pin.
     *   0xC0 = 1100_0000
     *   [7] I1_CLICK = 1 → Click interrupt routed to INT1
     *   [6] I1_IA1   = 1 → Activity (INT1_CFG) routed to INT1
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG3, 0xC0);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * INT1_CFG: Enable OR combination of high events on all axes
     *   0x2A = 0010_1010
     *   [5] ZHIE = 1, [3] YHIE = 1, [1] XHIE = 1
     *   Triggers when any axis exceeds the threshold
     */
    ret = write_reg(LIS3DHTR_REG_INT1_CFG, 0x2A);
    if (ret != ESP_OK) { goto init_fail; }   

    /*
     * INT1_THS: Motion detection threshold
     * With HPF on and ±16g: 1 LSB = 125 mg.
     * LIS3DHTR_MOTION_THS_DEFAULT = 26 LSB × 125 mg = 3250 mg ≈ 3.25 g
     *
     * Note: INT1_THS uses the same FS/128 scale as CLICK_THS.
     * At ±16g: 1 LSB = 16000/128 = 125 mg.
     * 26 × 125 = 3250 mg → triggers on sustained motion above 3.25g.
     * This is appropriate for motion detection (trike moving) vs impact
     * (short transient handled by CLICK). For very gentle trike movement
     * consider reducing to 8–12 LSB (~1–1.5g).
     */
    ret = write_reg(LIS3DHTR_REG_INT1_THS, LIS3DHTR_MOTION_THS_DEFAULT);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * INT1_DURATION: Minimum event duration before interrupt fires
     * At 25 Hz, 1 LSB = 1/25 = 40 ms
     * At 200Hz, 1 LSB = 1/200 = 5 ms
     * INT1_DURATION: 40 LSB × 5 ms = 200 ms debounce at 200 Hz
     */
    ret = write_reg(LIS3DHTR_REG_INT1_DURATION, LIS3DHTR_MOTION_DUR_DEFAULT);
    if (ret != ESP_OK) { goto init_fail; }
    
    /* -----------------------------------------------------------------------
     * CLICK / IMPACT DETECTION
     * -----------------------------------------------------------------------
     * CLICK_CFG (0x38): Enable single-click on all three axes.
     *   0x15 = 0001_0101
     *   [4] XS = 1 → single-click X enabled
     *   [2] YS = 1 → single-click Y enabled
     *   [0] ZS = 1 → single-click Z enabled
     *   Double-click bits [5,3,1] = 0 (not used — impact is single event)
     */
    ret = write_reg(LIS3DHTR_REG_CLICK_CFG, 0x15);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * CLICK_THS (0x3A): Impact threshold + latch enable
     *   Bit 7 (LIR_Click) = 1 → latch interrupt until CLICK_SRC is read
     *   Bits [6:0]         = threshold in FS/128 units
     *
     * At ±16g: 1 LSB = 186 mg. 27 LSB = 5000 mg = 5g (automotive impact).
     *   0x80 | 40 = 0xA8
     */
    ret = write_reg(LIS3DHTR_REG_CLICK_THS,
                    0x80 | LIS3DHTR_IMPACT_THS_16G);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * TIME_LIMIT (0x3B): Maximum click pulse duration
     *   At 200 Hz, 1 LSB = 5 ms.
     *   2 LSB = 10 ms → genuine impact transient window.
     *   Road bumps that last >10 ms will not trigger click interrupt.
     */
    ret = write_reg(LIS3DHTR_REG_TIME_LIMIT, LIS3DHTR_IMPACT_TIME_LIMIT);
    if (ret != ESP_OK) { goto init_fail; }

    /*
     * TIME_LATENCY (0x3C): For single-click only, set to 0
     * (latency only applies to double-click detection)
     */
    ret = write_reg(LIS3DHTR_REG_TIME_LATENCY, 0x00);
    if (ret != ESP_OK) { goto init_fail; }    

    /*
     * Configure INT1 GPIO as input (no pull — hardware driver on board)
     */
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << BOARD_ACCEL_INT1_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,  /* Polled — no FreeRTOS ISR needed */
    };
    gpio_config(&int_conf);

	s_mode = LIS3DHTR_MODE_PRODUCTION;
    s_initialized = true;
    ESP_LOGI(TAG, "LIS3DHTR initialised — 25 Hz, ±16 g, HR mode, INT1=IO%d",
             BOARD_ACCEL_INT1_PIN);
             
   vTaskDelay(pdMS_TO_TICKS(50));   // 50ms > 35ms required turn-on time at 200Hz
   /* Turn-on: 7/50 = 140 ms  ; Turn on time = 7/ODR*/
	//vTaskDelay(pdMS_TO_TICKS(150));  
	
	//vTaskDelay(pdMS_TO_TICKS(300));  /* 300ms > 280ms required at 25 Hz HR mode (datasheet Table 11: 7/ODR) */       
    return ESP_OK;

init_fail:
    ESP_LOGE(TAG, "LIS3DHTR configuration failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t lis3dhtr_check_who_am_i(uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = read_regs(LIS3DHTR_REG_WHO_AM_I, value, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X) — %s",
             *value, LIS3DHTR_WHO_AM_I_VAL,
             (*value == LIS3DHTR_WHO_AM_I_VAL) ? "PASS" : "FAIL");

    return (*value == LIS3DHTR_WHO_AM_I_VAL) ? ESP_OK :
                                                ESP_ERR_INVALID_RESPONSE;
}

esp_err_t lis3dhtr_read_raw(lis3dhtr_raw_t *raw)
{
    if (!s_initialized || raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[6] = {0};
    esp_err_t ret = read_regs(LIS3DHTR_REG_OUT_X_L, buf, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acceleration data: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    /* LIS3DHTR outputs little-endian (LSB at lower address) */
    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    return ESP_OK;
}

esp_err_t lis3dhtr_read_accel_mg(lis3dhtr_accel_mg_t *accel)
{
    if (!s_initialized || accel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dhtr_raw_t raw;
    esp_err_t ret = lis3dhtr_read_raw(&raw);
    if (ret != ESP_OK) {
        return ret;
    }

    /*
     * In high-resolution 12-bit mode the output is left-justified.
     * Right-shift 4 bits to get the 12-bit value, then multiply by
     * sensitivity (1 mg / digit at ±2 g).
     */
	/* Use active sensitivity — changes with mode/FS setting */
	    accel->x_mg = (float)(raw.x >> 4) * s_sensitivity_mg;
	    accel->y_mg = (float)(raw.y >> 4) * s_sensitivity_mg;
	    accel->z_mg = (float)(raw.z >> 4) * s_sensitivity_mg;

    return ESP_OK;
}

esp_err_t lis3dhtr_read_motion(lis3dhtr_motion_t *motion)
{
    if (!s_initialized || motion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(motion, 0, sizeof(lis3dhtr_motion_t));

    /* Read calibrated acceleration */
    lis3dhtr_accel_mg_t accel;
    esp_err_t ret = lis3dhtr_read_accel_mg(&accel);
    if (ret != ESP_OK) {
        return ret;
    }
	/* Store raw axis values for publishing */
	motion->x_mg = accel.x_mg;
	motion->y_mg = accel.y_mg;
	motion->z_mg = accel.z_mg; 
	
	/* RMS acceleration — used by server for vibration analysis.
	 * RMS = sqrt((x² + y² + z²) / 3)
	 * Dividing by 3 normalises across axes; the server receives a
	 * single scalar that represents the overall vibration energy level.
	 * Note: accel_mag_mg = sqrt(x²+y²+z²) is the vector magnitude (not RMS).
	 * RMS and magnitude differ: magnitude includes the 1g gravity component
	 * on the dominant axis; RMS normalises it across three axes.             */
	motion->accel_rms_mg = sqrtf(
	    (accel.x_mg * accel.x_mg +
	     accel.y_mg * accel.y_mg +
	     accel.z_mg * accel.z_mg) / 3.0f);	   

    /* Total acceleration magnitude */
    motion->accel_mag_mg = sqrtf(accel.x_mg * accel.x_mg +
                                  accel.y_mg * accel.y_mg +
                                  accel.z_mg * accel.z_mg);

    /*
     * Roll angle: rotation around X-axis (lateral tilt of trike)
     *   roll = atan2(y, z)
     */
    motion->roll_deg  = atan2f(accel.y_mg, accel.z_mg) * (180.0f / M_PI);

    /*
     * Pitch angle: rotation around Y-axis (forward/backward tilt)
     *   pitch = atan2(-x, sqrt(y²+z²))
     */
    motion->pitch_deg = atan2f(-accel.x_mg,
                                sqrtf(accel.y_mg * accel.y_mg +
                                      accel.z_mg * accel.z_mg))
                        * (180.0f / M_PI);

    /*
     * Free-fall: acceleration magnitude drops well below 1 g.
     * Threshold: < 350 mg on all axes simultaneously.
     */
    motion->free_fall = (motion->accel_mag_mg < 350.0f);

    /*
     * Motion / impact: read INT1_SRC register.
     * Reading this register automatically clears the latched interrupt.
     * Bit 6 (IA) = 1 → at least one enabled event has occurred.
     * Bit 4/5 (ZHIE/ZLIE etc.) give axis detail.
     */
    uint8_t int1_src = 0;
    ret = read_regs(LIS3DHTR_REG_INT1_SRC, &int1_src, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    bool hw_event = (int1_src & 0x40) != 0;   /* IA bit */
    motion->in_motion = hw_event;

    /*
     * Impact detection: use the click/shock detection register.
     * Read CLICK_SRC to check if a single or double click occurred.
     */
    uint8_t click_src = 0;
    ret = read_regs(LIS3DHTR_REG_CLICK_SRC, &click_src, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    // Set the software pending flag if an impact is detected.
    // This flag remains true until consumed by lis3dhtr_consume_impact().
    motion->impact_detected = (click_src & 0x40) != 0;  /* IA bit */
    if (motion->impact_detected) {
        s_impact_pending = true;
    }

    ESP_LOGD(TAG, "Motion: roll=%.1f° pitch=%.1f° |a|=%.0f mg "
             "in_motion=%d impact=%d ff=%d",
             motion->roll_deg, motion->pitch_deg, motion->accel_mag_mg,
             motion->in_motion, motion->impact_detected, motion->free_fall);

    return ESP_OK;
}

bool lis3dhtr_is_in_motion(void)
{
    if (!s_initialized) {
        return false;
    }

    /*
     * Check the physical INT1 GPIO level as a fast path — the hardware
     * interrupt is asserted (active-high after latch) when activity is
     * detected.  This avoids an I2C transaction on every poll when the
     * trike is stationary.
     */
    int gpio_level = gpio_get_level(BOARD_ACCEL_INT1_PIN);
    if (gpio_level == 0) {
        return false;   /* No interrupt pending → not in motion */
    }

    /*
     * GPIO is high → read INT1_SRC to confirm and clear the latch.
     */
    uint8_t int1_src = 0;
    if (read_regs(LIS3DHTR_REG_INT1_SRC, &int1_src, 1) != ESP_OK) {
        return false;
    }
    return (int1_src & 0x40) != 0;
}

bool lis3dhtr_check_clear_impact(void)
{
    if (!s_initialized) {
        return false;
    }

    uint8_t click_src = 0;
    if (read_regs(LIS3DHTR_REG_CLICK_SRC, &click_src, 1) != ESP_OK) {
        return false;
    }
    return (click_src & 0x40) != 0;
}

esp_err_t lis3dhtr_configure_mode(lis3dhtr_mode_t mode)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    if (mode == LIS3DHTR_MODE_TEST) {
        /*
         * TEST MODE: Remove HPF so gravity component is visible.
         * Switch to ±16g full-scale.
         * Useful for verifying 1g component when tilting each axis.
         *
         * CTRL_REG2 = 0x00 — no HPF anywhere
         * CTRL_REG4 = LIS3DHTR_CTRL4_16G — ±16g, HR, BDU
         * CLICK_THS — ±16g impact threshold (5g)
         * INT1_THS  — motion threshold recalculated for ±16g
         */
        ret = write_reg(LIS3DHTR_REG_CTRL_REG2, 0x00);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CTRL_REG4, LIS3DHTR_CTRL4_16G);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CLICK_THS,
                        0x80 | LIS3DHTR_IMPACT_THS_16G);
        if (ret != ESP_OK) return ret;

        s_sensitivity_mg = LIS3DHTR_SENS_MG_16G;
        s_mode           = LIS3DHTR_MODE_TEST;

        ESP_LOGI(TAG, "Mode → TEST: HPF OFF, ±16g, gravity visible. "
                 "Expected Z≈+856mg flat (1000mg / 11.718mg/LSB × 10)");
    } else {
        /*
         * PRODUCTION MODE: HPF on, ±16g (retain 16g for impact range).
         * Output registers show ~0mg when stationary (DC removed).
         */
        ret = write_reg(LIS3DHTR_REG_CTRL_REG2, 0x39);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CTRL_REG4, LIS3DHTR_CTRL4_16G);
        if (ret != ESP_OK) return ret;

        ret = write_reg(LIS3DHTR_REG_CLICK_THS,
                        0x80 | LIS3DHTR_IMPACT_THS_16G);
        if (ret != ESP_OK) return ret;

        s_sensitivity_mg = LIS3DHTR_SENS_MG_16G;
        s_mode           = LIS3DHTR_MODE_PRODUCTION;

        ESP_LOGI(TAG, "Mode → PRODUCTION: HPF ON, ±16g, vibration only");
    }

    return ESP_OK;
}

lis3dhtr_mode_t lis3dhtr_get_mode(void)
{
    return s_mode;
}

void lis3dhtr_i2c_scan(void)
{
    ESP_LOGI(TAG, "--- I2C Bus Scan (port %d) ---", BOARD_I2C_PORT);
    int found = 0;

    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd,
                                              pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Device at 0x%02X%s", addr,
                     (addr == LIS3DHTR_I2C_ADDR) ? " <- LIS3DHTR" : "");
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "  No devices found! Check wiring and pull-ups.");
    } else {
        ESP_LOGI(TAG, "  Scan complete — %d device(s) found.", found);
    }
    ESP_LOGI(TAG, "--- End I2C Scan ---");
}

bool lis3dhtr_consume_impact(void)
{
    bool ret = s_impact_pending;
    s_impact_pending = false;
    return ret;
}

/* =========================================================================
 * FIFO ENABLE
 * Datasheet §5.1, §8.12 CTRL_REG5(24h), §8.19 FIFO_CTRL_REG(2Eh)
 * ========================================================================= */

esp_err_t lis3dhtr_fifo_enable(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "fifo_enable: not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    /*
     * Step 1 — Reset to Bypass first.
     * Datasheet §5.1.2: "Bypass mode must be used in order to reset
     * the FIFO buffer when a different mode is operating."
     * Write FM[1:0]=00 to FIFO_CTRL_REG before switching to Stream.
     */
    ret = write_reg(LIS3DHTR_REG_FIFO_CTRL, LIS3DHTR_FIFO_BYPASS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FIFO bypass reset failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    /*
     * Step 2 — Enable FIFO block in CTRL_REG5.
     * Datasheet §8.12: FIFO_EN bit[6] = 1.
     * Preserve LIR_INT1 bit[3] = 1 (already set in init).
     * Result: 0x40 | 0x08 = 0x48
     */
    ret = write_reg(LIS3DHTR_REG_CTRL_REG5, 0x48);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CTRL_REG5 FIFO_EN failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * Step 3 — Select Stream mode in FIFO_CTRL_REG.
     * Datasheet §8.19, Table 50: FM[1:0] bits[7:6] = 10 → 0x80.
     * TR=0, FTH[4:0]=0 (watermark not used).
     * In Stream mode, FIFO fills to 32 then continuously overwrites
     * oldest sample — always holds the latest 160 ms at 200 Hz (§5.1.3).
     */
    ret = write_reg(LIS3DHTR_REG_FIFO_CTRL, LIS3DHTR_FIFO_STREAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FIFO stream mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_fifo_enabled = true;
    lis3dhtr_accumulator_reset();
    ESP_LOGI(TAG, "FIFO stream mode enabled — 32 samples @ 200 Hz = 160 ms/window");
    return ESP_OK;
}

/* =========================================================================
 * FIFO DRAIN — called once per second from diag_log_task
 * Datasheet §5.1.3, §5.1.5, §8.20 FIFO_SRC_REG(2Fh)
 * ========================================================================= */

esp_err_t lis3dhtr_fifo_drain_accumulate(void)
{
    if (!s_initialized || !s_fifo_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    /* ── 1. Read FIFO_SRC_REG (datasheet §8.20) ──────────────────────────
     * FSS[4:0] (bits 4:0) = number of unread samples currently in FIFO.
     * OVRN (bit 6) = 1 when FIFO has wrapped in stream mode — expected,
     *   harmless; oldest sample was replaced (§5.1.3).
     * EMPTY (bit 5) = 1 when no samples available.
     * -------------------------------------------------------------------- */
    uint8_t fifo_src = 0;
    esp_err_t ret = read_regs(LIS3DHTR_REG_FIFO_SRC, &fifo_src, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FIFO_SRC_REG read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (fifo_src & LIS3DHTR_FIFO_SRC_EMPTY) {
        ESP_LOGI(TAG, "FIFO empty — skipping drain");
        return ESP_OK;
    }

    /* OVRN is expected at 200 Hz when we drain only once per second */
    if (fifo_src & LIS3DHTR_FIFO_SRC_OVRN) {
        //ESP_LOGI(TAG, "FIFO overrun (normal in stream mode at 200 Hz or 50Hz)");
    }

    uint8_t n = fifo_src & LIS3DHTR_FIFO_SRC_FSS_MASK;
    //ESP_LOGI(TAG, "Samples stored in FIFO BUF %d",n);
    if (n == 0) return ESP_OK;
    if (n > LIS3DHTR_FIFO_DEPTH) n = LIS3DHTR_FIFO_DEPTH;  /* sanity cap */

    /* ── 2. Burst-read n×6 bytes from OUT_X_L(28h)–OUT_Z_H(2Dh) ─────────
     * Datasheet §5.1.5: "Each time data is read from the FIFO, the oldest
     * X, Y and Z data are placed in the OUT registers. The address rolls
     * back to 0x28 when 0x2D is reached."
     * read_regs() sets bit 7 of the register address (SUB auto-increment)
     * when len > 1, which is required for multi-byte burst reads (§6.1.1).
     * -------------------------------------------------------------------- */
     /*192 bytes in BSS (static data), never on the stack*/
    static uint8_t buf[LIS3DHTR_FIFO_DEPTH * 6];
    ret = read_regs(LIS3DHTR_REG_OUT_X_L, buf, (size_t)n * 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FIFO burst read (%d samples) failed: %s",
                 n, esp_err_to_name(ret));
        return ret;
    }

    /* ── 3. Parse and compute per-window statistics ───────────────────────
     *
     * RMS per axis: sqrt(mean(x_i²)) over n samples.
     * This equals the true RMS when the mean of x is ~0, which is the case
     * with HPF active — gravity is removed so the DC component is absent.
     *
     * Peak: largest absolute value, with sign preserved.
     * Signed peak captures directionality for post-processing on TB.
     * -------------------------------------------------------------------- */
    float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_z2 = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float peak_x_abs = 0.0f, peak_y_abs = 0.0f, peak_z_abs = 0.0f;
    float peak_x_sgn = 0.0f, peak_y_sgn = 0.0f, peak_z_sgn = 0.0f;

    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *b = &buf[i * 6];

        /* Little-endian 16-bit, left-justified → right-shift 4 for 12-bit */
        int16_t raw_x = (int16_t)((b[1] << 8) | b[0]);
        int16_t raw_y = (int16_t)((b[3] << 8) | b[2]);
        int16_t raw_z = (int16_t)((b[5] << 8) | b[4]);

        float x = (float)(raw_x >> 4) * s_sensitivity_mg;
        float y = (float)(raw_y >> 4) * s_sensitivity_mg;
        float z = (float)(raw_z >> 4) * s_sensitivity_mg;

        /* Remove gravity component and apply Witmotion scaling.
         * Gravity offsets are computed at boot by lis3dhtr_calibrate_gravity().
         * If not yet calibrated, offsets remain 0 (no correction applied). */
        if (s_gravity_calibrated) {
            x = (x - s_gravity_x_mg) * LIS3DHTR_WITMOTION_SCALE;
            y = (y - s_gravity_y_mg) * LIS3DHTR_WITMOTION_SCALE;
            z = (z - s_gravity_z_mg) * LIS3DHTR_WITMOTION_SCALE;
        }

        sum_x2 += x * x;
        sum_y2 += y * y;
        sum_z2 += z * z;
        
       	sum_x += x;
        sum_y += y;
        sum_z += z;

        float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);
        if (ax > peak_x_abs) { peak_x_abs = ax; peak_x_sgn = x; }
        if (ay > peak_y_abs) { peak_y_abs = ay; peak_y_sgn = y; }
        if (az > peak_z_abs) { peak_z_abs = az; peak_z_sgn = z; }
    }

    float fn = (float)n;
    lis3dhtr_window_t w = {
        .x_rms_mg        = sqrtf(sum_x2 / fn),
        .y_rms_mg        = sqrtf(sum_y2 / fn),
        //.y_rms_hpf_mg	 = sqrtf((sum_y2 / fn)-((sum_y / fn)*(sum_y / fn))),
        //.y_rms_hpf_mg = sqrtf(fmaxf((sum_y2 / fn) - ((sum_y / fn) * (sum_y / fn)), 0.0f)),
        .z_rms_mg        = sqrtf(sum_z2 / fn),
        .x_mean_mg        = sum_x / fn,
        .y_mean_mg        = sum_y / fn,
        .z_mean_mg        = sum_z / fn,
        .x_max_abs_mg    = peak_x_abs,
        .y_max_abs_mg    = peak_y_abs,
        .z_max_abs_mg    = peak_z_abs,
        .x_max_signed_mg = peak_x_sgn,
        .y_max_signed_mg = peak_y_sgn,
        .z_max_signed_mg = peak_z_sgn,
        .n_samples       = n,
    };

    /* ── 4. Store in ring buffer ──────────────────────────────────────────
     * s_acc_head always points to the next write slot (circular, 0–399).
     * s_acc_count tracks how many valid entries exist (capped at 400).
     * -------------------------------------------------------------------- */
    s_acc_buf[s_acc_head] = w;
    s_acc_head = (s_acc_head + 1) % LIS3DHTR_ACC_BUF_SIZE;
    if (s_acc_count < LIS3DHTR_ACC_BUF_SIZE) s_acc_count++;

/*	ESP_LOGI(TAG, "FIFO window: n=%d X_RMS: %.1f mg,Y_RMS: %.1f mg,Z_RMS: %.1f mg; X_MAX_RAW: %.1f mg,Y_MAX_RAW: %.1f mg,Z_MAX_RAW: %.1f mg",
	             n, w.x_rms_mg, w.y_rms_mg, w.z_rms_mg,
	             w.x_max_signed_mg, w.y_max_signed_mg, w.z_max_signed_mg);


    //--- DEBUG: log every raw sample for Witmotion comparison --- 
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *b = &buf[i * 6];
        int16_t raw_x = (int16_t)((b[1] << 8) | b[0]);
        int16_t raw_y = (int16_t)((b[3] << 8) | b[2]);
        int16_t raw_z = (int16_t)((b[5] << 8) | b[4]);
        float sx = (float)(raw_x >> 4) * s_sensitivity_mg;
        float sy = (float)(raw_y >> 4) * s_sensitivity_mg;
        float sz = (float)(raw_z >> 4) * s_sensitivity_mg;
        ESP_LOGI(TAG, "SAMPLE %02d: X=%.3f Y=%.3f Z=%.3f g",
                 i,
                 sx / 1000.0f,
                 sy / 1000.0f,
                 sz / 1000.0f);
    }
    // --- END DEBUG ---  */           

    return ESP_OK;
}

/* =========================================================================
 * COMPUTE PUBLISH STATISTICS — called at each 60-second publish tick
 * ========================================================================= */

esp_err_t lis3dhtr_get_publish_stats(lis3dhtr_publish_stats_t *stats)
{
    if (stats == NULL) return ESP_ERR_INVALID_ARG;
    memset(stats, 0, sizeof(lis3dhtr_publish_stats_t));

    uint16_t count = s_acc_count;
    if (count == 0) {
        ESP_LOGW(TAG, "get_publish_stats: accumulator empty");
        return ESP_OK;
    }

    /*
     * Iterate over the ring buffer in chronological order.
     * The oldest entry is at index:
     *   (s_acc_head - count + LIS3DHTR_ACC_BUF_SIZE) % LIS3DHTR_ACC_BUF_SIZE
     * We iterate count entries forward from there.
     *
     * RMS-of-RMS:
     *   rms60_x = sqrt( mean( rms_x_i² ) over i=0..count-1 )
     * This equals the true RMS over all n_i samples combined when
     * windows are contiguous and non-overlapping, which is the case
     * when lis3dhtr_fifo_drain_accumulate() is called once per second.
     */
    float sum_x2 = 0.0f, sum_y2 = 0.0f,sum_y2_hpf = 0.0f, sum_z2 = 0.0f;
    float pk_x_abs = 0.0f, pk_y_abs = 0.0f, pk_z_abs = 0.0f;
    float pk_x_sgn = 0.0f, pk_y_sgn = 0.0f, pk_z_sgn = 0.0f;

    uint16_t start = (uint16_t)((s_acc_head - count + LIS3DHTR_ACC_BUF_SIZE)
                               % LIS3DHTR_ACC_BUF_SIZE);

    for (uint16_t i = 0; i < count; i++) {
        const lis3dhtr_window_t *w =
            &s_acc_buf[(start + i) % LIS3DHTR_ACC_BUF_SIZE];

        sum_x2 += w->x_rms_mg * w->x_rms_mg;
        sum_y2 += w->y_rms_mg * w->y_rms_mg;
        sum_y2_hpf += w->y_rms_hpf_mg * w->y_rms_hpf_mg;
        sum_z2 += w->z_rms_mg * w->z_rms_mg;

        if (w->x_max_abs_mg > pk_x_abs) {
            pk_x_abs = w->x_max_abs_mg; pk_x_sgn = w->x_max_signed_mg;
        }
        if (w->y_max_abs_mg > pk_y_abs) {
            pk_y_abs = w->y_max_abs_mg; pk_y_sgn = w->y_max_signed_mg;
        }
        if (w->z_max_abs_mg > pk_z_abs) {
            pk_z_abs = w->z_max_abs_mg; pk_z_sgn = w->z_max_signed_mg;
        }
    }

    float fn = (float)count;
    stats->x_rms60_mg       = sqrtf(sum_x2 / fn);
    stats->y_rms60_mg       = sqrtf(sum_y2 / fn);
    stats->y_rms60_hpf_mg   = sqrtf(sum_y2_hpf / fn);
    stats->z_rms60_mg       = sqrtf(sum_z2 / fn);
    stats->magnitude_rms60_mg = sqrtf(
        stats->x_rms60_mg * stats->x_rms60_mg +
        stats->y_rms60_mg * stats->y_rms60_mg +
        stats->z_rms60_mg * stats->z_rms60_mg);
    stats->x_peak_abs_mg    = pk_x_abs;
    stats->y_peak_abs_mg    = pk_y_abs;
    stats->z_peak_abs_mg    = pk_z_abs;
    stats->x_peak_signed_mg = pk_x_sgn;
    stats->y_peak_signed_mg = pk_y_sgn;
    stats->z_peak_signed_mg = pk_z_sgn;
    stats->windows_captured = count;

    ESP_LOGI(TAG, "Publish stats (%d windows): rms60(%.1f,%.1f,%.1f,Y_RMS_60_HPF: %.1f)mg "
             "mag=%.1fmg peak_signed(%.1f,%.1f,%.1f)mg",
             count,
             stats->x_rms60_mg, stats->y_rms60_mg, stats->z_rms60_mg,stats->y_rms60_hpf_mg,
             stats->magnitude_rms60_mg,
             stats->x_peak_signed_mg, stats->y_peak_signed_mg, stats->z_peak_signed_mg);

    return ESP_OK;
}

void lis3dhtr_accumulator_reset(void)
{
    s_acc_head  = 0;
    s_acc_count = 0;
    memset(s_acc_buf, 0, sizeof(s_acc_buf));
    ESP_LOGI(TAG, "Accumulator reset");
}

uint16_t get_s_acc_count(){return s_acc_count;}

/**
 * @brief Compute RMS over the most recent last_n_windows windows.
 *
 * NON-DESTRUCTIVE — does not reset the ring buffer.
 * Used by diag_log_task for 1-second diagnostic logging.
 * Uses the same RMS-of-RMS formula as lis3dhtr_get_publish_stats().
 *
 * @param[out] stats         Filled with RMS values for the requested window.
 * @param[in]  last_n_windows Number of most-recent windows to include (0 = all).
 */
esp_err_t lis3dhtr_get_snapshot_rms(lis3dhtr_publish_stats_t *stats,
                                     uint16_t last_n_windows)
{
    if (stats == NULL) return ESP_ERR_INVALID_ARG;
    memset(stats, 0, sizeof(lis3dhtr_publish_stats_t));

    uint16_t count = s_acc_count;
    if (count == 0) return ESP_OK;

    /* Clamp to available windows */
    if (last_n_windows == 0 || last_n_windows > count) {
        last_n_windows = count;
    }

    float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_z2 = 0.0f;

    /* Most-recent last_n_windows entries: start from (head - last_n_windows) */
    uint16_t start = (uint16_t)((s_acc_head - last_n_windows +
                                LIS3DHTR_ACC_BUF_SIZE) % LIS3DHTR_ACC_BUF_SIZE);

    for (uint16_t i = 0; i < last_n_windows; i++) {
        const lis3dhtr_window_t *w =
            &s_acc_buf[(start + i) % LIS3DHTR_ACC_BUF_SIZE];
        sum_x2 += w->x_rms_mg * w->x_rms_mg;
        sum_y2 += w->y_rms_mg * w->y_rms_mg;
        sum_z2 += w->z_rms_mg * w->z_rms_mg;
    }

    float fn = (float)last_n_windows;
    stats->x_rms60_mg = sqrtf(sum_x2 / fn);
    stats->y_rms60_mg = sqrtf(sum_y2 / fn);
    stats->z_rms60_mg = sqrtf(sum_z2 / fn);
    stats->magnitude_rms60_mg = sqrtf(
        stats->x_rms60_mg * stats->x_rms60_mg +
        stats->y_rms60_mg * stats->y_rms60_mg +
        stats->z_rms60_mg * stats->z_rms60_mg);
    stats->windows_captured = last_n_windows;

    return ESP_OK;
}

esp_err_t lis3dhtr_calibrate_gravity(uint16_t n_samples)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "calibrate_gravity: not initialised");
        return ESP_ERR_INVALID_STATE;
    }
    if (n_samples == 0) n_samples = 200;

    ESP_LOGI(TAG, "Gravity calibration: collecting %d samples...", n_samples);

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < n_samples; i++) {
        lis3dhtr_raw_t raw;
        esp_err_t ret = lis3dhtr_read_raw(&raw);
        if (ret == ESP_OK) {
            sum_x += (double)(raw.x >> 4) * (double)s_sensitivity_mg;
            sum_y += (double)(raw.y >> 4) * (double)s_sensitivity_mg;
            sum_z += (double)(raw.z >> 4) * (double)s_sensitivity_mg;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));   /* 5ms between samples = ~200Hz */
    }

    if (valid < n_samples / 2) {
        ESP_LOGE(TAG, "calibrate_gravity: too many read failures (%d/%d)",
                 valid, n_samples);
        return ESP_FAIL;
    }

    s_gravity_x_mg = (float)(sum_x / valid);
    s_gravity_y_mg = (float)(sum_y / valid);
    s_gravity_z_mg = (float)(sum_z / valid);
    s_gravity_calibrated = true;

    /* Identify dominant axis for logging */
    float ax = fabsf(s_gravity_x_mg);
    float ay = fabsf(s_gravity_y_mg);
    float az = fabsf(s_gravity_z_mg);
    const char *dominant = (az >= ax && az >= ay) ? "Z" :
                           (ay >= ax)              ? "Y" : "X";

    ESP_LOGI(TAG, "Gravity offsets (%d samples): X=%.1f Y=%.1f Z=%.1f mg "
             "(dominant=%s, magnitude=%.1f mg)",
             valid,
             s_gravity_x_mg, s_gravity_y_mg, s_gravity_z_mg,
             dominant,
             sqrtf(s_gravity_x_mg * s_gravity_x_mg +
                   s_gravity_y_mg * s_gravity_y_mg +
                   s_gravity_z_mg * s_gravity_z_mg));

    return ESP_OK;
}

void lis3dhtr_get_gravity_offsets(float *x_mg, float *y_mg, float *z_mg)
{
    if (x_mg) *x_mg = s_gravity_x_mg;
    if (y_mg) *y_mg = s_gravity_y_mg;
    if (z_mg) *z_mg = s_gravity_z_mg;
}