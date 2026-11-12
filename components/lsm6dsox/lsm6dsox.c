/**
 * @file lsm6dsox.c
 * @brief LSM6DSOX 6-axis IMU Driver Implementation for KSC TCU 2.1
 */

#include "lsm6dsox.h"
#include "board_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "LSM6DSOX"

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static uint8_t s_active_addr = LSM6DSOX_I2C_ADDR_LOW;
static bool s_initialized = false;
static float s_sensitivity_mg = LSM6DSOX_SENS_MG_16G;
static lsm6dsox_mode_t s_mode = LSM6DSOX_MODE_PRODUCTION;
static bool s_impact_pending = false;

/* FIFO & Accumulator */
static bool s_fifo_enabled = false;
static lsm6dsox_window_t s_acc_buf[LSM6DSOX_ACC_BUF_SIZE];
static uint16_t s_acc_head  = 0;
static uint16_t s_acc_count = 0;
static SemaphoreHandle_t s_acc_mutex = NULL;

/* Gravity Calibration */
static float s_gravity_x_mg = 0.0f;
static float s_gravity_y_mg = 0.0f;
static float s_gravity_z_mg = 0.0f;
static bool  s_gravity_calibrated = false;

/* =========================================================================
 * LOW-LEVEL I2C PRIMITIVES
 * ========================================================================= */

static esp_err_t write_reg_addr(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    return write_reg_addr(s_active_addr, reg, value);
}

static esp_err_t read_regs_addr(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);   /* Repeated start */
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, dst, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dst + (len - 1), I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    return read_regs_addr(s_active_addr, reg, dst, len);
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

esp_err_t lsm6dsox_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (s_acc_mutex == NULL) {
        s_acc_mutex = xSemaphoreCreateMutex();
        if (!s_acc_mutex) {
            ESP_LOGE(TAG, "Failed to create accumulator mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Configure and install I2C master driver if not installed */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BOARD_I2C_SDA_PIN,
        .scl_io_num       = BOARD_I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = BOARD_I2C_FREQ_HZ,
    };

    i2c_param_config(BOARD_I2C_PORT, &conf);
    i2c_driver_install(BOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    /* Probe I2C address: check 0x6A first, then 0x6B */
    uint8_t who = 0;
    esp_err_t ret = read_regs_addr(LSM6DSOX_I2C_ADDR_LOW, LSM6DSOX_REG_WHO_AM_I, &who, 1);
    if (ret == ESP_OK && who == LSM6DSOX_WHO_AM_I_VAL) {
        s_active_addr = LSM6DSOX_I2C_ADDR_LOW;
        ESP_LOGI(TAG, "LSM6DSOX detected at address 0x%02X (WHO_AM_I = 0x%02X)", s_active_addr, who);
    } else {
        ret = read_regs_addr(LSM6DSOX_I2C_ADDR_HIGH, LSM6DSOX_REG_WHO_AM_I, &who, 1);
        if (ret == ESP_OK && who == LSM6DSOX_WHO_AM_I_VAL) {
            s_active_addr = LSM6DSOX_I2C_ADDR_HIGH;
            ESP_LOGI(TAG, "LSM6DSOX detected at address 0x%02X (WHO_AM_I = 0x%02X)", s_active_addr, who);
        } else {
            ESP_LOGE(TAG, "WHO_AM_I check failed! (0x6A: ret=%s, 0x6B: ret=%s, last_val=0x%02X)",
                     esp_err_to_name(ret), esp_err_to_name(ret), who);
            return ESP_ERR_NOT_FOUND;
        }
    }

    /* Step 1: Software Reset */
    write_reg(LSM6DSOX_REG_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Step 2: Enable Block Data Update (BDU = 1) and Auto-Increment (IF_INC = 1) */
    write_reg(LSM6DSOX_REG_CTRL3_C, 0x44);

    /* Step 3: Power down Gyroscope to conserve battery */
    write_reg(LSM6DSOX_REG_CTRL2_G, 0x00);

    /* Step 4: Configure Accelerometer ODR & Full-scale (208 Hz, ±16g initially) */
    /* CTRL1_XL: [7:4] ODR_XL = 0101 (208 Hz), [3:2] FS_XL = 01 (±16g) -> 0x54 */
    ret = write_reg(LSM6DSOX_REG_CTRL1_XL, 0x54);
    if (ret != ESP_OK) goto init_fail;
    s_sensitivity_mg = LSM6DSOX_SENS_MG_16G;
    s_mode = LSM6DSOX_MODE_PRODUCTION;

    /* Step 5: Configure INT1 pin for Motion/Wake-up Detection */
    /* Latched interrupt: TAP_CFG0 bit 0 (LIR) = 1 */
    write_reg(LSM6DSOX_REG_TAP_CFG0, 0x01);
    /* Basic interrupt enable: TAP_CFG2 bit 7 (INTERRUPTS_ENABLE) = 1 */
    write_reg(LSM6DSOX_REG_TAP_CFG2, 0x80);
    /* Wake-up threshold: WAKE_UP_THS: 1 LSB = 16g/64 = 250mg. 6 * 250mg = 1.5g */
    write_reg(LSM6DSOX_REG_WAKE_UP_THS, 0x06);
    /* Wake-up duration: 0 */
    write_reg(LSM6DSOX_REG_WAKE_UP_DUR, 0x00);
    /* Route wake-up & single-tap to INT1: MD1_CFG = 0x60 (INT1_WU=1, INT1_SINGLE_TAP=1) */
    write_reg(LSM6DSOX_REG_MD1_CFG, 0x60);

    /* Configure ESP32-S3 INT1 GPIO as input */
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << BOARD_ACCEL_INT1_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_cfg);

    s_initialized = true;
    ESP_LOGI(TAG, "LSM6DSOX initialised — 208 Hz, ±16g, INT1=IO%d", BOARD_ACCEL_INT1_PIN);
    return ESP_OK;

init_fail:
    ESP_LOGE(TAG, "LSM6DSOX configuration failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t lsm6dsox_check_who_am_i(uint8_t *value)
{
    if (value == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = read_regs(LSM6DSOX_REG_WHO_AM_I, value, 1);
    if (ret != ESP_OK) {
        /* If not initialized yet, try LOW address */
        ret = read_regs_addr(LSM6DSOX_I2C_ADDR_LOW, LSM6DSOX_REG_WHO_AM_I, value, 1);
        if (ret == ESP_OK && *value == LSM6DSOX_WHO_AM_I_VAL) {
            s_active_addr = LSM6DSOX_I2C_ADDR_LOW;
        } else {
            ret = read_regs_addr(LSM6DSOX_I2C_ADDR_HIGH, LSM6DSOX_REG_WHO_AM_I, value, 1);
            if (ret == ESP_OK && *value == LSM6DSOX_WHO_AM_I_VAL) {
                s_active_addr = LSM6DSOX_I2C_ADDR_HIGH;
            }
        }
    }
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X (expected 0x%02X) [%s]",
             *value, LSM6DSOX_WHO_AM_I_VAL,
             (*value == LSM6DSOX_WHO_AM_I_VAL) ? "PASS" : "FAIL");
    return (*value == LSM6DSOX_WHO_AM_I_VAL) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t lsm6dsox_read_raw(lsm6dsox_raw_t *raw)
{
    if (raw == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t buf[6];
    esp_err_t ret = read_regs(LSM6DSOX_REG_OUTX_L_A, buf, 6);
    if (ret != ESP_OK) return ret;

    /* Little-endian 16-bit 2's complement */
    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);
    return ESP_OK;
}

esp_err_t lsm6dsox_read_accel_mg(lsm6dsox_accel_mg_t *accel)
{
    if (accel == NULL) return ESP_ERR_INVALID_ARG;
    lsm6dsox_raw_t raw;
    esp_err_t ret = lsm6dsox_read_raw(&raw);
    if (ret != ESP_OK) return ret;

    accel->x_mg = (float)raw.x * s_sensitivity_mg;
    accel->y_mg = (float)raw.y * s_sensitivity_mg;
    accel->z_mg = (float)raw.z * s_sensitivity_mg;
    return ESP_OK;
}

esp_err_t lsm6dsox_read_motion(lsm6dsox_motion_t *motion)
{
    if (motion == NULL) return ESP_ERR_INVALID_ARG;
    memset(motion, 0, sizeof(lsm6dsox_motion_t));

    lsm6dsox_accel_mg_t accel;
    esp_err_t ret = lsm6dsox_read_accel_mg(&accel);
    if (ret != ESP_OK) return ret;

    motion->x_mg = accel.x_mg;
    motion->y_mg = accel.y_mg;
    motion->z_mg = accel.z_mg;

    /* Total vector magnitude */
    float mag_sq = accel.x_mg * accel.x_mg +
                   accel.y_mg * accel.y_mg +
                   accel.z_mg * accel.z_mg;
    motion->accel_mag_mg = sqrtf(mag_sq);
    motion->accel_rms_mg = motion->accel_mag_mg;

    /* Orientation angles */
    if (motion->accel_mag_mg > 100.0f) {
        motion->roll_deg  = atan2f(accel.y_mg, accel.z_mg) * (180.0f / (float)M_PI);
        motion->pitch_deg = atan2f(-accel.x_mg,
                                   sqrtf(accel.y_mg * accel.y_mg + accel.z_mg * accel.z_mg))
                            * (180.0f / (float)M_PI);
    }

    /* Motion status from WAKE_UP_SRC */
    uint8_t wu_src = 0;
    if (read_regs(LSM6DSOX_REG_WAKE_UP_SRC, &wu_src, 1) == ESP_OK) {
        motion->in_motion = (wu_src & 0x08) != 0; /* WU_IA bit 3 */
    }

    /* Impact status from TAP_SRC or shock magnitude */
    uint8_t tap_src = 0;
    if (read_regs(LSM6DSOX_REG_TAP_SRC, &tap_src, 1) == ESP_OK) {
        motion->impact_detected = (tap_src & 0x40) != 0; /* TAP_IA bit 6 */
    }
    if (motion->accel_mag_mg > 4500.0f) {
        motion->impact_detected = true;
    }
    if (motion->impact_detected) {
        s_impact_pending = true;
    }

    return ESP_OK;
}

bool lsm6dsox_is_in_motion(void)
{
    if (!s_initialized) return false;

    int gpio_level = gpio_get_level(BOARD_ACCEL_INT1_PIN);
    if (gpio_level == 0) return false;

    uint8_t wu_src = 0;
    if (read_regs(LSM6DSOX_REG_WAKE_UP_SRC, &wu_src, 1) != ESP_OK) return false;
    return (wu_src & 0x08) != 0;
}

bool lsm6dsox_check_clear_impact(void)
{
    if (!s_initialized) return false;
    uint8_t tap_src = 0;
    if (read_regs(LSM6DSOX_REG_TAP_SRC, &tap_src, 1) != ESP_OK) return false;
    return (tap_src & 0x40) != 0;
}

bool lsm6dsox_consume_impact(void)
{
    bool ret = s_impact_pending;
    s_impact_pending = false;
    return ret;
}

static esp_err_t fifo_reset_and_stream(void)
{
    /* 1. Bypass mode first to reset/flush stale FIFO contents */
    esp_err_t ret = write_reg(LSM6DSOX_REG_FIFO_CTRL4, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 2. Configure batching rate: Accelerometer at 208 Hz, Gyro disabled */
    /* FIFO_CTRL3: BDR_XL[3:0] = 0101 (208 Hz), BDR_GY[3:0] = 0000 */
    ret = write_reg(LSM6DSOX_REG_FIFO_CTRL3, 0x05);
    if (ret != ESP_OK) return ret;

    /* 3. Stream / Continuous mode: FIFO_MODE[2:0] = 110 (0x06) */
    ret = write_reg(LSM6DSOX_REG_FIFO_CTRL4, 0x06);
    if (ret != ESP_OK) return ret;

    lsm6dsox_accumulator_reset();
    return ESP_OK;
}

esp_err_t lsm6dsox_configure_mode(lsm6dsox_mode_t mode)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t ret;
    if (mode == LSM6DSOX_MODE_PRODUCTION) {
        /* PRODUCTION: ±2g, 208 Hz, HPF enabled for vibration tracking */
        ret = write_reg(LSM6DSOX_REG_CTRL1_XL, 0x50); /* ODR=208Hz, FS=±2g */
        if (ret != ESP_OK) return ret;
        /* Enable HPF on CTRL8_XL (bit 2 HP_SLOPE_XL_EN = 1) */
        ret = write_reg(LSM6DSOX_REG_CTRL8_XL, 0x04);
        if (ret != ESP_OK) return ret;
        s_sensitivity_mg = LSM6DSOX_SENS_MG_2G;
        s_mode = LSM6DSOX_MODE_PRODUCTION;
        /* Let the slope/HPF filter settle before batching into FIFO */
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "Mode -> PRODUCTION (208 Hz, ±2g, HPF on)");
    } else {
        /* TEST: ±16g, 208 Hz, HPF disabled (gravity visible) */
        ret = write_reg(LSM6DSOX_REG_CTRL1_XL, 0x54); /* ODR=208Hz, FS=±16g */
        if (ret != ESP_OK) return ret;
        /* Disable HPF on CTRL8_XL */
        ret = write_reg(LSM6DSOX_REG_CTRL8_XL, 0x00);
        if (ret != ESP_OK) return ret;
        s_sensitivity_mg = LSM6DSOX_SENS_MG_16G;
        s_mode = LSM6DSOX_MODE_TEST;
        ESP_LOGI(TAG, "Mode -> TEST (208 Hz, ±16g, HPF off, gravity visible)");
    }

    /* Flush FIFO if enabled so samples from the prior FS/filter aren't mis-scaled */
    if (s_fifo_enabled) {
        ret = fifo_reset_and_stream();
        if (ret != ESP_OK) return ret;
    }

    return ESP_OK;
}

lsm6dsox_mode_t lsm6dsox_get_mode(void)
{
    return s_mode;
}

void lsm6dsox_i2c_scan(void)
{
    ESP_LOGI(TAG, "--- I2C Bus Scan (port %d) ---", BOARD_I2C_PORT);
    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(BOARD_I2C_PORT, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X%s",
                     addr, (addr == s_active_addr) ? " <- LSM6DSOX active" : "");
            found++;
        }
    }
    ESP_LOGI(TAG, "--- End I2C Scan (total %d devices) ---", found);
}

esp_err_t lsm6dsox_fifo_enable(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = fifo_reset_and_stream();
    if (ret != ESP_OK) return ret;

    s_fifo_enabled = true;
    ESP_LOGI(TAG, "FIFO stream mode enabled — 208 Hz batching into Continuous FIFO");
    return ESP_OK;
}

esp_err_t lsm6dsox_fifo_drain_accumulate(void)
{
    if (!s_initialized || !s_fifo_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t status[2] = {0};
    if (read_regs(LSM6DSOX_REG_FIFO_STATUS1, status, sizeof(status)) != ESP_OK) {
        return ESP_FAIL;
    }

    /* LSM6DSOX FIFO_STATUS1/2: 10-bit unread word count in [9:0]; 0 == empty */
    uint16_t diff_fifo = (uint16_t)status[0] | (((uint16_t)(status[1] & 0x03)) << 8);
    if (diff_fifo == 0) {
        return ESP_OK;
    }

    if (status[1] & 0x40) {
        ESP_LOGW(TAG, "FIFO overrun detected (unread=%u words)", diff_fifo);
    }

    /* Drain all unread words in bursts of up to LSM6DSOX_FIFO_DEPTH (32).
     * At 208 Hz and a 160 ms drain period (~33.3 samples), a single 32-word cap
     * falls behind and causes continuous overrun. Bound to 512 words (full FIFO). */
    uint16_t remaining = (diff_fifo > 512) ? 512 : diff_fifo;
    const bool apply_sw_gravity_sub =
        s_gravity_calibrated && (s_mode == LSM6DSOX_MODE_TEST);

    static uint8_t buf[LSM6DSOX_FIFO_DEPTH * 7];
    float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_z2 = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float peak_x_abs = 0.0f, peak_y_abs = 0.0f, peak_z_abs = 0.0f;
    float peak_x_sgn = 0.0f, peak_y_sgn = 0.0f, peak_z_sgn = 0.0f;
    uint16_t valid_samples = 0;

    while (remaining > 0) {
        uint16_t n_words = (remaining > LSM6DSOX_FIFO_DEPTH) ? LSM6DSOX_FIFO_DEPTH : remaining;
        esp_err_t ret = read_regs(LSM6DSOX_REG_FIFO_DATA_TAG, buf, n_words * 7);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "FIFO burst read (%u words) failed: %s", n_words, esp_err_to_name(ret));
            return ret;
        }
        remaining -= n_words;

        for (uint16_t i = 0; i < n_words; i++) {
            const uint8_t *w = &buf[i * 7];
            uint8_t tag = w[0] >> 3;

            /* Verify accelerometer tag */
            if (tag != LSM6DSOX_FIFO_TAG_ACCEL && tag != 0x00) {
                continue;
            }

            int16_t raw_x = (int16_t)((w[2] << 8) | w[1]);
            int16_t raw_y = (int16_t)((w[4] << 8) | w[3]);
            int16_t raw_z = (int16_t)((w[6] << 8) | w[5]);

            float x = (float)raw_x * s_sensitivity_mg;
            float y = (float)raw_y * s_sensitivity_mg;
            float z = (float)raw_z * s_sensitivity_mg;

            /* Only subtract software gravity baseline when hardware HPF is off;
             * in PRODUCTION mode CTRL8_XL HPF already removes DC gravity. */
            if (apply_sw_gravity_sub) {
                x -= s_gravity_x_mg;
                y -= s_gravity_y_mg;
                z -= s_gravity_z_mg;
            }

            sum_x2 += x * x;
            sum_y2 += y * y;
            sum_z2 += z * z;
            sum_x  += x;
            sum_y  += y;
            sum_z  += z;

            float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);
            if (ax > peak_x_abs) { peak_x_abs = ax; peak_x_sgn = x; }
            if (ay > peak_y_abs) { peak_y_abs = ay; peak_y_sgn = y; }
            if (az > peak_z_abs) { peak_z_abs = az; peak_z_sgn = z; }

            /* Instant shock detection */
            if (sqrtf(x * x + y * y + z * z) > 5000.0f) {
                s_impact_pending = true;
            }

            valid_samples++;
        }
    }

    if (valid_samples == 0) return ESP_OK;

    float fn = (float)valid_samples;
    lsm6dsox_window_t win = {
        .x_rms_mg        = sqrtf(sum_x2 / fn),
        .y_rms_mg        = sqrtf(sum_y2 / fn),
        .z_rms_mg        = sqrtf(sum_z2 / fn),
        .x_mean_mg       = sum_x / fn,
        .y_mean_mg       = sum_y / fn,
        .z_mean_mg       = sum_z / fn,
        .x_max_abs_mg    = peak_x_abs,
        .y_max_abs_mg    = peak_y_abs,
        .z_max_abs_mg    = peak_z_abs,
        .x_max_signed_mg = peak_x_sgn,
        .y_max_signed_mg = peak_y_sgn,
        .z_max_signed_mg = peak_z_sgn,
        .n_samples       = (valid_samples > UINT8_MAX) ? UINT8_MAX : (uint8_t)valid_samples,
    };

    if (s_acc_mutex) {
        if (xSemaphoreTake(s_acc_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        s_acc_buf[s_acc_head] = win;
        s_acc_head = (s_acc_head + 1) % LSM6DSOX_ACC_BUF_SIZE;
        if (s_acc_count < LSM6DSOX_ACC_BUF_SIZE) s_acc_count++;
        xSemaphoreGive(s_acc_mutex);
    } else {
        s_acc_buf[s_acc_head] = win;
        s_acc_head = (s_acc_head + 1) % LSM6DSOX_ACC_BUF_SIZE;
        if (s_acc_count < LSM6DSOX_ACC_BUF_SIZE) s_acc_count++;
    }

    return ESP_OK;
}

esp_err_t lsm6dsox_get_publish_stats(lsm6dsox_publish_stats_t *stats)
{
    if (stats == NULL) return ESP_ERR_INVALID_ARG;
    memset(stats, 0, sizeof(lsm6dsox_publish_stats_t));

    if (s_acc_mutex && xSemaphoreTake(s_acc_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_acc_count == 0) {
        if (s_acc_mutex) xSemaphoreGive(s_acc_mutex);
        return ESP_OK;
    }

    float sum_x_rms2 = 0.0f, sum_y_rms2 = 0.0f, sum_z_rms2 = 0.0f;
    float peak_x_abs = 0.0f, peak_y_abs = 0.0f, peak_z_abs = 0.0f;
    float peak_x_sgn = 0.0f, peak_y_sgn = 0.0f, peak_z_sgn = 0.0f;

    for (uint16_t i = 0; i < s_acc_count; i++) {
        const lsm6dsox_window_t *w = &s_acc_buf[i];
        sum_x_rms2 += w->x_rms_mg * w->x_rms_mg;
        sum_y_rms2 += w->y_rms_mg * w->y_rms_mg;
        sum_z_rms2 += w->z_rms_mg * w->z_rms_mg;

        if (w->x_max_abs_mg > peak_x_abs) { peak_x_abs = w->x_max_abs_mg; peak_x_sgn = w->x_max_signed_mg; }
        if (w->y_max_abs_mg > peak_y_abs) { peak_y_abs = w->y_max_abs_mg; peak_y_sgn = w->y_max_signed_mg; }
        if (w->z_max_abs_mg > peak_z_abs) { peak_z_abs = w->z_max_abs_mg; peak_z_sgn = w->z_max_signed_mg; }
    }

    float fc = (float)s_acc_count;
    stats->x_rms60_mg         = sqrtf(sum_x_rms2 / fc);
    stats->y_rms60_mg         = sqrtf(sum_y_rms2 / fc);
    stats->z_rms60_mg         = sqrtf(sum_z_rms2 / fc);
    stats->magnitude_rms60_mg = sqrtf(stats->x_rms60_mg * stats->x_rms60_mg +
                                      stats->y_rms60_mg * stats->y_rms60_mg +
                                      stats->z_rms60_mg * stats->z_rms60_mg);

    stats->x_peak_abs_mg      = peak_x_abs;
    stats->y_peak_abs_mg      = peak_y_abs;
    stats->z_peak_abs_mg      = peak_z_abs;
    stats->x_peak_signed_mg   = peak_x_sgn;
    stats->y_peak_signed_mg   = peak_y_sgn;
    stats->z_peak_signed_mg   = peak_z_sgn;
    stats->windows_captured   = s_acc_count;

    if (s_acc_mutex) xSemaphoreGive(s_acc_mutex);
    return ESP_OK;
}

void lsm6dsox_accumulator_reset(void)
{
    if (s_acc_mutex && xSemaphoreTake(s_acc_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_acc_head  = 0;
        s_acc_count = 0;
        xSemaphoreGive(s_acc_mutex);
    } else {
        s_acc_head  = 0;
        s_acc_count = 0;
    }
}

esp_err_t lsm6dsox_get_snapshot_rms(lsm6dsox_publish_stats_t *stats, uint16_t last_n_windows)
{
    if (stats == NULL) return ESP_ERR_INVALID_ARG;
    memset(stats, 0, sizeof(lsm6dsox_publish_stats_t));

    if (s_acc_mutex && xSemaphoreTake(s_acc_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_acc_count == 0) {
        if (s_acc_mutex) xSemaphoreGive(s_acc_mutex);
        return ESP_OK;
    }

    uint16_t n = (last_n_windows > s_acc_count) ? s_acc_count : last_n_windows;
    if (n == 0) {
        if (s_acc_mutex) xSemaphoreGive(s_acc_mutex);
        return ESP_OK;
    }

    float sum_x_rms2 = 0.0f, sum_y_rms2 = 0.0f, sum_z_rms2 = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        int idx = ((int)s_acc_head - 1 - (int)i + (int)LSM6DSOX_ACC_BUF_SIZE) % (int)LSM6DSOX_ACC_BUF_SIZE;
        const lsm6dsox_window_t *w = &s_acc_buf[idx];
        sum_x_rms2 += w->x_rms_mg * w->x_rms_mg;
        sum_y_rms2 += w->y_rms_mg * w->y_rms_mg;
        sum_z_rms2 += w->z_rms_mg * w->z_rms_mg;
    }

    float fn = (float)n;
    stats->x_rms60_mg         = sqrtf(sum_x_rms2 / fn);
    stats->y_rms60_mg         = sqrtf(sum_y_rms2 / fn);
    stats->z_rms60_mg         = sqrtf(sum_z_rms2 / fn);
    stats->magnitude_rms60_mg = sqrtf(stats->x_rms60_mg * stats->x_rms60_mg +
                                      stats->y_rms60_mg * stats->y_rms60_mg +
                                      stats->z_rms60_mg * stats->z_rms60_mg);
    stats->windows_captured   = n;

    if (s_acc_mutex) xSemaphoreGive(s_acc_mutex);
    return ESP_OK;
}

esp_err_t lsm6dsox_calibrate_gravity(uint16_t n_samples)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (n_samples == 0) n_samples = 200;

    ESP_LOGI(TAG, "Calibrating gravity (%u samples)...", n_samples);

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < n_samples; i++) {
        lsm6dsox_raw_t raw;
        if (lsm6dsox_read_raw(&raw) == ESP_OK) {
            sum_x += (float)raw.x * s_sensitivity_mg;
            sum_y += (float)raw.y * s_sensitivity_mg;
            sum_z += (float)raw.z * s_sensitivity_mg;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid < (n_samples / 2)) {
        ESP_LOGE(TAG, "Gravity calibration failed: only %u valid readings", valid);
        return ESP_FAIL;
    }

    s_gravity_x_mg = sum_x / (float)valid;
    s_gravity_y_mg = sum_y / (float)valid;
    s_gravity_z_mg = sum_z / (float)valid;
    s_gravity_calibrated = true;

    ESP_LOGI(TAG, "Gravity offsets calibrated: X=%.1f mg, Y=%.1f mg, Z=%.1f mg",
             s_gravity_x_mg, s_gravity_y_mg, s_gravity_z_mg);
    return ESP_OK;
}

void lsm6dsox_get_gravity_offsets(float *x_mg, float *y_mg, float *z_mg)
{
    if (x_mg) *x_mg = s_gravity_x_mg;
    if (y_mg) *y_mg = s_gravity_y_mg;
    if (z_mg) *z_mg = s_gravity_z_mg;
}
