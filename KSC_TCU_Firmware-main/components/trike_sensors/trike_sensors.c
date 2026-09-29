/**
 * @file trike_sensors.c
 * @brief Trike Analogue Sensor Readings — Voltage, Current, Temperature
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#include "trike_sensors.h"
#include "board_config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define TAG "TRIKE_SENSORS"

/* =========================================================================
 * NVS KEYS FOR CALIBRATION PERSISTENCE
 * ========================================================================= */

#define SENSORS_NVS_NAMESPACE    "trike_sens"
#define SENSORS_NVS_KEY_SHUNT    "shunt_ohms"
#define TRIKE_SENSORS_NVS_KEY_OFFSET "cur_offset"
#define SENSORS_NVS_KEY_TOFFSET  "temp_offset"

//#define SENSORS_NVS_KEY_CURR_OFFSET  "curr_offset"
static float s_current_offset_a = BOARD_CURRENT_ZERO_OFFSET_A;
static bool  s_board_offset_measured  = false;

/* =========================================================================
 * MODULE STATE
 * ========================================================================= */

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t         s_cali_handle  = NULL;
static bool                      s_cali_valid   = false;
static bool                      s_initialized  = false;
static float                     s_shunt_ohms   = BOARD_SHUNT_RESISTANCE_OHMS;
static float                     s_temp_offset_c = 0.0f;

/* =========================================================================
 * INTERNAL HELPERS
 * ========================================================================= */

/**
 * @brief Take BOARD_ADC_OVERSAMPLE_COUNT raw samples from an ADC1 channel
 *        and return the arithmetic mean as a calibrated millivolt value.
 *
 * Uses esp_adc_cal_raw_to_voltage() for per-chip eFuse calibration.
 * Samples are taken back-to-back; the MCU is not yielded during sampling
 * so the total acquisition window is approximately:
 *   64 samples × ~10 µs/sample = ~640 µs
 *
 * This is safe to call from any task context.
 *
 * @param[in]  channel   ADC1 channel to sample.
 * @param[out] mv_out    Pointer to receive the averaged voltage in mV.
 * @param[out] raw_out    Raw averaged ADC count (0-4095). Pass NULL to ignore.
 * @return ESP_OK on success, ESP_FAIL if any sample returns -1.
 */
static esp_err_t sample_adc1_mv(adc_channel_t channel, float *mv_out, int   *raw_out)
{
    int sum = 0;
    for (int i = 0; i < BOARD_ADC_OVERSAMPLE_COUNT; i++) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc1_handle, channel, &raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read error on channel %d: %s",
                     channel, esp_err_to_name(ret));
            return ret;
        }
        sum += raw;
    }
    int avg_raw = sum / BOARD_ADC_OVERSAMPLE_COUNT;
    
    /* Return raw count to caller if requested */
    if (raw_out != NULL) {
        *raw_out = avg_raw;
    }    

    if (s_cali_valid) {
        int mv = 0;
        adc_cali_raw_to_voltage(s_cali_handle, avg_raw, &mv);
        *mv_out = (float)mv;
    } else {
        /* Fallback linear conversion when eFuse calibration unavailable */
        *mv_out = (float)avg_raw * (BOARD_ADC_VMAX_MV / (float)BOARD_ADC_MAX_RAW);
    }
    return ESP_OK;
}

/**
 * @brief Load calibration values from NVS.
 *
 * Silently uses board_config.h defaults if the NVS namespace does not
 * yet exist (first boot scenario).
 */
static void load_calibration_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(SENSORS_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "No calibration NVS — using defaults");
        return;
    }
    uint32_t shunt_uohm = 0;
    if (nvs_get_u32(h, SENSORS_NVS_KEY_SHUNT, &shunt_uohm) == ESP_OK) {
        s_shunt_ohms = (float)shunt_uohm / 1000000.0f;
    }
    
    /* Single NVS key for current offset — stored as integer milliamps */
    int32_t offset_ma = 0;
    if (nvs_get_i32(h, TRIKE_SENSORS_NVS_KEY_OFFSET, &offset_ma) == ESP_OK) {
        s_current_offset_a = (float)offset_ma / 1000.0f;
        s_board_offset_measured = true;
        ESP_LOGI(TAG, "Board current offset loaded from NVS: %.3fA",
                 s_current_offset_a);
    } else {
        /* First boot or NVS erased — start from pre-configured constant */
        s_current_offset_a = BOARD_CURRENT_ZERO_OFFSET_A;
        ESP_LOGI(TAG, "No board offset in NVS — using pre-configured: %.3fA",
                 BOARD_CURRENT_ZERO_OFFSET_A);
    }
    
    int32_t toffset_mdeg = 0;
    if (nvs_get_i32(h, SENSORS_NVS_KEY_TOFFSET, &toffset_mdeg) == ESP_OK) {
        s_temp_offset_c = (float)toffset_mdeg / 1000.0f;
    }
    
/*	int32_t curr_offset_ua = 0;
	if (nvs_get_i32(h, SENSORS_NVS_KEY_CURR_OFFSET,
	                &curr_offset_ua) == ESP_OK) {
	    s_current_offset_a = (float)curr_offset_ua / 1000000.0f;
	    ESP_LOGI(TAG, "Current offset loaded: %.6f A", s_current_offset_a);
	}*/
       
    nvs_close(h);
}

/**
 * @brief Persist current calibration values to NVS.
 *
 * @return ESP_OK on success.
 */
static esp_err_t save_calibration_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(SENSORS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    nvs_set_u32(h, SENSORS_NVS_KEY_SHUNT,
                (uint32_t)(s_shunt_ohms * 1000000.0f));
    nvs_set_i32(h, SENSORS_NVS_KEY_TOFFSET,
                (int32_t)(s_temp_offset_c * 1000.0f));
    /* Current offset is saved exclusively via trike_sensors_update_board_offset()
     * using TRIKE_SENSORS_NVS_KEY_OFFSET — not here, to avoid overwriting
     * a BMS-calibrated value with a stale default on next shunt recalibration. */

	nvs_set_i32(h, TRIKE_SENSORS_NVS_KEY_OFFSET,
	            (int32_t)(s_current_offset_a * 1000.0f));                
    ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

esp_err_t trike_sensors_init(void)
{
    if (s_initialized) return ESP_OK;
    
    // Calibrate shunt resistance and current offset in amps
	trike_sensors_calibrate_shunt(BOARD_SHUNT_RESISTANCE_OHMS);
	
	/**
	* @TODO: If running this function BELOW: trike_sensors_calibrate_current_offset(BOARD_CURRENT_ZERO_OFFSET_A), remember to uncomment  save current offset in NVS in save_calibration_to_nvs()
	*/
	trike_sensors_calibrate_current_offset(BOARD_CURRENT_ZERO_OFFSET_A); 

    /* Create ADC1 oneshot unit */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle));

    /* Configure each channel */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BOARD_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,   /* 12-bit on ESP32-S3 */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                    BOARD_ADC_VOLTAGE_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                    BOARD_ADC_CURRENT_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle,
                    BOARD_ADC_TEMP_CH,    &chan_cfg));

    /* Calibration — try curve fitting first, fall back to line fitting */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = BOARD_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(
                             &cali_cfg, &s_cali_handle);
    if (cali_ret == ESP_OK) {
        s_cali_valid = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
    } else {
        s_cali_valid = false;
        ESP_LOGW(TAG, "ADC curve fitting calibration unavailable — using linear approx");
    }
    load_calibration_from_nvs();
    
      
    s_initialized = true;
    ESP_LOGI(TAG, "Trike sensors initialised");
    return ESP_OK;
}

esp_err_t trike_sensors_read_voltage(trike_sensor_data_t *data,
                                      float               *voltage_v)
{
    if (!s_initialized || data == NULL || voltage_v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float adc_mv = 0.0f;
    esp_err_t ret = sample_adc1_mv(BOARD_ADC_VOLTAGE_CH,
                                    &adc_mv,
                                    &data->raw_voltage);
    if (ret != ESP_OK) {
        return ret;
    }

    data->adc_mv_voltage = adc_mv;

    /*
     * Reverse the resistive voltage divider:
     *   Vin = Vadc_V x (R_upper + R_lower) / R_lower
     *       = Vadc_V x BOARD_VOLT_SCALE_FACTOR
     */
     if(adc_mv < 1350.00f){		
		 // BOARD_VOLT_SCALE_FACTOR set to 40.4215f for low mv values between 40-55V Actual voltage
		*voltage_v = (adc_mv / 1000.0f) * BOARD_VOLT_SCALE_FACTOR_LOW_MV ; 
	 }
     else{*voltage_v = (adc_mv / 1000.0f) * BOARD_VOLT_SCALE_FACTOR;}

    ESP_LOGI(TAG, "Voltage: raw=%d counts  adc=%.1f mV  Vin=%.2f V",
             data->raw_voltage, adc_mv, *voltage_v);
    return ESP_OK;
}

esp_err_t trike_sensors_read_current(trike_sensor_data_t *data,
                                      float               *current_a)
{
    if (!s_initialized || data == NULL || current_a == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float adc_mv = 0.0f;
    esp_err_t ret = sample_adc1_mv(BOARD_ADC_CURRENT_CH,
                                    &adc_mv,
                                    &data->raw_current);
    if (ret != ESP_OK) {
        return ret;
    }

    data->adc_mv_current = adc_mv;

    /*
     * Step 1 — Remove REF midpoint to get shunt-only signal:
     *   delta_mv = adc_mv - Vref
     *   At zero current: delta_mv = 0
     *   Positive current (discharge): delta_mv > 0
     *   Negative current (charge):    delta_mv < 0
     */
    float delta_mv = adc_mv - BOARD_CURRENT_VREF_MV;

    /*
     * Step 2 — Convert delta to current:
     *   I = delta_mv / (Gain x R_shunt x 1000)
     *
     *   During office test:  Gain=20, R_shunt=1.0 Ω  → divisor = 20000
     *   In production:       Gain=20, R_shunt=0.00025 → divisor = 5
     */
    float raw_current_a = delta_mv /
                          (BOARD_INA240_GAIN *
                           s_shunt_ohms *
                           1000.0f);

    /*
     * Step 3 — Apply zero-offset correction (NVS-stored):
     *   Determined by shorting J3 and calling
     *   trike_sensors_calibrate_current_offset(-observed_reading)
     */
   // *current_a = raw_current_a + s_current_offset_a;
    *current_a = (raw_current_a * 0.97) + s_current_offset_a;
    data->current_offset_applied_a = s_current_offset_a;

    ESP_LOGI(TAG,
             "Current: raw=%d  adc=%.2f mV  delta=%.3f mV"
             "  raw_I=%.5f A  offset=%.5f A  final_I=%.5f A"
             "  shunt=%.6f Ω",
             data->raw_current, adc_mv, delta_mv,
             raw_current_a, s_current_offset_a, *current_a,
             s_shunt_ohms);

    return ESP_OK;
}

esp_err_t trike_sensors_read_temperature(trike_sensor_data_t *data,
                                          float               *temp_c)
{
    if (!s_initialized || data == NULL || temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float adc_mv = 0.0f;
    esp_err_t ret = sample_adc1_mv(BOARD_ADC_TEMP_CH,
                                    &adc_mv,
                                    &data->raw_temp);
    if (ret != ESP_OK) {
        return ret;
    }

    data->adc_mv_temp = adc_mv;

    /*
     * NTC thermistor voltage divider:
     *   Vout = VDDA x R_ntc / (R_pullup + R_ntc)
     *   R_ntc = R_pullup x Vout / (VDDA - Vout)
     *
     * Guard against divide-by-zero at ADC rail limits.
     */
    if (adc_mv <= 0.0f || adc_mv >= BOARD_NTC_VDDA_MV) {
        ESP_LOGW(TAG, "NTC ADC out of range: raw=%d counts  adc=%.1f mV",
                 data->raw_temp, adc_mv);
        *temp_c = -99.0f;
        return ESP_OK;
    }

    float r_ntc = BOARD_NTC_R_PULLUP_OHMS * adc_mv /
                  (BOARD_NTC_VDDA_MV - adc_mv);

/*    
     * Steinhart-Hart Beta equation:
     *   1/T = 1/T0 + (1/B) x ln(R_ntc / R0)
     
    float inv_t = (1.0f / BOARD_NTC_T0_KELVIN) +
                  (1.0f / BOARD_NTC_BETA) *
                  logf(r_ntc / BOARD_NTC_R0_OHMS);
    *temp_c = (1.0f / inv_t - 273.15f) + s_temp_offset_c;*/
    
	/* The full three-coefficient Steinhart-Hart equation fits the NTC curve accurately across the full range:
	1/T = A + B×ln(R) + C×(ln(R))³  */ 
	float ln_r   = logf(r_ntc);
	float inv_T  = BOARD_NTC_SH_A
	             + BOARD_NTC_SH_B * ln_r
	             + BOARD_NTC_SH_C * (ln_r * ln_r * ln_r);
	*temp_c = (1.0f / inv_T - 273.15f) + s_temp_offset_c;    

    ESP_LOGI(TAG, "Temp: raw=%d counts  adc=%.1f mV"
             "  R_ntc=%.0f ohms  T=%.2f degC",
             data->raw_temp, adc_mv, r_ntc, *temp_c);
    return ESP_OK;
}

esp_err_t trike_sensors_read_all(trike_sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(data, 0, sizeof(trike_sensor_data_t));

    esp_err_t rv = trike_sensors_read_voltage(data,    &data->voltage_v);
    esp_err_t ri = trike_sensors_read_current(data,    &data->current_a);
    esp_err_t rt = trike_sensors_read_temperature(data, &data->temperature_c);

    data->valid = (rv == ESP_OK && ri == ESP_OK && rt == ESP_OK);

    if (!data->valid) {
        ESP_LOGW(TAG, "Sensor read errors — V:%s  I:%s  T:%s",
                 esp_err_to_name(rv),
                 esp_err_to_name(ri),
                 esp_err_to_name(rt));
    }

    return data->valid ? ESP_OK : ESP_FAIL;
}

esp_err_t trike_sensors_calibrate_shunt(float shunt_ohms)
{
    if (shunt_ohms <= 0.0f || shunt_ohms > 1.0f) {
        ESP_LOGE(TAG, "Invalid shunt value %.6f Ω (must be 0 < R ≤ 1 Ω)",
                 shunt_ohms);
        return ESP_ERR_INVALID_ARG;
    }
    s_shunt_ohms = shunt_ohms;
    ESP_LOGI(TAG, "Shunt calibrated to %.6f Ω", s_shunt_ohms);
    return save_calibration_to_nvs();
}

esp_err_t trike_sensors_calibrate_temp_offset(float offset_deg_c)
{
    s_temp_offset_c = offset_deg_c;
    ESP_LOGI(TAG, "Temperature offset calibrated to %.3f °C", s_temp_offset_c);
    return save_calibration_to_nvs();
}

float trike_sensors_get_shunt_ohms(void)
{
    return s_shunt_ohms;
}

float trike_sensors_get_temp_offset(void)
{
    return s_temp_offset_c;
}

esp_err_t trike_sensors_calibrate_current_offset(float offset_a)
{
    s_current_offset_a = offset_a;
    ESP_LOGI(TAG, "Current offset set to %.6f A — saving to NVS",
             s_current_offset_a);
    return save_calibration_to_nvs();
}

float trike_sensors_get_current_offset(void)
{
    return s_current_offset_a;
}

void trike_sensors_update_board_offset(float bms_total_current_a)
{
    /* Guard — only update once per boot. NVS write wear protection.
     * The flag is checked here in addition to the caller's first_boot
     * guard, making this function safe even if called accidentally again. */
    static bool s_offset_set_this_boot = false;
    if (s_offset_set_this_boot) {
        ESP_LOGD(TAG, "Board offset already set this boot — skipping");
        return;
    }

    /* Read raw ADC current WITHOUT applying the existing offset.
     * We need the raw ADC-derived current to compute the true error. */
    float adc_mv = 0.0f;
    trike_sensor_data_t tmp = {0};
    if (sample_adc1_mv(BOARD_ADC_CURRENT_CH, &adc_mv, &tmp.raw_current) != ESP_OK) {
        ESP_LOGW(TAG, "Cannot update board offset — ADC read failed");
        return;
    }

    /* Compute raw current exactly as trike_sensors_read_current() does,
     * but WITHOUT subtracting s_current_offset_a */
    float delta_mv      = adc_mv - BOARD_CURRENT_VREF_MV;
    float raw_current_a = delta_mv / (BOARD_INA240_GAIN *
                                       s_shunt_ohms * 1000.0f);

    /* True offset = what BMS measures minus what raw ADC measures.
     * At zero current ideally: bms_total = 0, raw_current_a = some drift value.
     * new_offset is what needs to be SUBTRACTED from raw_current_a to get 0.
     * Convention matches trike_sensors_read_current():
     *   final_current = raw_current_a - new_offset  → 0 at rest */
    //float new_offset = raw_current_a - bms_total_current_a;
    float new_offset = bms_total_current_a - raw_current_a;

    /* Sanity clamp — reject implausible values */
    if (new_offset < -15.0f || new_offset > 15.0f) {
        ESP_LOGW(TAG, "Board offset %.3fA out of plausible range (±15A) — rejected",
                 new_offset);
        return;
    }

    float old_offset   = s_current_offset_a;
    s_current_offset_a = new_offset;
    s_board_offset_measured = true;
    s_offset_set_this_boot  = true;

    ESP_LOGI(TAG,
             "Board offset updated: %.3fA → %.3fA "
             "(raw_ADC=%.3fA  BMS_total=%.3fA  adc_mv=%.2fmV)",
             old_offset, new_offset, raw_current_a, bms_total_current_a, adc_mv);

    /* Persist to NVS — only happens once per boot */
    nvs_handle_t h;
    if (nvs_open(SENSORS_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        int32_t offset_ma = (int32_t)(new_offset * 1000.0f);
        nvs_set_i32(h, TRIKE_SENSORS_NVS_KEY_OFFSET, offset_ma);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Board offset persisted to NVS: %ldmA", (long)offset_ma);
    }
}

/*void trike_sensors_update_board_offset(float bms_total_current_a)
{
     Read current ADC measurement using the current (pre-existing) offset 
    trike_sensor_data_t sensors;
    if (trike_sensors_read_all(&sensors) != ESP_OK || !sensors.valid) {
        ESP_LOGW(TAG, "Cannot update board offset — ADC read failed");
        return;
    }

     new_offset = adc_measured - bms_total
     * This is the systematic error the ADC has relative to BMS ground truth 
    float new_offset = bms_total_current_a - sensors.current_a;

     Sanity check — reject if implausible (>15A offset is likely a bad read) 
    if (new_offset < -15.0f || new_offset > 15.0f) {
        ESP_LOGW(TAG,
                 "Board offset %.3fA out of plausible range — rejected",
                 new_offset);
        return;
    }

    float old_offset = s_current_offset_a;
    s_current_offset_a = new_offset;
    s_board_offset_measured  = true;

    ESP_LOGI(TAG,
             "Board current offset updated: %.3fA → %.3fA "
             "(ADC=%.3fA BMS_total=%.3fA)",
             old_offset, new_offset,
             sensors.current_a, bms_total_current_a);

     Persist to NVS 
    nvs_handle_t h;
    if (nvs_open(SENSORS_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
         Store as integer milliamps to avoid float NVS storage issues 
        int32_t offset_ma = (int32_t)(new_offset * 1000.0f);
        nvs_set_i32(h, TRIKE_SENSORS_NVS_KEY_OFFSET, offset_ma);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Board offset saved to NVS: %ldmA", (long)offset_ma);
    }
}*/

float trike_sensors_get_precfg_offset(void)
{
    return BOARD_CURRENT_ZERO_OFFSET_A;
}