/**
 * @file trike_sensors.h
 * @brief Trike Analogue Sensor Readings — Voltage, Current, Temperature
 *
 * Provides calibrated ADC readings for:
 *   - Trike pack voltage via resistive voltage divider
 *   - Trike pack current via INA240A1PWR current-sense amplifier
 *   - Powertrain temperature via NTC thermistor
 *
 * All three channels use ADC1 to avoid the Wi-Fi / ADC2 conflict on
 * ESP32-S3.  The ESP IDF esp_adc_cal library is used for hardware and
 * software calibration to achieve the best possible accuracy.
 *
 * Calibration offsets for both current (shunt resistance) and temperature
 * (NTC offset) are stored in NVS and can be updated at runtime without
 * a firmware reflash.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef TRIKE_SENSORS_H_
#define TRIKE_SENSORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * DATA STRUCTURE
 * ========================================================================= */

/**
 * @brief Snapshot of all three analogue sensor readings.
 *
 * Engineering-unit values are the primary output for JSON publishing.
 * Raw ADC counts are provided for factory calibration, fault diagnosis,
 * and offline recalculation if circuit parameters change in the field.
 *
 * Raw count range: 0 to BOARD_ADC_MAX_RAW (4095) for 12-bit ADC.
 * Each count represents BOARD_ADC_VMAX_MV / BOARD_ADC_MAX_RAW = ~0.757 mV
 * before calibration correction.
 */
typedef struct {
    /* Engineering-unit values (primary — use these for publishing) */
    float    voltage_v;       /**< Trike pack voltage (Volts)                */
    float    current_a;       /**< Trike pack current (Amps, +ve = discharge) */
    float    temperature_c;   /**< Powertrain temperature (degrees Celsius)   */
    bool     valid;           /**< true if all three readings succeeded       */

    /* Raw ADC counts (secondary — use for calibration and diagnostics)    */
    int      raw_voltage;     /**< Averaged raw ADC count, voltage channel    */
    int      raw_current;     /**< Averaged raw ADC count, current channel    */
    int      raw_temp;        /**< Averaged raw ADC count, temperature channel */
    float    adc_mv_voltage;  /**< Calibrated millivolts, voltage channel     */
    float    adc_mv_current;  /**< Calibrated millivolts, current channel     */
    float    adc_mv_temp;     /**< Calibrated millivolts, temperature channel  */

    /* NEW — offset diagnostics */
    float    current_offset_applied_a;  /**< Offset subtracted from raw reading */

} trike_sensor_data_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise ADC hardware and load calibration values from NVS.
 *
 * Configures ADC1 width, per-channel attenuation, and runs the ESP IDF
 * esp_adc_cal characterisation for accurate voltage conversion.
 * Loads shunt resistance and temperature offset from NVS (uses defaults
 * from board_config.h on first boot).
 *
 * Must be called once before any trike_sensors_read_*() call.
 *
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_init(void);

/**
 * @brief Read all three sensor channels in one call.
 *
 * Performs BOARD_ADC_OVERSAMPLE_COUNT samples per channel and returns
 * averaged, calibrated, engineering-unit values.
 *
 * @param[out] data  Pointer to structure to receive the readings.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if data is NULL.
 */
esp_err_t trike_sensors_read_all(trike_sensor_data_t *data);

/**
 * @brief Read trike pack voltage, storing both the raw ADC count and
 *        the calibrated engineering-unit result.
 *
 * @param[out] data       Struct whose raw_voltage and adc_mv_voltage
 *                        fields are populated by this call.
 * @param[out] voltage_v  Trike pack voltage in Volts.
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_read_voltage(trike_sensor_data_t *data,
                                      float               *voltage_v);

/**
 * @brief Read trike pack current, storing both the raw ADC count and
 *        the calibrated engineering-unit result.
 *
 * @param[out] data       Struct whose raw_current and adc_mv_current
 *                        fields are populated by this call.
 * @param[out] current_a  Trike pack current in Amps (+ve = discharge).
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_read_current(trike_sensor_data_t *data,
                                      float               *current_a);

/**
 * @brief Read powertrain temperature, storing both the raw ADC count
 *        and the calibrated engineering-unit result.
 *
 * @param[out] data    Struct whose raw_temp and adc_mv_temp fields
 *                     are populated by this call.
 * @param[out] temp_c  Temperature in degrees Celsius.
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_read_temperature(trike_sensor_data_t *data,
                                          float               *temp_c);

/**
 * @brief Update shunt resistance calibration value and persist to NVS.
 *
 * Call this after measuring the actual shunt resistance with a reference
 * instrument.  The new value takes effect immediately.
 *
 * @param[in] shunt_ohms  Actual shunt resistance in Ohms (e.g. 0.00095).
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_calibrate_shunt(float shunt_ohms);

/**
 * @brief Update NTC temperature offset and persist to NVS.
 *
 * Add a signed offset (degrees C) to the computed temperature to correct
 * for thermistor tolerance or mounting position error.
 *
 * @param[in] offset_deg_c  Signed offset in degrees C (e.g. -2.5).
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_calibrate_temp_offset(float offset_deg_c);

/**
 * @brief Return the currently active shunt resistance (Ohms).
 *
 * @return Shunt resistance value in use.
 */
float trike_sensors_get_shunt_ohms(void);

/**
 * @brief Return the currently active temperature calibration offset (°C).
 *
 * @return Temperature offset in degrees Celsius.
 */
float trike_sensors_get_temp_offset(void);

/**
 * @brief Set and persist a current zero-offset correction.
 *
 * Measure with J3 shorted, read firmware value, pass negative of
 * that value here. Stored in NVS, survives reboot.
 *
 * @param[in] offset_a  Value to SUBTRACT from every raw current reading.
 * @return ESP_OK on success.
 */
esp_err_t trike_sensors_calibrate_current_offset(float offset_a);

/**
 * @brief Return the currently active current offset (A).
 */
float trike_sensors_get_current_offset(void);

/* trike_sensors.h — add these declarations */

/**
 * @brief Update the per-board current offset from BMS total current.
 * Called once all 3 batteries have been successfully read.
 * @param bms_total_current_a  Sum of all battery currents in Amperes.
 */
void trike_sensors_update_board_offset(float bms_total_current_a);

/**
 * @brief Get the pre-configured (hardcoded) current offset constant.
 */
float trike_sensors_get_precfg_offset(void);

#ifdef __cplusplus
}
#endif

#endif /* TRIKE_SENSORS_H_ */
