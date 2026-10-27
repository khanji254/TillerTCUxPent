#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "pins/pins.h"
#include "test_current_sensor.h"

static const char *TAG = "ADC_SIMPLE";

void test_current_sensor_run(void)
{
    // Initialize the ADC unit
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // PIN_ADC_REF = 4 (ADC1_CHANNEL_3), PIN_ADC_SHUNT = 5 (ADC1_CHANNEL_4)
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &chan_config));

    // Optional curve fitting calibration
    adc_cali_handle_t cali_handle = NULL;
    bool cali_valid = false;
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    cali_valid = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK);

    ESP_LOGI(TAG, "ADC Initialized. Reading raw single samples from GPIO%d (Ref) and GPIO%d (Shunt)...", 
             PIN_ADC_REF, PIN_ADC_SHUNT);

    while (1) {
        int raw1 = 0;
        int raw2 = 0;

        // Direct single-sample read (sample_count = 1)
        esp_err_t err1 = adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw1);
        esp_err_t err2 = adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw2);

        // Direct single-sample voltage conversion
        int mv1 = 0;
        int mv2 = 0;

        if (err1 == ESP_OK) {
            if (cali_valid) {
                adc_cali_raw_to_voltage(cali_handle, raw1, &mv1);
            } else {
                mv1 = (int)(raw1 * 3300 / 4095);
            }
        }

        if (err2 == ESP_OK) {
            if (cali_valid) {
                adc_cali_raw_to_voltage(cali_handle, raw2, &mv2);
            } else {
                mv2 = (int)(raw2 * 3300 / 4095);
            }
        }

        if (err1 != ESP_OK || err2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC channels!");
        } else {
            ESP_LOGI(TAG, "GPIO%d: %4d raw | %4d mV  ||  GPIO%d: %4d raw | %4d mV", 
                     PIN_ADC_REF, raw1, mv1, 
                     PIN_ADC_SHUNT, raw2, mv2);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}