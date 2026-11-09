/**
 * @file jkbms_serial_storage.h
 * @brief NVS storage for battery serial numbers with wear protection
 */

#ifndef BMS_SERIAL_STORAGE_H_
#define BMS_SERIAL_STORAGE_H_

#include "esp_err.h"
#include <stdbool.h> 
#include "bms_monitor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize NVS storage for battery serials
 * @return ESP_OK on success
 */
esp_err_t bms_serial_storage_init(void);

/**
 * @brief Load stored serial numbers from NVS
 * @param serials Array to store loaded serials [BMS_BATTERY_COUNT][16]
 * @return ESP_OK on success
 */
esp_err_t bms_serial_storage_load(char serials[BMS_BATTERY_COUNT][16]);

/**
 * @brief Save serial number to NVS (with wear protection)
 * @param battery_id Battery ID (0-2)
 * @param serial_no  Serial number string
 * @param force      Force write even if no change detected
 * @return ESP_OK on success
 */
esp_err_t bms_serial_storage_save(bms_battery_id_t battery_id, const char *serial_no, bool force);

/**
 * @brief Get last saved serial for a battery
 * @param battery_id Battery ID (0-2)
 * @param serial_out Buffer to store serial (min 16 bytes)
 * @return ESP_OK on success
 */
esp_err_t bms_serial_storage_get(bms_battery_id_t battery_id, char *serial_out);

/**
 * @brief Erase ALL stored serial numbers from NVS.
 * Called on triple-button-press NVS reset.
 * @return ESP_OK on success
 */
esp_err_t bms_serial_storage_clear_all(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_SERIAL_STORAGE_H_ */