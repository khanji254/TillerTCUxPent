/**
 * @file tcu_nvs_creds.h
 * @brief KSC TCU V1.1 — NVS Credential Storage for MQTT Provisioning
 *
 * Manages persistent MQTT credentials obtained via ThingsBoard device
 * provisioning.  Credentials are stored in a dedicated NVS namespace
 * "tcu_creds" so they survive power cycles but are isolated from other
 * NVS data.
 *
 * Key design decisions:
 *  - A separate "prov_done" flag is the single source of truth for whether
 *    provisioning has been completed.  Credentials without this flag are
 *    treated as invalid.
 *  - All reads/writes are atomic from the caller's perspective (each
 *    function opens and closes the namespace handle internally).
 *  - NVS corruption is detected by read failures, triggering a full erase
 *    and re-provisioning via the fallback path.
 *
 * @author  Mary Mbugua
 * @date    2026-04-10
 */

#ifndef TCU_NVS_CREDS_H_
#define TCU_NVS_CREDS_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * NVS NAMESPACE AND KEY NAMES
 * ========================================================================= */

#define TCU_NVS_NAMESPACE       "tcu_creds"

#define TCU_NVS_KEY_PROV_DONE   "prov_done"   /**< uint8:  1 = provisioned    */
#define TCU_NVS_KEY_USERNAME    "username"     /**< str:   MQTT username       */
#define TCU_NVS_KEY_PASSWORD    "password"     /**< str:   MQTT password       */
#define TCU_NVS_KEY_CLIENT_ID   "client_id"   /**< str:   MQTT client ID      */
#define TCU_NVS_KEY_DEVICE_NAME "device_name" /**< str:   TB device name      */

/* Maximum credential string lengths (including null terminator) */
#define TCU_CRED_MAX_LEN        64

/*Define TCU device names*/
#define TCU_DEVICE_1 	"TCU_0001"
#define TCU_DEVICE_2 	"TCU_0002"
#define TCU_DEVICE_3 	"TCU_0003"
#define TCU_DEVICE_4 	"TCU_0004"
#define TCU_DEVICE_5	"TCU_0005"
#define TCU_DEVICE_6 	"TCU_0006"
#define TCU_DEVICE_7 	"TCU_0007"
#define TCU_DEVICE_8 	"TCU_0008"
#define TCU_DEVICE_9 	"TCU_0009"
#define TCU_DEVICE_10 	"TCU_0010"

/* =========================================================================
 * CREDENTIAL STRUCTURE
 * ========================================================================= */

/**
 * @brief MQTT credentials loaded from NVS after successful provisioning.
 */
typedef struct {
    char username[TCU_CRED_MAX_LEN];
    char password[TCU_CRED_MAX_LEN];
    char client_id[TCU_CRED_MAX_LEN];
    char device_name[TCU_CRED_MAX_LEN];
} tcu_mqtt_creds_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Check whether provisioning credentials are stored in NVS.
 *
 * Reads the "prov_done" flag.  If NVS is unreadable this returns false,
 * which forces re-provisioning on next boot.
 *
 * @return true if credentials are available and the prov_done flag is set.
 */
bool tcu_nvs_is_provisioned(void);

/**
 * @brief Load MQTT credentials from NVS.
 *
 * @param[out] creds  Pointer to credential structure to populate.
 * @return ESP_OK on success, ESP_FAIL if any key is missing or NVS is
 *         corrupt.  On failure, creds contents are undefined.
 */
esp_err_t tcu_nvs_load_creds(tcu_mqtt_creds_t *creds);

/**
 * @brief Save MQTT credentials to NVS and mark provisioning complete.
 *
 * Writes all credential fields atomically (within the limits of NVS
 * transactions on ESP32-S3) and then sets prov_done=1.
 *
 * @param[in] creds  Credentials to persist.
 * @return ESP_OK on success.
 */
esp_err_t tcu_nvs_save_creds(const tcu_mqtt_creds_t *creds);

/**
 * @brief Erase all provisioning credentials from NVS.
 *
 * Sets prov_done=0 and clears all credential keys.  Used by:
 *   - Factory reset / trike swap workflow
 *   - NVS corruption fallback
 *   - Device deleted on ThingsBoard
 *
 * @return ESP_OK on success.
 */
esp_err_t tcu_nvs_erase_creds(void);

/**
 * @brief Attempt NVS namespace repair after corruption is detected.
 *
 * Erases only the tcu_creds namespace (not the entire NVS partition).
 * If even the erase fails, performs a full NVS partition erase.
 *
 * @return ESP_OK if the namespace was successfully cleared and is ready
 *         for fresh provisioning.
 */
esp_err_t tcu_nvs_repair(void);

#ifdef __cplusplus
}
#endif

#endif /* TCU_NVS_CREDS_H_ */