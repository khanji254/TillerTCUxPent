/**
 * @file tcu_nvs_creds.c
 * @brief KSC TCU V2.1 — NVS Credential Storage Implementation
 *
 * @author  Collins N. Osebe
 * @date    2026-11-09
 */

#include "tcu_nvs_creds.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

#ifndef CONFIG_TB_PROVISION_KEY
#define CONFIG_TB_PROVISION_KEY ""
#endif

#ifndef CONFIG_TB_PROVISION_SECRET
#define CONFIG_TB_PROVISION_SECRET ""
#endif

#define TAG "NVS_CREDS"

/* =========================================================================
 * INTERNAL HELPERS
 * ========================================================================= */

/**
 * @brief Open the tcu_creds NVS namespace.
 *
 * @param[out] handle  NVS handle to populate.
 * @param[in]  mode    NVS_READONLY or NVS_READWRITE.
 * @return ESP_OK on success.
 */
static esp_err_t open_namespace(nvs_handle_t *handle, nvs_open_mode_t mode)
{
    esp_err_t err = nvs_open(TCU_NVS_NAMESPACE, mode, handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) failed: %s",
                 TCU_NVS_NAMESPACE, esp_err_to_name(err));
    }
    return err;
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

bool tcu_nvs_is_provisioned(void)
{
    nvs_handle_t h;
    if (open_namespace(&h, NVS_READONLY) != ESP_OK) {
        return false;
    }

    uint8_t done = 0;
    esp_err_t err = nvs_get_u8(h, TCU_NVS_KEY_PROV_DONE, &done);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "prov_done read failed (%s) — treating as unprovisioned",
                 esp_err_to_name(err));
        return false;
    }
    return (done == 1);
}

esp_err_t tcu_nvs_load_creds(tcu_mqtt_creds_t *creds)
{
    if (!creds) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = open_namespace(&h, NVS_READONLY);
    if (err != ESP_OK) return err;

    size_t len;
    esp_err_t rc = ESP_OK;

#define READ_STR(key, dst) \
    do { \
        len = sizeof(dst); \
        err = nvs_get_str(h, (key), (dst), &len); \
        if (err != ESP_OK) { \
            ESP_LOGE(TAG, "nvs_get_str(%s) failed: %s", \
                     (key), esp_err_to_name(err)); \
            rc = ESP_FAIL; \
        } \
    } while(0)

    READ_STR(TCU_NVS_KEY_USERNAME,    creds->username);
    READ_STR(TCU_NVS_KEY_PASSWORD,    creds->password);
    READ_STR(TCU_NVS_KEY_CLIENT_ID,   creds->client_id);
    READ_STR(TCU_NVS_KEY_DEVICE_NAME, creds->device_name);

#undef READ_STR

    nvs_close(h);

    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Credentials loaded — device=%s clientId=%s",
                 creds->device_name, creds->client_id);
    } else {
        ESP_LOGE(TAG, "Credential load failed — NVS may be corrupt");
    }
    return rc;
}

esp_err_t tcu_nvs_save_creds(const tcu_mqtt_creds_t *creds)
{
    if (!creds) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = open_namespace(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    esp_err_t rc = ESP_OK;

#define WRITE_STR(key, src) \
    do { \
        err = nvs_set_str(h, (key), (src)); \
        if (err != ESP_OK) { \
            ESP_LOGE(TAG, "nvs_set_str(%s) failed: %s", \
                     (key), esp_err_to_name(err)); \
            rc = ESP_FAIL; \
        } \
    } while(0)

    WRITE_STR(TCU_NVS_KEY_USERNAME,    creds->username);
    WRITE_STR(TCU_NVS_KEY_PASSWORD,    creds->password);
    WRITE_STR(TCU_NVS_KEY_CLIENT_ID,   creds->client_id);
    WRITE_STR(TCU_NVS_KEY_DEVICE_NAME, creds->device_name);

#undef WRITE_STR

    if (rc == ESP_OK) {
        err = nvs_set_u8(h, TCU_NVS_KEY_PROV_DONE, 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_set_u8(prov_done) failed: %s",
                     esp_err_to_name(err));
            rc = ESP_FAIL;
        }
    }

    if (rc == ESP_OK) {
        err = nvs_commit(h);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
            rc = ESP_FAIL;
        }
    }

    nvs_close(h);

    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Credentials saved — device=%s", creds->device_name);
    }
    return rc;
}

esp_err_t tcu_nvs_erase_creds(void)
{
    nvs_handle_t h;
    esp_err_t err = open_namespace(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    /* Erase only MQTT credentials, preserving device_name and prov_key */
    nvs_erase_key(h, TCU_NVS_KEY_PROV_DONE);
    nvs_erase_key(h, TCU_NVS_KEY_USERNAME);
    nvs_erase_key(h, TCU_NVS_KEY_PASSWORD);
    nvs_erase_key(h, TCU_NVS_KEY_CLIENT_ID);

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Provisioning credentials erased — device will re-provision on next boot (retaining device name)");
    }
    return err;
}

esp_err_t tcu_nvs_factory_reset(void)
{
    nvs_handle_t h;
    esp_err_t err = open_namespace(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    /* Erase all keys in namespace */
    err = nvs_erase_all(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_erase_all failed: %s", esp_err_to_name(err));
        nvs_close(h);
        return err;
    }

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Full factory reset: all keys in %s erased", TCU_NVS_NAMESPACE);
    }
    return err;
}

esp_err_t tcu_nvs_get_device_name(char *name, size_t max_len)
{
    if (!name || max_len == 0) return ESP_ERR_INVALID_ARG;
    name[0] = '\0';

    nvs_handle_t h;
    esp_err_t err = open_namespace(&h, NVS_READONLY);
    if (err != ESP_OK) return err;

    size_t len = max_len;
    err = nvs_get_str(h, TCU_NVS_KEY_DEVICE_NAME, name, &len);
    nvs_close(h);

    if (err == ESP_OK && strlen(name) > 0) {
        return ESP_OK;
    }
    return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
}

esp_err_t tcu_nvs_set_device_name(const char *name)
{
    if (!name || strlen(name) == 0) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = open_namespace(&h, NVS_READWRITE);
    if (err != ESP_OK) return err;

    err = nvs_set_str(h, TCU_NVS_KEY_DEVICE_NAME, name);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t tcu_nvs_repair(void)
{
    ESP_LOGW(TAG, "Attempting NVS credentials namespace repair...");

    /* Erase only our namespace to avoid destroying sensor calibration (trike_sens) or contactor state */
    esp_err_t err = tcu_nvs_erase_creds();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS credentials namespace repaired successfully");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "NVS credentials repair failed: %s (preserving non-credential namespaces)",
             esp_err_to_name(err));
    return err;
}

esp_err_t tcu_nvs_get_provisioning_credentials(char *key, size_t key_len,
                                               char *secret, size_t secret_len)
{
    if (!key || !secret || key_len == 0 || secret_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    key[0] = '\0';
    secret[0] = '\0';

    /* 1. Try NVS first */
    nvs_handle_t h;
    if (open_namespace(&h, NVS_READONLY) == ESP_OK) {
        size_t klen = key_len;
        size_t slen = secret_len;
        esp_err_t err_k = nvs_get_str(h, TCU_NVS_KEY_PROV_KEY, key, &klen);
        esp_err_t err_s = nvs_get_str(h, TCU_NVS_KEY_PROV_SECRET, secret, &slen);
        nvs_close(h);

        if (err_k == ESP_OK && err_s == ESP_OK && strlen(key) > 0 && strlen(secret) > 0) {
            ESP_LOGI(TAG, "Provisioning credentials retrieved from NVS");
            return ESP_OK;
        }
    }

    /* 2. Fall back to Kconfig */
    if (strlen(CONFIG_TB_PROVISION_KEY) > 0 && strlen(CONFIG_TB_PROVISION_SECRET) > 0) {
        strncpy(key, CONFIG_TB_PROVISION_KEY, key_len - 1);
        strncpy(secret, CONFIG_TB_PROVISION_SECRET, secret_len - 1);
        key[key_len - 1] = '\0';
        secret[secret_len - 1] = '\0';
        ESP_LOGI(TAG, "Provisioning credentials retrieved from Kconfig / build config");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No provisioning credentials found in NVS or Kconfig");
    return ESP_ERR_NOT_FOUND;
}
