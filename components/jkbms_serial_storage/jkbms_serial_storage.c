/**
 * @file bms_serial_storage.c
 * @brief NVS storage for battery serial numbers with wear protection
 *
 * CHANGES: Added bms_serial_storage_clear_all() for triple-button NVS reset.
 */

#include "jkbms_serial_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#define TAG "BMS_SERIAL_STORAGE"
#define NVS_NAMESPACE           "bms_serials"
#define NVS_KEY_PREFIX          "bat_sn_"
#define NVS_KEY_COUNTER_PREFIX  "bat_cnt_"
#define NVS_WRITE_THRESHOLD     100

static char     cached_serials[BMS_BATTERY_COUNT][16] = {0};
static uint32_t write_counters[BMS_BATTERY_COUNT]     = {0};
static bool     initialized                           = false;

// ============================================================================
// PUBLIC API
// ============================================================================

esp_err_t bms_serial_storage_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full or version mismatch, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = bms_serial_storage_load(cached_serials);
    if (ret != ESP_OK) {
        for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
            snprintf(cached_serials[i], 16, "BMS%d_UNKNOWN", i + 1);
        }
    }

    initialized = true;
    ESP_LOGI(TAG, "Serial storage initialized");
    return ESP_OK;
}

esp_err_t bms_serial_storage_load(char serials[BMS_BATTERY_COUNT][16])
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
        char key[16];
        snprintf(key, sizeof(key), "%s%d", NVS_KEY_PREFIX, i);

        size_t required_size = 16;
        ret = nvs_get_str(nvs_handle, key, serials[i], &required_size);

        if (ret != ESP_OK) {
            snprintf(serials[i], 16, "BMS%d_UNKNOWN", i + 1);
        } else {
            ESP_LOGI(TAG, "Loaded serial for battery %d: %s", i, serials[i]);
        }

        char counter_key[16];
        snprintf(counter_key, sizeof(counter_key), "%s%d", NVS_KEY_COUNTER_PREFIX, i);
        nvs_get_u32(nvs_handle, counter_key, &write_counters[i]);
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t bms_serial_storage_save(bms_battery_id_t battery_id, const char *serial_no, bool force)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Storage not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (battery_id >= BMS_BATTERY_COUNT || serial_no == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cached_serials[battery_id], serial_no) == 0 && !force) {
        return ESP_OK;
    }

    strncpy(cached_serials[battery_id], serial_no, 15);
    cached_serials[battery_id][15] = '\0';
    write_counters[battery_id]++;

    if (!force && (write_counters[battery_id] % NVS_WRITE_THRESHOLD != 0)) {
        return ESP_OK;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for write: %s", esp_err_to_name(ret));
        return ret;
    }

    char key[16];
    snprintf(key, sizeof(key), "%s%d", NVS_KEY_PREFIX, battery_id);
    ret = nvs_set_str(nvs_handle, key, serial_no);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write serial: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    char counter_key[16];
    snprintf(counter_key, sizeof(counter_key), "%s%d", NVS_KEY_COUNTER_PREFIX, battery_id);
    nvs_set_u32(nvs_handle, counter_key, write_counters[battery_id]);

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved serial for battery %d: %s (writes: %" PRIu32 ")",
                 battery_id, serial_no, write_counters[battery_id]);
    }

    return ret;
}

esp_err_t bms_serial_storage_get(bms_battery_id_t battery_id, char *serial_out)
{
    if (!initialized || battery_id >= BMS_BATTERY_COUNT || serial_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(serial_out, cached_serials[battery_id], 15);
    serial_out[15] = '\0';
    return ESP_OK;
}

/**
 * @brief Erase ALL stored serial numbers from NVS.
 * Also resets in-memory cache to UNKNOWN strings.
 */
esp_err_t bms_serial_storage_clear_all(void)
{
    ESP_LOGW(TAG, "Clearing all battery serials from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (ret != ESP_OK) {
        // Namespace may not exist yet - that's fine
        ESP_LOGW(TAG, "NVS namespace not found, nothing to clear: %s", esp_err_to_name(ret));
        return ESP_OK;
    }

    // Erase entire namespace
    ret = nvs_erase_all(nvs_handle);
    if (ret == ESP_OK) {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "All battery serials erased from NVS");
    } else {
        ESP_LOGE(TAG, "Failed to erase battery serials: %s", esp_err_to_name(ret));
    }

    nvs_close(nvs_handle);

    // Reset in-memory cache
    for (int i = 0; i < BMS_BATTERY_COUNT; i++) {
        snprintf(cached_serials[i], 16, "BMS%d_UNKNOWN", i + 1);
        write_counters[i] = 0;
    }

    return ret;
}