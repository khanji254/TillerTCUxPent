/**
 * @file tcu_wifi_ap.c
 * @brief ESP32-S3 Wi-Fi SoftAP, Web Server, and Telemetry Portal Implementation
 */

#include "tcu_wifi_ap.h"
#include "tcu_dns_server.h"
#include "tcu_web_page.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TAG "TCU_WIFI_AP"

/* =========================================================================
 * STATIC VARIABLES & DATA BUFFERS
 * ========================================================================= */

static SemaphoreHandle_t s_data_mutex = NULL;
static tcu_telemetry_record_t s_latest_record;
static bool s_has_record = false;

/* Circular log buffer in RAM */
static tcu_telemetry_record_t s_log_buffer[TCU_TELEMETRY_LOG_MAX_RECORDS];
static size_t s_log_head = 0;
static size_t s_log_count = 0;

static uint8_t s_client_count = 0;
static httpd_handle_t s_http_server = NULL;
static esp_netif_t *s_netif_ap = NULL;
static char s_ap_ssid[33] = {0};

/* =========================================================================
 * WI-FI EVENT HANDLER
 * ========================================================================= */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            s_client_count++;
            ESP_LOGI(TAG, "Client connected: " MACSTR ", AID=%d, Total clients: %d",
                     MAC2STR(event->mac), event->aid, s_client_count);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            if (s_client_count > 0) s_client_count--;
            ESP_LOGI(TAG, "Client disconnected: " MACSTR ", AID=%d, Total clients: %d",
                     MAC2STR(event->mac), event->aid, s_client_count);
        }
    }
}

/* =========================================================================
 * HTTP REQUEST HANDLERS
 * ========================================================================= */

/**
 * @brief GET / - Serve the main web dashboard
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, TCU_INDEX_HTML, sizeof(TCU_INDEX_HTML) - 1);
}

/**
 * @brief Captive portal redirect handler for OS detection endpoints
 */
static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

/**
 * @brief GET /api/live - Returns current telemetry snapshot in JSON format
 */
static esp_err_t api_live_get_handler(httpd_req_t *req)
{
    char buf[1280];
    tcu_telemetry_record_t rec;
    bool valid = false;

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_has_record) {
            rec = s_latest_record;
            valid = true;
        }
        xSemaphoreGive(s_data_mutex);
    }

    if (!valid) {
        memset(&rec, 0, sizeof(rec));
        snprintf(rec.device_serial, sizeof(rec.device_serial), "TCU");
        snprintf(rec.hub_name, sizeof(rec.hub_name), "TCU_DEV");
    }

    uint32_t free_heap = esp_get_free_heap_size();

    int len = snprintf(buf, sizeof(buf),
        "{"
        "\"device_serial\":\"%s\","
        "\"hub_name\":\"%s\","
        "\"fw_version\":\"%s\","
        "\"uptime_s\":%" PRIu32 ","
        "\"free_heap\":%" PRIu32 ","
        "\"clients_connected\":%d,"
        "\"log_records_count\":%zu,"
        "\"log_capacity\":%d,"
        "\"power\":{"
            "\"voltage_v\":%.2f,"
            "\"current_a\":%.3f,"
            "\"current_max_a\":%.3f,"
            "\"power_w\":%.1f,"
            "\"temperature_c\":%.1f,"
            "\"soc_pct\":%u,"
            "\"trike_power_on\":%s"
        "},"
        "\"motion\":{"
            "\"is_moving\":%s,"
            "\"impact_alert\":%s,"
            "\"accel_x_mg\":%.1f,"
            "\"accel_y_mg\":%.1f,"
            "\"accel_z_mg\":%.1f,"
            "\"accel_mag_mg\":%.1f"
        "},"
        "\"gps\":{"
            "\"valid\":%s,"
            "\"lat\":%.6f,"
            "\"lon\":%.6f,"
            "\"altitude_m\":%.1f,"
            "\"speed_kmh\":%.1f,"
            "\"satellites\":%u,"
            "\"utc_time\":\"%s\","
            "\"utc_date\":\"%s\""
        "},"
        "\"cellular\":{"
            "\"rssi\":%d,"
            "\"dbm\":%d,"
            "\"mqtt_connected\":%s"
        "}"
        "}",
        rec.device_serial,
        rec.hub_name,
        rec.fw_version[0] ? rec.fw_version : "1.0.0 (Wan)",
        rec.uptime_s,
        free_heap,
        s_client_count,
        s_log_count,
        TCU_TELEMETRY_LOG_MAX_RECORDS,
        (double)rec.pack_voltage_v,
        (double)rec.shunt_current_a,
        (double)rec.shunt_max_a,
        (double)rec.power_w,
        (double)rec.temp_c,
        rec.avg_soc_pct,
        rec.trike_pwr_on ? "true" : "false",
        rec.motion_detected ? "true" : "false",
        rec.impact_detected ? "true" : "false",
        (double)rec.accel_x_mg,
        (double)rec.accel_y_mg,
        (double)rec.accel_z_mg,
        (double)rec.accel_mag_mg,
        rec.gps_valid ? "true" : "false",
        rec.latitude,
        rec.longitude,
        (double)rec.altitude_m,
        (double)rec.speed_kmh,
        rec.satellites,
        rec.utc_time,
        rec.utc_date,
        rec.gsm_rssi,
        rec.gsm_dbm,
        rec.mqtt_connected ? "true" : "false"
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, buf, len);
}

/**
 * @brief GET /api/snapshot/download - Download instant JSON snapshot
 */
static esp_err_t api_snapshot_download_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"tcu_telemetry_snapshot.json\"");
    return api_live_get_handler(req);
}

/**
 * @brief GET /api/log/download - Streams full rolling history as a CSV file
 */
static esp_err_t api_log_download_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"tcu_telemetry_log.csv\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    /* Send CSV Header */
    const char *header =
        "timestamp_ms,uptime_s,voltage_v,current_a,current_max_a,power_w,temp_c,soc_pct,"
        "trike_on,motion,impact,lat,lon,altitude_m,speed_kmh,sats,gps_valid,"
        "accel_x_mg,accel_y_mg,accel_z_mg,accel_mag_mg,gsm_rssi,gsm_dbm\r\n";
    httpd_resp_send_chunk(req, header, strlen(header));

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        size_t count = s_log_count;
        size_t start_idx = (count < TCU_TELEMETRY_LOG_MAX_RECORDS) ? 0 : s_log_head;
        char row[320];

        for (size_t i = 0; i < count; i++) {
            size_t idx = (start_idx + i) % TCU_TELEMETRY_LOG_MAX_RECORDS;
            const tcu_telemetry_record_t *r = &s_log_buffer[idx];

            int rlen = snprintf(row, sizeof(row),
                "%" PRIu32 ",%" PRIu32 ",%.2f,%.3f,%.3f,%.1f,%.1f,%u,"
                "%d,%d,%d,%.6f,%.6f,%.1f,%.1f,%u,%d,"
                "%.1f,%.1f,%.1f,%.1f,%d,%d\r\n",
                r->timestamp_ms,
                r->uptime_s,
                (double)r->pack_voltage_v,
                (double)r->shunt_current_a,
                (double)r->shunt_max_a,
                (double)r->power_w,
                (double)r->temp_c,
                r->avg_soc_pct,
                r->trike_pwr_on ? 1 : 0,
                r->motion_detected ? 1 : 0,
                r->impact_detected ? 1 : 0,
                r->latitude,
                r->longitude,
                (double)r->altitude_m,
                (double)r->speed_kmh,
                r->satellites,
                r->gps_valid ? 1 : 0,
                (double)r->accel_x_mg,
                (double)r->accel_y_mg,
                (double)r->accel_z_mg,
                (double)r->accel_mag_mg,
                r->gsm_rssi,
                r->gsm_dbm
            );

            if (rlen > 0) {
                esp_err_t err = httpd_resp_send_chunk(req, row, rlen);
                if (err != ESP_OK) {
                    xSemaphoreGive(s_data_mutex);
                    return err;
                }
            }
        }
        xSemaphoreGive(s_data_mutex);
    }

    /* End chunked transfer */
    return httpd_resp_send_chunk(req, NULL, 0);
}

/**
 * @brief POST /api/log/clear - Clears the rolling history buffer
 */
static esp_err_t api_log_clear_handler(httpd_req_t *req)
{
    tcu_wifi_ap_clear_log();
    const char *resp = "{\"status\":\"ok\",\"cleared\":true}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, strlen(resp));
}

/* =========================================================================
 * HTTP SERVER INITIALIZATION
 * ========================================================================= */

static esp_err_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.stack_size = 6144;
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&s_http_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register URI handlers */
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_root);

    httpd_uri_t uri_index = {
        .uri = "/index.html",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_index);

    httpd_uri_t uri_live = {
        .uri = "/api/live",
        .method = HTTP_GET,
        .handler = api_live_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_live);

    httpd_uri_t uri_log = {
        .uri = "/api/log/download",
        .method = HTTP_GET,
        .handler = api_log_download_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_log);

    httpd_uri_t uri_snapshot = {
        .uri = "/api/snapshot/download",
        .method = HTTP_GET,
        .handler = api_snapshot_download_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_snapshot);

    httpd_uri_t uri_clear = {
        .uri = "/api/log/clear",
        .method = HTTP_POST,
        .handler = api_log_clear_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_clear);

    /* Captive Portal URLs */
    httpd_uri_t uri_gen204 = {
        .uri = "/generate_204",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_gen204);

    httpd_uri_t uri_apple = {
        .uri = "/hotspot-detect.html",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_apple);

    httpd_uri_t uri_msft = {
        .uri = "/connecttest.txt",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_msft);

    ESP_LOGI(TAG, "Web server started on port %d with live dashboard", config.server_port);
    return ESP_OK;
}

/* =========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

esp_err_t tcu_wifi_ap_init(const char *ssid_override, const char *password)
{
    ESP_LOGI(TAG, "Initializing TCU Wi-Fi Access Point...");

    if (!s_data_mutex) {
        s_data_mutex = xSemaphoreCreateMutex();
        if (!s_data_mutex) {
            ESP_LOGE(TAG, "Failed to create telemetry data mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop if not already present */
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create default event loop: %s", esp_err_to_name(err));
        return err;
    }

    s_netif_ap = esp_netif_create_default_wifi_ap();

    /* Wi-Fi initialization */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL
    ));

    /* Build SSID */
    if (ssid_override && strlen(ssid_override) > 0) {
        strncpy(s_ap_ssid, ssid_override, sizeof(s_ap_ssid) - 1);
    } else {
        uint8_t mac[6];
        esp_base_mac_addr_get(mac);
        snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s%02X%02X%02X%02X%02X%02X",
                 TCU_WIFI_AP_DEFAULT_SSID_PREFIX,
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strncpy((char *)wifi_config.ap.ssid, s_ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(s_ap_ssid);
    wifi_config.ap.channel = TCU_WIFI_AP_DEFAULT_CHANNEL;
    wifi_config.ap.max_connection = TCU_WIFI_AP_MAX_CONNECTIONS;
    wifi_config.ap.beacon_interval = 100;

    /* Configure static IP and restart DHCP server */
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr = esp_ip4addr_aton(TCU_WIFI_AP_IP);
    ip_info.gw.addr = esp_ip4addr_aton(TCU_WIFI_AP_IP);
    ip_info.netmask.addr = esp_ip4addr_aton("255.255.255.0");
    esp_netif_dhcps_stop(s_netif_ap);
    esp_netif_set_ip_info(s_netif_ap, &ip_info);
    esp_netif_dhcps_start(s_netif_ap);

    const char *pwd = password ? password : TCU_WIFI_AP_DEFAULT_PASSWORD;
    if (pwd && strlen(pwd) >= 8) {
        strncpy((char *)wifi_config.ap.password, pwd, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        wifi_config.ap.password[0] = 0;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi SoftAP started successfully!");
    ESP_LOGI(TAG, "  SSID: %s", s_ap_ssid);
    ESP_LOGI(TAG, "  Auth: %s", (wifi_config.ap.authmode == WIFI_AUTH_OPEN) ? "OPEN" : "WPA2-PSK");
    if (wifi_config.ap.authmode != WIFI_AUTH_OPEN) {
        ESP_LOGI(TAG, "  Password: %s", (char *)wifi_config.ap.password);
    }
    ESP_LOGI(TAG, "  IP Address: %s", TCU_WIFI_AP_IP);

    /* Start Captive DNS server on UDP port 53 */
    tcu_dns_server_start();

    /* Start HTTP Web Server */
    start_web_server();

    return ESP_OK;
}

void tcu_wifi_ap_update_telemetry(const tcu_telemetry_record_t *record)
{
    if (!record || !s_data_mutex) return;

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        /* Update latest snapshot */
        s_latest_record = *record;
        s_has_record = true;

        /* Append to circular buffer */
        s_log_buffer[s_log_head] = *record;
        s_log_head = (s_log_head + 1) % TCU_TELEMETRY_LOG_MAX_RECORDS;
        if (s_log_count < TCU_TELEMETRY_LOG_MAX_RECORDS) {
            s_log_count++;
        }

        xSemaphoreGive(s_data_mutex);
    }
}

bool tcu_wifi_ap_get_latest(tcu_telemetry_record_t *out)
{
    if (!out || !s_data_mutex) return false;
    bool ok = false;
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_has_record) {
            *out = s_latest_record;
            ok = true;
        }
        xSemaphoreGive(s_data_mutex);
    }
    return ok;
}

void tcu_wifi_ap_clear_log(void)
{
    if (!s_data_mutex) return;
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        s_log_head = 0;
        s_log_count = 0;
        xSemaphoreGive(s_data_mutex);
        ESP_LOGI(TAG, "Telemetry log buffer cleared");
    }
}

size_t tcu_wifi_ap_get_log_count(void)
{
    size_t count = 0;
    if (s_data_mutex && xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        count = s_log_count;
        xSemaphoreGive(s_data_mutex);
    }
    return count;
}

uint8_t tcu_wifi_ap_get_client_count(void)
{
    return s_client_count;
}
