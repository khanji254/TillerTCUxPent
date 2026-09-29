/**
 * @file Quectel_mqtt.h
 * @brief Quectel EG915N Modem MQTT Transport Layer for KSC TCU V1.1
 *
 * Provides AT-command driven MQTT connectivity over the Quectel EG915N
 * cellular modem.  All pin assignments are sourced from board_config.h.
 *
 * Key changes from original V1.1:
 *   - Full MQTT provisioning workflow integrated.
 *   - PROVISION_KEY / PROVISION_SECRET updated to Songa_Trikes profile.
 *   - MQTT_USERNAME / MQTT_PASSWORD / MQTT_CLIENT_ID are now runtime
 *     variables loaded from NVS (tcu_nvs_creds), not compile-time defines.
 *   - mqtt_subclient() now checks NVS and runs provisioning if needed.
 *   - tcu_nvs_creds.h included for credential management.
 *   - publish_rpc_responses() is now used in the RPC handler for
 *     two-way command acknowledgement.
 *   - open_mqtts() now accepts explicit credential parameters so both
 *     normal and provisioned paths share one connection function.
 *   - New: tcu_force_reprovision() for trike-swap / device-deleted recovery.
 *
 * @author  Mary Mbugua
 * @date    2026-04-10
 */
 
#ifndef QUECTEL_MQTT_H_
#define QUECTEL_MQTT_H_
 
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include <math.h>
 
#include "board_config.h"
#include "bms_monitor_types.h"
#include "bms_monitor_task.h"
#include "Quectel_gps.h"
#include "tcu_nvs_creds.h"
 
#ifdef __cplusplus
extern "C" {
#endif
 
/* =========================================================================
 * MODEM UART ALIASES (derived from board_config.h)
 * ========================================================================= */
 
#define MODEM_UART_NUM       BOARD_MODEM_UART_NUM
#define TX_PIN               BOARD_MODEM_TX_PIN
#define RX_PIN               BOARD_MODEM_RX_PIN
#define RESET_PIN            BOARD_MODEM_RST_PIN
#define PWR_PIN              BOARD_MODEM_PWR_PIN
#define QUECTEL_BAUDRATE     BOARD_MODEM_BAUD_RATE
#define BUF_SIZE             BOARD_MODEM_BUF_SIZE
 
#define TAG_GSM              "MODEM"
 
/* =========================================================================
 * THINGSBOARD CONNECTION
 * ========================================================================= */
 
#define MQTT_BROKER          "thingsboard.iot.songa.mobi"
#define MQTT_PORT             1883
 
/* =========================================================================
 * PROVISIONING PROFILE CREDENTIALS
 * =========================================================================
 * These are the PROFILE-LEVEL credentials from the Songa_Trikes device
 * profile.  They are ONLY used during the one-time provisioning handshake.
 * After provisioning, the device uses per-device credentials stored in NVS.
 * Never log or expose PROVISION_SECRET in production builds.
 * ========================================================================= */
 
#define PROVISION_KEY        "c1mtrworzsysp7vrth4z"
#define PROVISION_SECRET     "30irfv1tah0d5on6b1tx"
#define CREDENTIALS_TYPE     "MQTT_BASIC"
 
/* Provisioning MQTT connection uses these fixed credentials (TB spec) */
#define MQTT_PROV_USERNAME   "provision"
#define MQTT_PROV_PASSWORD   ""
 
/* =========================================================================
 * RUNTIME DEVICE IDENTITY
 * =========================================================================
 * deviceSerial is set once at boot by getChipIdString() in main.c.
 * HUB_NAME is "TCU_<MAC>" — used as the ThingsBoard device name.
 *
 * mqtt_username, mqtt_password, mqtt_client_id are populated either:
 *   a) From NVS after a previous successful provisioning, OR
 *   b) Derived from deviceSerial during the provisioning flow before
 *      being saved to NVS.
 * ========================================================================= */
 
extern char deviceSerial[13];
extern char HUB_NAME[32];
 
/* Runtime credential buffers — populated by tcu_load_or_provision_creds() */
extern char mqtt_username[TCU_CRED_MAX_LEN];
extern char mqtt_password[TCU_CRED_MAX_LEN];
extern char mqtt_client_id[TCU_CRED_MAX_LEN];
 
/* =========================================================================
 * AT COMMAND RETRY DEFAULTS
 * ========================================================================= */
 
static const int     RETRIES      = 6;
static const int     CMD_DELAY_MS = 200;
 
/* =========================================================================
 * CELLULAR APN
 * ========================================================================= */
 
extern char APN[32];
 
/* =========================================================================
 * PROVISIONING STATE
 * =========================================================================
 * provision == 0  → device is in provisioning mode (parsing /provision/response)
 * provision == 1  → device is in normal telemetry mode
 * ========================================================================= */
 
extern int32_t provision;
extern bool    prov_resub;
 
/* =========================================================================
 * UART MODEM MUTEX
 * ========================================================================= */
 
extern SemaphoreHandle_t uart_modem_mutex;
void uart_modem_mutex_init(void);
 
/* =========================================================================
 * URC NOTIFICATION QUEUE
 * ========================================================================= */
 
typedef struct {
    uint8_t  client_idx;
    uint8_t  recv_id;
    uint32_t timestamp_ms;
} urc_notification_t;
 
extern QueueHandle_t urc_notification_queue;
void urc_queue_init(void);
 
/* =========================================================================
 * RPC RESPONSE
 * ========================================================================= */
 
typedef struct {
    int   request_id;
    char *response_payload;   /**< Heap-allocated — freed by publish_rpc_responses() */
} RpcResponse;

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/** @brief Initialise UART driver for modem communication. */
void uart_init(void);

/** @brief Assert modem reset pin sequence. */
void gsm_reset(void);

/** @brief Power on the Quectel module via PWR_KEY toggling. */
void gsm_poweron(void);

/** @brief Hardware power off (GPIO toggle). */
void hardware_poweroff(void);

/** @brief Power off modem via AT+QPOWD command. */
void at_poweroff(void);

/**
 * @brief Send an AT command and wait for the expected response.
 *
 * Thread-safe: acquires uart_modem_mutex before sending.
 *
 * The caller may optionally supply a response buffer.  If response_buf
 * is non-NULL the raw modem response is copied into it.  This eliminates
 * the shared global urcbuffer / gpsbuffer pattern from V1.0 and prevents
 * race conditions when multiple tasks send AT commands.
 *
 * @param[in]  command            AT command string (without \\r\\n).
 * @param[in]  expected_response  Substring to search for in modem response.
 * @param[in]  retries            Number of send attempts before giving up.
 * @param[in]  timeout_ms         Per-attempt read timeout in milliseconds.
 * @param[out] response_buf       Optional buffer to receive the raw response.
 *                                Pass NULL if the response content is not needed.
 * @param[in]  response_buf_size  Size of response_buf in bytes (0 if NULL).
 * @return true if expected_response was found, false otherwise.
 */
bool send_at_command(const char *command,
                     const char *expected_response,
                     int         retries,
                     uint32_t    timeout_ms,
                     char       *response_buf,
                     size_t      response_buf_size);

/** @brief Power on modem and verify AT communication. */
bool poweron_modem(void);

/** @brief Attach to cellular network and activate PDP context. */
bool activate_pdp(void);

 
/**
 * @brief Open MQTT connection using explicit credentials.
 *
 * Replaces the old open_mqtts() that used compile-time macros.
 * Both the normal and provisioning paths call this with their
 * respective credentials, avoiding duplicated QMTOPEN logic.
 *
 * @param[in] client_id  MQTT client identifier string.
 * @param[in] username   MQTT username string.
 * @param[in] password   MQTT password string (may be empty "").
 * @return true on successful CONNACK from broker.
 */
bool open_mqtts_with_creds(const char *client_id,
                            const char *username,
                            const char *password);

/** @brief Disconnect MQTT session. */
bool mqtts_disconnect(void);

/** @brief Deactivate PDP context. */
bool deactivate_pdp(void);

/** @brief Power down modem gracefully. */
void powerdown_modem(void);

/** @brief Full modem init sequence: power on + PDP + MQTT. */
bool mqtts_init(void);

/** @brief Provisioning modem init sequence. */
bool provision_mqtts_init(void);

/** @brief Disconnect MQTT and power down modem. */
void mqtts_deinit(void);

/** @brief Configure MQTT receive mode settings. */
bool configure_mqtt_settings(void);

/** @brief Decode hex-encoded ASCII string. */
bool hex_to_ascii(const char *hex, char *output);

/** @brief Handle MQTT error codes and trigger reconnect as appropriate. */
void handle_mqtt_error(int err_code);

/** @brief Verify MQTT connection is active; reconnect if not. */
void check_mqtt_link_status(void);

/**
 * @brief Read and process a buffered MQTT message from slot recv_id.
 *
 * @param[in] recv_id  Receive slot index from URC notification.
 * @return true on success.
 */
bool read_buffered_messages(int recv_id);

/**
 * @brief Poll UART for unsolicited result codes (URCs).
 *
 * @return true if a URC was found and enqueued.
 */
bool check_mqtt_urc(void);

/** @brief Send a keep-alive dummy publish. */
bool mqtt_dummypub(void);

/**
 * @brief Initialise MQTT connection and subscribe to RPC topic.
 *
 * @return true on success.
 */
bool mqtt_subclient(void);

/** @brief Initialise provisioning MQTT session. */
bool mqtt_provision_subclient(void);

/** @brief Re-subscribe to the appropriate MQTT topic after reconnect. */
bool mqtt_resubscribe(void);

/** @brief Reconnect MQTT after a connection error. */
void mqtts_error_reconnect(void);

/** @brief Handle an RPC state read request (stub for future extension). */
void handle_rpc_state_request(int slave_id, int request_id,
                               const char *param_name,
                               RpcResponse *responses, int *response_count);

/**
 * @brief Publish an array of RPC response payloads.
 *
 * @param[in] responses  Array of RpcResponse structures.
 * @param[in] count      Number of responses.
 * @return true if all responses were published successfully.
 */
bool publish_rpc_responses(RpcResponse *responses, int count);

/** @brief Build device provisioning JSON payload. */
char *create_provision_json_payload(void);

/**
 * @brief Publish battery telemetry payloads plus metadata to ThingsBoard.
 *
 * @return true on success.
 */
bool mqtt_pubclient_battery(void);

/** @brief Publish the provisioning request payload. */
bool mqtt_provision_pubclient(void);

/**
 * @brief Initialise GPS streaming (called from mqtt_publish_task).
 *
 * @return true on success.
 */
bool gps_mqtt_init(void);

/**
 * @brief Publish metadata-only status payload to ThingsBoard.
 *
 * @return true on success.
 */
bool mqtt_pubclient_status();

/**
 * @brief Return the most recently computed average battery SOC (0–100%).
 *
 * @return Average SOC percentage, or 0 if no valid readings.
 */
uint8_t get_average_battery_soc(void);

char *create_battery_json_payload(bms_queued_data_t *battery_data,
                                  bms_queued_data_t *all_battery_data,
                                  int batteries_read);

char *create_metadata_payload(float shunt_snapshot_a, float            *shunt_max_out);

 
/**
 * @brief Top-level credential resolution: load from NVS or run provisioning.
 *
 * Called once from mqtt_subclient() before attempting normal MQTT connection.
 * If NVS credentials are present and valid, they are loaded into the runtime
 * buffers (mqtt_username / mqtt_password / mqtt_client_id).
 * If not, the full provisioning flow executes:
 *   1. Connect with "provision" username.
 *   2. Publish to /provision/request.
 *   3. Wait for /provision/response.
 *   4. On SUCCESS: save credentials to NVS and restart.
 *   5. On failure: retry up to TCU_PROV_MAX_RETRIES times, then restart.
 *
 * @return true if credentials are ready for normal connection.
 *         Returns false only on hard failure (modem won't power on, etc.).
 */
bool tcu_load_or_provision_creds(void);
 
/**
 * @brief Force re-provisioning on next boot.
 *
 * Erases NVS credentials.  Use cases:
 *   - Moving TCU to a different trike (trike swap)
 *   - Device deleted from ThingsBoard
 *   - Factory reset
 *
 * Does NOT restart the device — caller must call esp_restart() if
 * immediate re-provisioning is required.
 *
 * @return ESP_OK on success.
 */
esp_err_t tcu_force_reprovision(void);

bool get_gsm_signal_quality(int *rssi_out, int *dbm_out);

 
/* ---- Number of provisioning retries before hard restart ---- */
#define TCU_PROV_MAX_RETRIES   3
 
/* ---- Provisioning timeout (ms) waiting for /provision/response ---- */
#define TCU_PROV_RESPONSE_TIMEOUT_MS   15000

#ifdef __cplusplus
}
#endif

#endif /* QUECTEL_MQTT_H_ */