/**
 * @file tcu_dns_server.h
 * @brief Captive DNS server for KSC TCU Wi-Fi Access Point
 *
 * Resolves all DNS queries to 192.168.4.1 so mobile devices
 * can trigger captive portal detection and users can browse
 * to any hostname (e.g. tcu.local or setup.com) to reach the dashboard.
 */

#ifndef TCU_DNS_SERVER_H_
#define TCU_DNS_SERVER_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the captive DNS server task.
 * @return ESP_OK on success.
 */
esp_err_t tcu_dns_server_start(void);

/**
 * @brief Stop the captive DNS server task.
 */
void tcu_dns_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TCU_DNS_SERVER_H_ */
