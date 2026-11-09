/**
 * @file tcu_dns_server.c
 * @brief Captive DNS server for KSC TCU Wi-Fi Access Point
 */

#include "tcu_dns_server.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "TCU_DNS"
#define DNS_PORT 53
#define DNS_BUFFER_SIZE 256
#define DNS_TASK_STACK_SIZE 4096

static TaskHandle_t dns_task_handle = NULL;
static int dns_sock = -1;
static volatile bool dns_running = false;

/* Static buffers to avoid placing large arrays on the task stack */
static uint8_t s_rx_buffer[DNS_BUFFER_SIZE];
static uint8_t s_tx_buffer[DNS_BUFFER_SIZE + 16];

static void dns_server_task(void *pvParameters)
{
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    dns_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (dns_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    /* Set socket receive timeout (1 second) so it unblocks periodically */
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(dns_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(DNS_PORT);

    if (bind(dns_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind DNS socket to port %d: errno %d", DNS_PORT, errno);
        close(dns_sock);
        dns_sock = -1;
        dns_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Captive DNS server listening on UDP port %d", DNS_PORT);
    dns_running = true;

    while (dns_running) {
        client_len = sizeof(client_addr);
        int len = recvfrom(dns_sock, s_rx_buffer, sizeof(s_rx_buffer), 0,
                           (struct sockaddr *)&client_addr, &client_len);
        if (len < 12) {
            continue;
        }

        /* Check if it is a standard query: QR=0, Opcode=0 */
        uint8_t flags_hi = s_rx_buffer[2];
        if ((flags_hi & 0xFA) != 0) {
            continue; // Not a standard query
        }

        /* Prepare DNS response:
         * Copy header and question from query.
         */
        memcpy(s_tx_buffer, s_rx_buffer, len);

        /* Set Response Flags: QR=1 (response), AA=1, RA=0, RCODE=0 -> 0x8180 */
        s_tx_buffer[2] = 0x81;
        s_tx_buffer[3] = 0x80;

        /* Set Answer Count to 1 */
        s_tx_buffer[6] = 0x00;
        s_tx_buffer[7] = 0x01;

        /* Authority and Additional RR counts = 0 */
        s_tx_buffer[8] = 0x00;
        s_tx_buffer[9] = 0x00;
        s_tx_buffer[10] = 0x00;
        s_tx_buffer[11] = 0x00;

        /* Append Answer record:
         * Name pointer to offset 12: 0xC00C
         * Type: A (0x0001)
         * Class: IN (0x0001)
         * TTL: 60s (0x0000003C)
         * Data length: 4 (0x0004)
         * IP Address: 192.168.4.1
         */
        int offset = len;
        if (offset + 16 <= (int)sizeof(s_tx_buffer)) {
            s_tx_buffer[offset++] = 0xC0;
            s_tx_buffer[offset++] = 0x0C; // Pointer to question name at header offset 12
            s_tx_buffer[offset++] = 0x00;
            s_tx_buffer[offset++] = 0x01; // Type A
            s_tx_buffer[offset++] = 0x00;
            s_tx_buffer[offset++] = 0x01; // Class IN
            s_tx_buffer[offset++] = 0x00;
            s_tx_buffer[offset++] = 0x00;
            s_tx_buffer[offset++] = 0x00;
            s_tx_buffer[offset++] = 0x3C; // TTL 60
            s_tx_buffer[offset++] = 0x00;
            s_tx_buffer[offset++] = 0x04; // Data length 4
            s_tx_buffer[offset++] = 192;
            s_tx_buffer[offset++] = 168;
            s_tx_buffer[offset++] = 4;
            s_tx_buffer[offset++] = 1;

            sendto(dns_sock, s_tx_buffer, offset, 0,
                   (struct sockaddr *)&client_addr, client_len);
        }
    }

    if (dns_sock >= 0) {
        close(dns_sock);
        dns_sock = -1;
    }
    dns_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t tcu_dns_server_start(void)
{
    if (dns_task_handle != NULL) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        dns_server_task,
        "tcu_dns",
        DNS_TASK_STACK_SIZE,
        NULL,
        3,
        &dns_task_handle,
        0
    );

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}

void tcu_dns_server_stop(void)
{
    dns_running = false;
    if (dns_sock >= 0) {
        close(dns_sock);
        dns_sock = -1;
    }
    dns_task_handle = NULL;
}
