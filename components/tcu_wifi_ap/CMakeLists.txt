idf_component_register(
    SRCS "tcu_wifi_ap.c" "tcu_dns_server.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event esp_http_server nvs_flash lwip freertos esp_timer
)
