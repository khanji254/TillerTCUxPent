idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_adc esp_event esp_http_server esp_netif esp_wifi nvs_flash
)
