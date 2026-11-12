idf_component_register(SRCS "Quectel_gps.c"
                    INCLUDE_DIRS "include"
                    REQUIRES Quectel_mqtt driver esp_driver_uart esp_driver_gpio esp_timer freertos board_config
                    )

