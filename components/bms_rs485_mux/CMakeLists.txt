idf_component_register(SRCS "bms_rs485_mux.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio freertos board_config rgb_led bms_monitor_types
                    )

