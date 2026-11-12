idf_component_register(SRCS "power_trike_ctrl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio esp_driver_uart freertos nvs_flash esp_system board_config rgb_led Quectel_mqtt bms_monitor_task
                    )

