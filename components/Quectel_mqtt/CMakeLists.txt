idf_component_register(SRCS "Quectel_mqtt.c"
                    INCLUDE_DIRS "include"
				    REQUIRES driver esp_driver_gpio esp_driver_uart freertos esp_timer nvs_flash json board_config
				             bms_monitor_types bms_monitor_task Quectel_gps rgb_led
				             power_trike_ctrl tcu_nvs_creds                   
                    )

