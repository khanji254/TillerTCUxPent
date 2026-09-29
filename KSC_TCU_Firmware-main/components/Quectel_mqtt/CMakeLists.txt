idf_component_register(SRCS "Quectel_mqtt.c"
                    INCLUDE_DIRS "include"
				    REQUIRES driver freertos esp_timer nvs_flash json board_config
				             bms_monitor_types bms_monitor_task Quectel_gps rgb_led
				             power_trike_ctrl tcu_nvs_creds                   
                    )

