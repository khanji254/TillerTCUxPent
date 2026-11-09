idf_component_register(SRCS "trike_sensors.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_adc nvs_flash board_config                   
                    )

