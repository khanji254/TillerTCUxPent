idf_component_register(SRCS "Quectel_gps.c"
                    INCLUDE_DIRS "include"
                    REQUIRES Quectel_mqtt
                    REQUIRES driver
                    REQUIRES esp_timer
                    REQUIRES freertos
                    REQUIRES board_config
                    )

