idf_component_register(SRCS "rgb_led.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio board_config freertos                 
                    )

