idf_component_register(SRCS "lis3dhtr.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_gpio esp_driver_i2c board_config freertos                     
                    )

