idf_component_register(SRCS "rgb_led.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    REQUIRES board_config 
                    REQUIRES freertos                 
                    )

