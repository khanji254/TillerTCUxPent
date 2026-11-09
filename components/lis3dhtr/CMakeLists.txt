idf_component_register(SRCS "lis3dhtr.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    REQUIRES board_config 
                    REQUIRES freertos                     
                    )

