idf_component_register(SRCS "lsm6dsox.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    REQUIRES board_config 
                    REQUIRES freertos                     
                    )
