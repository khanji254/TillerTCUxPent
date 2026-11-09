idf_component_register(SRCS "factory_test.c"
                    INCLUDE_DIRS "include"
					    REQUIRES driver freertos board_config lsm6dsox rgb_led
					             trike_sensors bms_rs485_mux Quectel_mqtt                                       
                    )

