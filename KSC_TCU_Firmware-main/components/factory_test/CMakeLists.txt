idf_component_register(SRCS "factory_test.c"
                    INCLUDE_DIRS "include"
					    REQUIRES driver freertos board_config lis3dhtr rgb_led
					             trike_sensors bms_rs485_mux Quectel_mqtt                                       
                    )

