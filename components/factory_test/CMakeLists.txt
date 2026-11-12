idf_component_register(SRCS "factory_test.c"
                    INCLUDE_DIRS "include"
					    REQUIRES driver esp_driver_gpio esp_driver_uart esp_driver_i2c freertos board_config lsm6dsox rgb_led
					             trike_sensors bms_rs485_mux Quectel_mqtt                                       
                    )

