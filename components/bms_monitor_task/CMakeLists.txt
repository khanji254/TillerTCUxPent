idf_component_register(
    SRCS "bms_monitor_task.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos driver esp_driver_gpio esp_driver_uart esp_driver_i2c board_config bms_monitor_types bms_rs485_mux
             jk_bms modbus_rtu jkbms_serial_storage lsm6dsox rgb_led esp_timer trike_sensors Quectel_gps
)


