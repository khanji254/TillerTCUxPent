idf_component_register(
    SRCS "bms_monitor_task.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos driver board_config bms_monitor_types bms_rs485_mux
             jk_bms modbus_rtu jkbms_serial_storage lis3dhtr rgb_led esp_timer trike_sensors Quectel_gps
)


