idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        freertos driver nvs_flash esp_system esp_timer
        board_config rgb_led trike_sensors lsm6dsox
        power_trike_ctrl Quectel_mqtt Quectel_gps
        bms_monitor_task jkbms_serial_storage factory_test
        tcu_wifi_ap json
)