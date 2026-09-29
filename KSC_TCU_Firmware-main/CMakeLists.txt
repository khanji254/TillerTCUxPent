cmake_minimum_required(VERSION 3.5)

# -----------------------------------------------------------
# Project Name
# -----------------------------------------------------------
set(PROJECT_NAME KSC_TCU)

# -----------------------------------------------------------
# Component Directories
# Organized by subsystem for maintainability
# -----------------------------------------------------------

# Core BMS System
set(COMPONENT_DIR_BMS
    components/jk_bms
    # components/bms_common
    components/bms_monitor_task
    components/bms_monitor_types
    components/bms_rs485_mux
)

# Communication Stack
set(COMPONENT_DIR_COMMS
    components/modbus_rtu
    components/Quectel_mqtt
    components/Quectel_gps
)

# Storage & Caching
set(COMPONENT_DIR_STORAGE
    components/jkbms_serial_storage
    components/trike_sensors
    components/tcu_nvs_creds
)

# Power & Peripherals
set(COMPONENT_DIR_PERIPHERALS	
    components/power_trike_ctrl
    components/rgb_led
    components/lis3dhtr
    components/factory_test   
)

# TCU Board Configurations
set(TCU_CONFIG
    components/board_config
)

# Combine all component directories
set(EXTRA_COMPONENT_DIRS
    ${COMPONENT_DIR_BMS}
    ${COMPONENT_DIR_COMMS}
    ${COMPONENT_DIR_STORAGE}
    ${COMPONENT_DIR_PERIPHERALS}
    ${TCU_CONFIG}
)

# -----------------------------------------------------------
# Include ESP-IDF project system
# -----------------------------------------------------------
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})