# Compiles every .c under src/ - adding a test file needs no edit here.
file(GLOB_RECURSE SRCS "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "." "pins" "main" "tests"
                 "tests/current_sensor"
                 "tests/rgb_led"
                 "tests/trike_power"
                 "tests/lis3dhtr"
                 "tests/rs485_mux"
                 "tests/quectel_modem"
                 "tests/quectel_gps"
    REQUIRES driver esp_adc
)
