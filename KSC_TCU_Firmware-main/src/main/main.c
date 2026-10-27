// Edit app_main() to choose which test runs, then click Upload.
// Leave exactly one test call uncommented.
#include "esp_log.h"

#include "tests/current_sensor/test_current_sensor.h"
#include "tests/rgb_led/test_rgb_led.h"
#include "tests/trike_power/test_trike_power.h"
#include "tests/lis3dhtr/test_lis3dhtr.h"
#include "tests/rs485_mux/test_rs485_mux.h"
#include "tests/quectel_modem/test_quectel_modem.h"
#include "tests/quectel_gps/test_quectel_gps.h"

void app_main(void)
{
    ESP_LOGI("main", "TCU test starting");

    test_current_sensor_run();        // 01 working
    // test_rgb_led_run();            // 02 to do
    // test_trike_power_run();        // 03 to do
    // test_lis3dhtr_run();           // 04 to do
    // test_rs485_mux_run();          // 05 to do
    // test_quectel_modem_run();      // 06 to do
    // test_quectel_gps_run();        // 07 to do
}
