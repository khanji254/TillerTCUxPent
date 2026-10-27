// TEST 07: Quectel GPS NMEA stream (UART0, GPIO 43 and 44).
// TODO: read NMEA lines from UART0 on GPIO 43/44 and print them (look for GGA and RMC).
//       UART0 is also the console, so print over the USB port instead.
#include "test_quectel_gps.h"

#include "esp_log.h"
#include "pins/pins.h"

void test_quectel_gps_run(void)
{
    ESP_LOGI("quectel_gps", "not written yet - see the TODO at the top of this file");
}
