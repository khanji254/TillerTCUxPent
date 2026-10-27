// TEST 05: RS-485 and multiplexer (battery ports 1..3).
// TODO: add the pin numbers to pins.h first (RSTX, RSRX, MUX_SEL_A/B, ESP_DE/~RE_0..2).
//       Then for each port: set the mux, enable that port's driver, send a BMS request,
//       switch back to receive and print the reply.
#include "test_rs485_mux.h"

#include "esp_log.h"
#include "pins/pins.h"

void test_rs485_mux_run(void)
{
    ESP_LOGI("rs485_mux", "not written yet - see the TODO at the top of this file");
}
