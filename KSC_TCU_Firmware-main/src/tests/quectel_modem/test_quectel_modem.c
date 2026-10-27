// TEST 06: Quectel EG915N 4G modem (UART2, GPIO 17 and 18).
// TODO: start UART2 on GPIO 17 (TX) / 18 (RX), send "AT" and wait for OK.
//       Then try ATI, AT+CPIN?, AT+CSQ, AT+CREG?. The modem power and reset pins may be needed.
#include "test_quectel_modem.h"

#include "esp_log.h"
#include "pins/pins.h"

void test_quectel_modem_run(void)
{
    ESP_LOGI("quectel_modem", "not written yet - see the TODO at the top of this file");
}
