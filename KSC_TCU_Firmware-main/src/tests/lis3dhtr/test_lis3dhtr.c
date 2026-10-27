// TEST 04: LIS3DHTR accelerometer (I2C, GPIO 40 and 41).
// TODO: start I2C on GPIO 40/41, find the chip at 0x18 or 0x19, read WHO_AM_I (should be 0x33),
//       then read and print X, Y, Z. Check which pin is SDA and which is SCL.
#include "test_lis3dhtr.h"

#include "esp_log.h"
#include "pins/pins.h"

void test_lis3dhtr_run(void)
{
    ESP_LOGI("lis3dhtr", "not written yet - see the TODO at the top of this file");
}
