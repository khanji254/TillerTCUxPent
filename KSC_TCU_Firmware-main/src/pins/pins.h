#pragma once
// All pin numbers in one place.

// 01 Current sensor
#define PIN_ADC_REF      4       // IO4 reference voltage
#define PIN_ADC_SHUNT    5       // IO5 shunt amplifier output
#define SHUNT_MOHM       0.5f    // shunt resistance in milliohms
#define AMP_GAIN         10.0f   // INA241A11DDFR gain = 10 V/V

// 02 RGB status LEDs
#define PIN_LED_1        1
#define PIN_LED_2        2
#define PIN_LED_3        42

// 03 Trike power relay
#define PIN_TRIKE_POWER  20

// 04 LIS3DHTR accelerometer (I2C)
#define PIN_I2C_SDA      40
#define PIN_I2C_SCL      41

// 05 RS-485 + mux: pins not set yet - take them from the schematic
//    (RSTX, RSRX, MUX_SEL_A, MUX_SEL_B, ESP_DE/~RE_0..2)

// 06 Quectel EG915N modem (UART2)
#define PIN_MODEM_TX     17
#define PIN_MODEM_RX     18

// 07 Quectel GPS NMEA (UART0)
#define PIN_GPS_TX       43
#define PIN_GPS_RX       44
