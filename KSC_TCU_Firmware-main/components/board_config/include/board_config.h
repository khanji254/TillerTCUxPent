/**
 * @file board_config.h
 * @brief KSC TCU V1.1 Hardware Pin Configuration — ESP32-S3
 *
 * Single source of truth for all GPIO assignments, UART numbers,
 * ADC channels, and hardware-level constants for the KSC TCU
 * Version 1.1 board (ESP32-S3 variant).
 *
 * To port to a new hardware revision only this file needs updating.
 * No application source file should contain raw GPIO_NUM_xx literals
 * outside of this header.
 *
 * @author  Mary Mbugua
 * @date    2026-04-02
 */

#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * BOARD IDENTITY
 * ========================================================================= */

/** PCB hardware revision string — published to ThingsBoard as attribute */
#define BOARD_HW_REVISION           "V1.0.0"

/** Number of JK-BMS batteries managed by this TCU */
#define BOARD_BMS_BATTERY_COUNT      2

/* =========================================================================
 * RGB STATUS LEDs
 * =========================================================================
 *  Red   GPIO1  — Power indicator; always ON at boot; used for fault blink
 *  Green GPIO2  — Normal-operation heartbeat (OFF until system is healthy)
 *  Blue  GPIO42 — MQTT / RPC activity (OFF unless an event is occurring)
 * ========================================================================= */

#define BOARD_LED_RED_PIN            GPIO_NUM_1
#define BOARD_LED_GREEN_PIN          GPIO_NUM_2
#define BOARD_LED_BLUE_PIN           GPIO_NUM_42

/* =========================================================================
 * RS-485 MUX — MC74HC4052ADG Dual 4-Channel Analog Multiplexer
 * =========================================================================
 * The mux routes the single RS-485 UART to one of three battery lines.
 * TX → GPIO12, RX ← GPIO13 (shared through the mux).
 * Channel select pins A (GPIO8) and B (GPIO9):
 *
 *   A=0, B=0  →  Battery 1
 *   A=1, B=0  →  Battery 2
 *   A=0, B=1  →  Battery 3
 *   A=1, B=1  →  Unused
 *
 * DE/RE pins of the three SP3485EN-L transceivers (auto-switched on
 * current board; pins reserved for future manual control):
 *   Battery 1 DE/RE → GPIO14
 *   Battery 2 DE/RE → GPIO21
 *   Battery 3 DE/RE → GPIO38
 * ========================================================================= */

#define BOARD_RS485_UART_NUM         UART_NUM_1
#define BOARD_RS485_TX_PIN           GPIO_NUM_12
#define BOARD_RS485_RX_PIN           GPIO_NUM_13
#define BOARD_RS485_BAUD_RATE        115200

#define BOARD_MUX_SEL_A_PIN          GPIO_NUM_8
#define BOARD_MUX_SEL_B_PIN          GPIO_NUM_9

#define BOARD_BMS1_DE_RE_PIN         GPIO_NUM_14
#define BOARD_BMS2_DE_RE_PIN         GPIO_NUM_21
#define BOARD_BMS3_DE_RE_PIN         GPIO_NUM_38

/* =========================================================================
 * QUECTEL EG915N MODEM — MAIN UART (AT commands + MQTT)
 * =========================================================================
 *  Quectel TXD → GPIO18 (MCU RX)
 *  Quectel RXD ← GPIO17 (MCU TX)
 *  MAIN_RI    → GPIO15 (ring indicator input)
 *  MAIN_DTR   ← GPIO16 (DTR output)
 *  PWR_KEY    ← GPIO11 (active-high)
 *  RST        ← GPIO10 (active-low)
 * ========================================================================= */

#define BOARD_MODEM_UART_NUM         UART_NUM_2
#define BOARD_MODEM_TX_PIN           GPIO_NUM_17
#define BOARD_MODEM_RX_PIN           GPIO_NUM_18
#define BOARD_MODEM_PWR_PIN          GPIO_NUM_11
#define BOARD_MODEM_RST_PIN          GPIO_NUM_10
#define BOARD_MODEM_RI_PIN           GPIO_NUM_15
#define BOARD_MODEM_DTR_PIN          GPIO_NUM_16
#define BOARD_MODEM_BAUD_RATE        115200
#define BOARD_MODEM_BUF_SIZE         832

/* =========================================================================
 * QUECTEL EG915N MODEM — DEBUG UART (continuous NMEA GPS streaming)
 * =========================================================================
 *  Quectel DBG_TXD → GPIO44 (UART0 RX, ESP32-S3 physical pin 37)
 *  Quectel DBG_RXD ← GPIO43 (UART0 TX, ESP32-S3 physical pin 36)
 *  Connected via TXS0108EPWR level shifter.
 * ========================================================================= */

#define BOARD_GPS_UART_NUM           UART_NUM_0
#define BOARD_GPS_TX_PIN             GPIO_NUM_43
#define BOARD_GPS_RX_PIN             GPIO_NUM_44
#define BOARD_GPS_BAUD_RATE          115200
#define BOARD_GPS_BUF_SIZE           512

/* =========================================================================
 * TRIKE POWER CONTROL OUTPUT
 * ========================================================================= */

/** GPIO that drives the trike downstream power relay / MOSFET gate */
#define BOARD_TRIKE_PWR_PIN          GPIO_NUM_40

/* =========================================================================
 * LIS3DHTR ACCELEROMETER — I2C
 * =========================================================================
 *  SCL → GPIO47  (4.7 kΩ external pull-up to 3.3 V)
 *  SDA → GPIO48  (4.7 kΩ external pull-up to 3.3 V)
 *  CS  → 3.3 V via 4.7 kΩ (SPI disabled, I2C mode)
 *  SA0/SDO → GND  →  I2C address = 0x18
 *  INT1 → GPIO7  (hardware interrupt for motion / tap detection)
 *  INT2 → not connected
 * ========================================================================= */

#define BOARD_I2C_PORT               I2C_NUM_0
#define BOARD_I2C_SCL_PIN            GPIO_NUM_47
#define BOARD_I2C_SDA_PIN            GPIO_NUM_48
#define BOARD_I2C_FREQ_HZ            100000
#define BOARD_ACCEL_INT1_PIN         GPIO_NUM_7

/* =========================================================================
 * ADC INPUTS — all on ADC1 (GPIO1–GPIO10) to avoid Wi-Fi / ADC2 conflict
 * =========================================================================
 *  GPIO4 → ADC1_CH3 → VOLTAGE_MEASUREMENT
 *  GPIO5 → ADC1_CH4 → CURRENT_MEASUREMENT
 *  GPIO6 → ADC1_CH5 → NTC temperature
 * ========================================================================= */

#define BOARD_ADC_VOLTAGE_GPIO       GPIO_NUM_4
#define BOARD_ADC_VOLTAGE_CH    	 ADC_CHANNEL_3
#define BOARD_ADC_CURRENT_CH    	 ADC_CHANNEL_4
#define BOARD_ADC_TEMP_CH       	 ADC_CHANNEL_5

#define BOARD_ADC_CURRENT_GPIO       GPIO_NUM_5

#define BOARD_ADC_TEMP_GPIO          GPIO_NUM_6

/** All three ADC channels: 11 dB attenuation → 0–3.1 V measurable range */
#define BOARD_ADC_ATTEN     		ADC_ATTEN_DB_12 
#define BOARD_ADC_VMAX_MV            3100.0f
#define BOARD_ADC_MAX_RAW            4095

/**
 * Number of samples averaged per reading to reduce quantisation noise.
 * At 80 MHz APB (normal TCU operating frequency): 64 × ~10 µs = ~0.64 ms per read.
 * No vTaskDelay inside sample_adc1_mv() — safe to call from any task context
 * without yielding, including inside the pass 1 BMS loop.
 * Note: the earlier ~6 ms estimate assumed 11 MHz APB (light-sleep mode) —
 * that mode is not used by the TCU.
 */
#define BOARD_ADC_OVERSAMPLE_COUNT   64

/* =========================================================================
 * VOLTAGE MEASUREMENT CIRCUIT PARAMETERS
 * =========================================================================
 * Resistive divider:
 *   R_upper = R80 (100 kΩ) + R10 (100 kΩ) = 200 kΩ
 *   R_lower = R11 = 5 kΩ
 *   Vout = Vin × 5 / 205  →  scale factor = 205/5 = 41.0
 *
 * Range check (VIN 56–84 V):
 *   56 V → ADC sees 1.366 V  ✓
 *   84 V → ADC sees 2.049 V  ✓  (both within 11 dB range of 0–3.1 V)
 * ========================================================================= */

#define BOARD_VOLT_R_UPPER_OHMS      200000.0f
//#define BOARD_VOLT_R_UPPER_OHMS      102300.0f
//#define BOARD_VOLT_R_LOWER_OHMS        4800.0f
#define BOARD_VOLT_R_LOWER_OHMS        5000.0f
#define BOARD_VOLT_SCALE_FACTOR      ((BOARD_VOLT_R_UPPER_OHMS + \
                                       BOARD_VOLT_R_LOWER_OHMS) / \
                                       BOARD_VOLT_R_LOWER_OHMS)
/*Change board scale factor to characterize the innacurracies of ESP32 SAR ADC at low mv values <1000mv*/                                       
#define BOARD_VOLT_SCALE_FACTOR_LOW_MV      40.4215f

/* =========================================================================
 * CURRENT MEASUREMENT CIRCUIT PARAMETERS (INA240A1PWR)
 * =========================================================================
 * INA240A1PWR fixed gain = 20 V/V.
 * REF1 & REF2 mid-rail bias:
 *   VDDA (3.3 V) → R12 (10 kΩ) → junction → R13 (10 kΩ) → GND
 *   Vref = 3300 / 2 = 1650 mV
 * Output: Vout = Vref_mV + (Gain × I_shunt_A × R_shunt_Ω) × 1000
 *   (×1000 converts Ω·A = V to mV)
 *
 * Low-pass filter on output: R83 (1 kΩ) + C84 (1.5 µF), f_c ≈ 106 Hz.
 * Signal delivered directly to MCU CURRENT_MEASUREMENT ADC pin.
 *
 * Full-scale at ±60 A with R_shunt = 1 mΩ:
 *   Vout_max = 1650 + (20 × 60 × 0.001 × 1000) = 1650 + 1200 = 2850 mV ✓
 *   Vout_min = 1650 − 1200 = 450 mV ✓
 *
 * SHUNT_RESISTANCE_OHMS is NVS-tunable at runtime.
 * ========================================================================= */

#define BOARD_INA240_GAIN            20.0f
// characterized from 70 measurement points, 2 boards, 2 R values:
// Note: very close to 20 — the INA240 is working correctly.
// The small adjustment accounts for minor component tolerances.
//#define BOARD_INA240_GAIN            20.3779f

#define BOARD_CURRENT_VREF_MV        1650.0f
/* NEW — correct for CG FL-2C 300A/75mV shunt */
/* R = 75 mV / 300 A = 0.00025 Ohm = 250 uOhm */

// ORIGINAL LINE (restore this after testing):
 #define BOARD_SHUNT_RESISTANCE_OHMS  0.00025f

// TEST SESSION LINE (active during office test):
//#define BOARD_SHUNT_RESISTANCE_OHMS  0.147f      /* 1 Ω JINSHAN office test shunt */
#define BOARD_CURRENT_MAX_A          300.0f    /* Rated current of shunt */

// Standard zero-current offset ───────────────────────────
// Added this new define (standardized across all boards, no per-board cal):
// Board 0005 actual: +5.81 A offset
// Board 0006 actual: +3.93 A offset
// Generic 4.87 A keeps worst-case board error within ±0.94 A at zero current

//Initial per board offset
//#define BOARD_CURRENT_ZERO_OFFSET_A  4.87f

//Board 8
//#define BOARD_CURRENT_ZERO_OFFSET_A  2.36f	

//Board 5	
#define BOARD_CURRENT_ZERO_OFFSET_A  5.23

/* =========================================================================
 * NTC TEMPERATURE CIRCUIT PARAMETERS
 * =========================================================================
 * Pull-up resistor R14 = 10 kΩ to VDDA (3.3 V).
 * NTC thermistor between NTC net and GND.
 * Filter capacitor C25 = 100 nF across ADC input to GND.
 *
 * Vout = VDDA × R_ntc / (R_pullup + R_ntc)
 * Steinhart-Hart Beta equation:
 *   1/T = 1/T0 + (1/B) × ln(R/R0)
 *
 * Default: generic 10 kΩ NTC, B = 3950 K.
 * TEMP_OFFSET_DEG_C is a trim value stored in NVS for field calibration.
 * ========================================================================= */

#define BOARD_NTC_R_PULLUP_OHMS      10000.0f
#define BOARD_NTC_R0_OHMS            10000.0f
#define BOARD_NTC_T0_KELVIN          298.15f
#define BOARD_NTC_BETA               3380.0f
#define BOARD_NTC_SH_A        6.6264337863e-04f
#define BOARD_NTC_SH_B        2.9900068628e-04f
#define BOARD_NTC_SH_C       -7.5152777392e-08f
#define BOARD_NTC_VDDA_MV            3300.0f

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H_ */