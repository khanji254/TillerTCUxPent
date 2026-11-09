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
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * BOARD IDENTITY
 * ========================================================================= */

/** PCB hardware revision string — published to ThingsBoard as attribute */
#define BOARD_HW_REVISION "V2.1.0"

/** Number of JK-BMS batteries managed by this TCU */
#define BOARD_BMS_BATTERY_COUNT 2

/* =========================================================================
 * STATUS LEDs (TCU Gen 2.1)
 * =========================================================================
 *  Red   GPIO1  — Power indicator; always ON at boot; used for fault blink
 *  Green GPIO37 — Normal-operation heartbeat (OFF until system is healthy)
 *  Blue  GPIO36 — MQTT / RPC activity (OFF unless an event is occurring)
 *  Aux   GPIO35 — General purpose LED (via R70 2.2k)
 * ========================================================================= */

#define BOARD_LED_RED_PIN GPIO_NUM_1
#define BOARD_LED_GREEN_PIN GPIO_NUM_37
#define BOARD_LED_BLUE_PIN GPIO_NUM_36
#define BOARD_LED_AUX_PIN GPIO_NUM_35

/* =========================================================================
 * RS-485 MUX — MC74HC4052ADG Dual 4-Channel Analog Multiplexer (TCU Gen 2.1)
 * =========================================================================
 * The mux routes the single RS-485 UART to one of three battery lines.
 * TX → GPIO12, RX ← GPIO13 (shared through the mux).
 * Channel select pins A (GPIO39) and B (GPIO40):
 *
 *   A=0, B=0  →  Battery 1
 *   A=1, B=0  →  Battery 2
 *   A=0, B=1  →  Battery 3
 *   A=1, B=1  →  Unused
 * ========================================================================= */

#define BOARD_RS485_UART_NUM UART_NUM_1
#define BOARD_RS485_TX_PIN GPIO_NUM_12
#define BOARD_RS485_RX_PIN GPIO_NUM_13
#define BOARD_RS485_BAUD_RATE 115200

#define BOARD_MUX_SEL_A_PIN GPIO_NUM_39
#define BOARD_MUX_SEL_B_PIN GPIO_NUM_40

/* =========================================================================
 * QUECTEL EG915N MODEM — MAIN UART (AT commands + MQTT) (TCU Gen 2.1)
 * =========================================================================
 *  Quectel TXD → GPIO18 (MCU RX)
 *  Quectel RXD ← GPIO17 (MCU TX)
 *  MAIN_RI    → GPIO41 (ring indicator input)
 *  MAIN_DTR   ← GPIO42 (DTR output)
 *  PWR_KEY    ← GPIO11 (active-high)
 *  RST        ← GPIO10 (active-low)
 * ========================================================================= */

#define BOARD_MODEM_UART_NUM UART_NUM_2
#define BOARD_MODEM_TX_PIN GPIO_NUM_17
#define BOARD_MODEM_RX_PIN GPIO_NUM_18
#define BOARD_MODEM_PWR_PIN GPIO_NUM_11
#define BOARD_MODEM_RST_PIN GPIO_NUM_10
#define BOARD_MODEM_RI_PIN GPIO_NUM_41
#define BOARD_MODEM_DTR_PIN GPIO_NUM_42
#define BOARD_MODEM_BAUD_RATE 115200
#define BOARD_MODEM_BUF_SIZE 832

/* =========================================================================
 * QUECTEL EG915N MODEM — DEBUG UART (continuous NMEA GPS streaming)
 * =========================================================================
 *  Quectel DBG_TXD → GPIO44 (UART0 RX, ESP32-S3 physical pin 37)
 *  Quectel DBG_RXD ← GPIO43 (UART0 TX, ESP32-S3 physical pin 36)
 *  Connected via TXS0108EPWR level shifter.
 * ========================================================================= */

#define BOARD_GPS_UART_NUM UART_NUM_0
#define BOARD_GPS_TX_PIN GPIO_NUM_43
#define BOARD_GPS_RX_PIN GPIO_NUM_44
#define BOARD_GPS_BAUD_RATE 115200
#define BOARD_GPS_BUF_SIZE 512

/* =========================================================================
 * TRIKE POWER CONTROL OUTPUT / KILL SWITCH (TCU Gen 2.1)
 * =========================================================================
 * Net Kill_switch on GPIO21 controls high-voltage kill circuit / P-MOSFET gate.
 * Active-high signal asserts power.
 * ========================================================================= */

#define BOARD_TRIKE_PWR_PIN GPIO_NUM_21

/* =========================================================================
 * LSM6DSOX 6-AXIS IMU — I2C
 * =========================================================================
 *  SCL → GPIO47  (4.7 kΩ external pull-up to 3.3 V)
 *  SDA → GPIO48  (4.7 kΩ external pull-up to 3.3 V)
 *  SA0/SDO → GND (0x6A) or VDD (0x6B)
 *  INT1 → GPIO7  (hardware interrupt for motion / wake-up / tap detection)
 * ========================================================================= */

#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_I2C_SCL_PIN GPIO_NUM_47
#define BOARD_I2C_SDA_PIN GPIO_NUM_48
#define BOARD_I2C_FREQ_HZ 100000
#define BOARD_ACCEL_INT1_PIN GPIO_NUM_7

/* =========================================================================
 * USER TERMINAL J10 (5-way screw terminal with TVS protection)
 * ========================================================================= */
#define BOARD_J10_PIN3_GPIO GPIO_NUM_38
#define BOARD_J10_PIN4_GPIO GPIO_NUM_2

/* =========================================================================
 * ADC INPUTS (TCU Gen 2.1)
 * =========================================================================
 *  GPIO4  → ADC1_CH3 → CURRENT_1.65V (INA241 mid-rail bias reference)
 *  GPIO5  → ADC1_CH4 → CURRENT_MEASUREMENT (INA241 output)
 *  GPIO6  → ADC1_CH5 → NTC temperature
 *  GPIO8  → ADC1_CH7 → LOAD_CELL_A (via JP18)
 *  GPIO9  → ADC1_CH8 → LOAD_CELL_A_1.65V
 *  GPIO14 → ADC2_CH3 → VOLTAGE_MEASUREMENT (÷41 resistive divider)
 *  GPIO15 → ADC2_CH4 → LOAD_CELL_B_1.65V
 *  GPIO16 → ADC2_CH5 → LOAD_CELL_B
 * ========================================================================= */

#define BOARD_ADC_VOLTAGE_GPIO     GPIO_NUM_14
#define BOARD_ADC_VOLTAGE_UNIT     ADC_UNIT_2
#define BOARD_ADC_VOLTAGE_CH       ADC_CHANNEL_3

#define BOARD_ADC_CURRENT_REF_GPIO GPIO_NUM_4
#define BOARD_ADC_CURRENT_REF_UNIT ADC_UNIT_1
#define BOARD_ADC_CURRENT_REF_CH   ADC_CHANNEL_3

#define BOARD_ADC_CURRENT_GPIO     GPIO_NUM_5
#define BOARD_ADC_CURRENT_UNIT     ADC_UNIT_1
#define BOARD_ADC_CURRENT_CH       ADC_CHANNEL_4

#define BOARD_ADC_TEMP_GPIO        GPIO_NUM_6
#define BOARD_ADC_TEMP_UNIT        ADC_UNIT_1
#define BOARD_ADC_TEMP_CH          ADC_CHANNEL_5

#define BOARD_ADC_LOAD_CELL_A_GPIO GPIO_NUM_8
#define BOARD_ADC_LOAD_CELL_A_REF_GPIO GPIO_NUM_9
#define BOARD_ADC_LOAD_CELL_B_REF_GPIO GPIO_NUM_15
#define BOARD_ADC_LOAD_CELL_B_GPIO GPIO_NUM_16

/** Attenuation: 12 dB (~11 dB) → 0–3.1 V measurable range */
#define BOARD_ADC_ATTEN ADC_ATTEN_DB_12
#define BOARD_ADC_VMAX_MV 3100.0f
#define BOARD_ADC_MAX_RAW 4095

/**
 * Number of samples averaged per reading to reduce quantisation noise.
 * At 80 MHz APB (normal TCU operating frequency): 64 × ~10 µs = ~0.64 ms per
 * read. No vTaskDelay inside sample_adc1_mv() — safe to call from any task
 * context without yielding, including inside the pass 1 BMS loop. Note: the
 * earlier ~6 ms estimate assumed 11 MHz APB (light-sleep mode) — that mode is
 * not used by the TCU.
 */
#define BOARD_ADC_OVERSAMPLE_COUNT 64

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

#define BOARD_VOLT_R_UPPER_OHMS 200000.0f
// #define BOARD_VOLT_R_UPPER_OHMS      102300.0f
// #define BOARD_VOLT_R_LOWER_OHMS        4800.0f
#define BOARD_VOLT_R_LOWER_OHMS 5000.0f
#define BOARD_VOLT_SCALE_FACTOR                                                \
	((BOARD_VOLT_R_UPPER_OHMS + BOARD_VOLT_R_LOWER_OHMS) /                     \
	 BOARD_VOLT_R_LOWER_OHMS)
/*Change board scale factor to characterize the innacurracies of ESP32 SAR ADC
 * at low mv values <1000mv*/
#define BOARD_VOLT_SCALE_FACTOR_LOW_MV 40.4215f

/* =========================================================================
 * CURRENT MEASUREMENT CIRCUIT PARAMETERS (INA241A1IDDFR)
 * =========================================================================
 * Texas Instruments INA241A1IDDFR (SOT-23-8):
 * Fixed gain = 10 V/V (Note: INA241A1 is 10 V/V, unlike INA240A1 which was 20 V/V).
 * REF1 & REF2 mid-rail bias:
 *   VDDA (3.3 V) → R12 (10 kΩ) → junction → R13 (10 kΩ) → GND
 *   Vref = 3300 / 2 = 1650 mV
 * Output: Vout = Vref_mV + (Gain × I_shunt_A × R_shunt_Ω) × 1000
 *   (×1000 converts Ω·A = V to mV)
 *
 * Low-pass filter on output: R83 (1 kΩ) + C84 (1.5 µF), f_c ≈ 106 Hz.
 * Signal delivered directly to MCU CURRENT_MEASUREMENT ADC pin (GPIO5).
 *
 * SHUNT_RESISTANCE_OHMS is NVS-tunable at runtime.
 * ========================================================================= */

#define BOARD_INA241_GAIN 10.0f
#define BOARD_INA240_GAIN BOARD_INA241_GAIN /* Backward compatibility */

/**
 * Mid-rail zero-current ADC reference voltage in millivolts.
 * Nominal midpoint is 1650.0 mV (3.3V / 2), but measured ADC baseline
 * with current pins shorted on this setup is 1452.0 mV.
 */
#define BOARD_CURRENT_VREF_MV 1452.0f

/**
 * Calibrated effective shunt resistance in Ohms.
 * Calibrated against Fluke 374FC clamp multimeter over live run:
 * Measured transfer sensitivity: 10.60 mV/A (d(Vadc)/dI).
 * With INA240A1 Gain = 20.0 V/V:
 *   R_shunt = 10.60 mV/A / (20.0 * 1000) = 0.000530 Ω (530 µΩ)
 * (Nominal 500 µΩ / 150A-75mV bar with parasitic terminal/trace resistance).
 */
#define BOARD_SHUNT_RESISTANCE_OHMS 0.000500f
#define BOARD_CURRENT_MAX_A 300.0f /* Rated current of shunt */

// Standard zero-current offset ───────────────────────────
// Set to 0.0f default until calibrated with physical shunt in place.
#define BOARD_CURRENT_ZERO_OFFSET_A 0.0f

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

#define BOARD_NTC_R_PULLUP_OHMS 10000.0f
#define BOARD_NTC_R0_OHMS 10000.0f
#define BOARD_NTC_T0_KELVIN 298.15f
#define BOARD_NTC_BETA 3380.0f
#define BOARD_NTC_SH_A 6.6264337863e-04f
#define BOARD_NTC_SH_B 2.9900068628e-04f
#define BOARD_NTC_SH_C -7.5152777392e-08f
#define BOARD_NTC_VDDA_MV 3300.0f

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H_ */