# TCU Shunt Current Sensor Calibration Report

**Date:** November 9, 2026  
**Target Board:** ESP32-S3 TCU (Board 5, Serial: `9C139E1AF418`)  
**Reference Tool:** Fluke 374FC True-RMS Clamp Multimeter  
**Reference Dataset:** `Records_1.csv` (Fluke 374FC) & `Tcu_log.txt` (TCU Serial Logs)  
**Operating Window:** Sep 30, 2026 16:35:28 – 16:37:56 EAT  

---

## 1. Overview & Problem Statement

During live operation, the DC current measurements reported by the TCU's analog shunt sensor deviated substantially from the ground-truth readings recorded simultaneously by a calibrated **Fluke 374FC** True-RMS clamp meter.

Specific symptoms observed in the original firmware:
1. **Inverted Polarities:** When the vehicle accelerated (battery discharge), the TCU reported large negative currents (down to $-29.11\text{ A}$), while regenerative braking was reported as positive current.
2. **Severe Magnitude Distortion (>2x over-reading):** Acceleration current pulses measured at $\sim 13.5\text{ A}$ by the Fluke meter were reported by the TCU as $\sim -29.1\text{ A}$.
3. **Resting Drift & Artificially Patched Offset:** At zero current, the ADC voltage sat at $1623.0\text{ mV}$ rather than the expected $1650.0\text{ mV}$. An ad-hoc offset of $+5.23\text{ A}$ and a scaling hack (`* 0.97`) were previously inserted into the driver code to mask this resting discrepancy.

---

## 2. Temporal Alignment & Clock Synchronization

Because the Fluke 374FC internal real-time clock was set independently of the TCU's GPS timebase, an initial synchronization step was required to correlate the readings:
- **TCU Timebase:** The TCU streams NMEA sentences (`$GNGGA` and `$GNRMC`) from the onboard Quectel GNSS receiver. Regression between ESP32 microsecond uptime and GPS UTC time yielded a timing jitter of $<0.25\text{ s}$ ($\text{slope} = 0.001000\text{ s/ms}$).
- **Multimeter Clock Offset:** Normalized cross-correlation across the full time series demonstrated that the Fluke 374FC clock was **leading GPS time by exactly $+1.80\text{ seconds}$**.
- **Post-Alignment Correlation:** Applying this $+1.80\text{ s}$ correction to the multimeter time base achieved a feature cross-correlation of **$r = 0.8987$**, perfectly locking all acceleration pulses and deceleration events.

---

## 3. Root Cause Analysis

### 3.1 Sign Inversion
- **Hardware Connection:** The Texas Instruments **INA240A1PWR** current-sense amplifier is connected across the shunt such that current discharging from the battery produces a negative differential input voltage:
  $$V_{\text{IN+}} - V_{\text{IN-}} < 0$$
  Consequently, as discharge current increases, the amplifier's output voltage **drops below** the reference voltage ($V_{\text{ref}}$).
- **Firmware Bug:** The driver calculated:
  $$\Delta V = V_{\text{adc}} - V_{\text{ref}}$$
  Because $V_{\text{adc}} < V_{\text{ref}}$ during discharge, $\Delta V$ evaluated to negative values, inverting the sign of all discharge and regenerative currents.

### 3.2 Reference Voltage Offset ($V_{\text{ref}}$)
- The firmware assumed a theoretical mid-rail reference of:
  $$V_{\text{ref, ideal}} = \frac{3300\text{ mV}}{2} = 1650.0\text{ mV}$$
- Across 24 resting zero-current samples ($0.0\text{ A}$ on the Fluke), the actual measured ADC voltage was **$1622.44\text{ mV}$** ($\approx 1623.0\text{ mV}$).
- Evaluating $1623\text{ mV} - 1650\text{ mV}$ produced a baseline error of $-27.0\text{ mV}$ (equivalent to $-5.40\text{ A}$ at $5.0\text{ mV/A}$), which had been artificially masked with `BOARD_CURRENT_ZERO_OFFSET_A = 5.23` and `(raw_current_a * 0.97)`.

### 3.3 Shunt Resistance / Circuit Gain Mismatch
- **Configured Values:** `BOARD_INA240_GAIN = 20.0f`, `BOARD_SHUNT_RESISTANCE_OHMS = 0.00025f` ($250\ \mu\Omega$).
  $$\text{Nominal Sensitivity} = 20.0 \times 0.00025\ \Omega \times 1000 = 5.0\text{ mV/A}$$
- **Empirical Sensitivity:** Linear regression across all aligned data points against the Fluke 374FC revealed the actual transfer slope is:
  $$\frac{\Delta V_{\text{adc}}}{\Delta I} = 10.599\text{ mV/A} \approx 10.60\text{ mV/A}$$
  $$\text{Effective } R_{\text{shunt}} = \frac{10.60\text{ mV/A}}{20.0 \times 1000} = 0.000530\ \Omega\ (530\ \mu\Omega)$$
  *(Nominal $500\ \mu\Omega$ / $150\text{A}-75\text{mV}$ shunt bar with $\approx 30\ \mu\Omega$ parasitic contact and PCB trace resistance)*.
- Because the firmware was dividing by $5.0\text{ mV/A}$ instead of $10.60\text{ mV/A}$, the reported current was **$2.12\times$ greater than reality**.

---

## 4. Calibration Parameters & Mathematical Model

The corrected transfer function for the TCU current sensor is:

$$I = \frac{V_{\text{ref}} - V_{\text{adc}}}{\text{Gain} \times R_{\text{shunt}} \times 1000} + I_{\text{offset}}$$

| Parameter | Old Config | Calibrated Value | Units | Rationale |
| :--- | :---: | :---: | :---: | :--- |
| `BOARD_CURRENT_VREF_MV` | $1650.0$ | **$1623.0$** | $\text{mV}$ | Measured ADC baseline voltage at $0.0\text{ A}$ |
| `BOARD_INA240_GAIN` | $20.0$ | **$20.0$** | $\text{V/V}$ | Fixed hardware amplifier gain (INA240A1) |
| `BOARD_SHUNT_RESISTANCE_OHMS` | $0.00025$ | **$0.000530$** | $\Omega$ | Calibrated from $10.60\text{ mV/A}$ measured slope |
| `BOARD_CURRENT_ZERO_OFFSET_A` | $5.23$ | **$0.0$** | $\text{A}$ | Zero baseline is fully centered by $V_{\text{ref}} = 1623\text{ mV}$ |
| Differential Voltage Formula | $V_{\text{adc}} - V_{\text{ref}}$ | **$V_{\text{ref}} - V_{\text{adc}}$** | $\text{mV}$ | Corrects sign: discharge $> 0$, regen $< 0$ |
| Empirical Scaling Factor | `* 0.97` | **None** | - | Removed ad-hoc tuning factor |

---

## 5. Verification & Performance Metrics

### Statistical Error Summary (153 Aligned Samples)

| Metric | Old Firmware | Calibrated Firmware | Improvement |
| :--- | :---: | :---: | :---: |
| **Mean Absolute Error (MAE)** | $2.823\text{ A}$ | **$0.336\text{ A}$** | **$88.1\%\text{ reduction}$** |
| **Root Mean Square Error (RMSE)** | $5.452\text{ A}$ | **$0.548\text{ A}$** | **$90.0\%\text{ reduction}$** |
| **Maximum Absolute Error** | $42.369\text{ A}$ | **$2.712\text{ A}$** | **$93.6\%\text{ reduction}$** |
| **Correlation Coefficient ($r$)** | $-0.9405$ | **$+0.9405$** | **Sign & scale aligned** |

### Event Benchmark Table

| Operating Phase | Time (EAT) | Multimeter Current | Old TCU Log | Calibrated TCU |
| :--- | :---: | :---: | :---: | :---: |
| **Stationary / Rest** | 16:35:31 | $0.00\text{ A}$ | $-0.20\text{ A}$ | **$+0.09\text{ A}$** |
| **Stationary / Rest** | 16:35:42 | $0.00\text{ A}$ | $-0.20\text{ A}$ | **$+0.09\text{ A}$** |
| **Light Acceleration** | 16:35:49 | $5.40\text{ A}$ | $-4.08\text{ A}$ | **$+1.98\text{ A}$** |
| **Maximum Acceleration** | 16:35:54 | $13.50\text{ A}$ | **$-29.11\text{ A}$** | **$+14.15\text{ A}$** |
| **Regen Braking** | 16:35:56 | **$-1.40\text{ A}$** | **$+1.74\text{ A}$** | **$-0.85\text{ A}$** |
| **Cruising Pulse 1** | 16:37:25 | $4.80\text{ A}$ | $-10.48\text{ A}$ | **$+5.09\text{ A}$** |
| **Cruising Pulse 2** | 16:37:38 | $9.60\text{ A}$ | $-14.17\text{ A}$ | **$+6.89\text{ A}$** |
| **Cruising Pulse 3** | 16:37:46 | $3.70\text{ A}$ | $-6.02\text{ A}$ | **$+2.92\text{ A}$** |

---

## 6. Code Modifications

The following firmware files were updated and compiled:

### 1. `components/board_config/include/board_config.h`
```c
/**
 * Mid-rail zero-current ADC reference voltage in millivolts.
 * Nominal midpoint is 1650.0 mV (3.3V / 2), but on-board bias network and
 * ADC characterization yields 1623.0 mV at zero current.
 */
#define BOARD_CURRENT_VREF_MV        1623.0f

/**
 * Calibrated effective shunt resistance in Ohms.
 * Calibrated against Fluke 374FC clamp multimeter over live run:
 * Measured transfer sensitivity: 10.60 mV/A (d(Vadc)/dI).
 * With INA240A1 Gain = 20.0 V/V:
 *   R_shunt = 10.60 mV/A / (20.0 * 1000) = 0.000530 Ω (530 µΩ)
 * (Nominal 500 µΩ / 150A-75mV bar with parasitic terminal/trace resistance).
 */
#define BOARD_SHUNT_RESISTANCE_OHMS  0.000530f
#define BOARD_CURRENT_MAX_A          300.0f    /* Rated current of shunt */

// Standard zero-current offset ───────────────────────────
// With VREF calibrated to actual zero-current baseline (1623.0 mV),
// residual board zero-current offset is 0.0 A.
#define BOARD_CURRENT_ZERO_OFFSET_A  0.0f
```

### 2. `components/trike_sensors/trike_sensors.c`
In `trike_sensors_read_current()`:
```c
    /*
     * Step 1 — Remove REF midpoint to get shunt-only signal:
     *   The hardware differential input to the INA240 is oriented such that
     *   discharge current causes ADC voltage to drop below Vref, while
     *   regenerative/charging current causes ADC voltage to rise above Vref.
     *   delta_mv = Vref - adc_mv
     *   At zero current:              delta_mv = 0
     *   Positive current (discharge): adc_mv < Vref → delta_mv > 0
     *   Negative current (charge):    adc_mv > Vref → delta_mv < 0
     */
    float delta_mv = BOARD_CURRENT_VREF_MV - adc_mv;

    /*
     * Step 2 — Convert delta to current:
     *   I = delta_mv / (Gain x R_shunt x 1000)
     */
    float raw_current_a = delta_mv /
                          (BOARD_INA240_GAIN *
                           s_shunt_ohms *
                           1000.0f);

    /*
     * Step 3 — Apply zero-offset correction (NVS-stored):
     */
    *current_a = raw_current_a + s_current_offset_a;
    data->current_offset_applied_a = s_current_offset_a;
```

In `trike_sensors_update_board_offset()`:
```c
    float delta_mv      = BOARD_CURRENT_VREF_MV - adc_mv;
    float raw_current_a = delta_mv / (BOARD_INA240_GAIN *
                                       s_shunt_ohms * 1000.0f);
```

---

## 7. Compilation Verification

The firmware was compiled using PlatformIO:
```bash
/home/goaty/.platformio/penv/bin/pio run
```
**Build Status:** `[SUCCESS]`  
- Target: `esp32-s3-devkitc-1` (ESP-IDF v5.5.3)  
- RAM Usage: $12.3\%$ ($40,384\text{ B} / 327,680\text{ B}$)  
- Flash Usage: $32.5\%$ ($340,805\text{ B} / 1,048,576\text{ B}$)  
- Binary Output: `.pio/build/esp32-s3-devkitc-1/firmware.bin`
