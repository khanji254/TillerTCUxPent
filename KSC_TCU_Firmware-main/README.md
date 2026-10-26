# TCU current AP firmware

This is a minimal ESP32-S3 ESP-IDF/PlatformIO application for the TCU Gen 2.1
current-sense circuit.

## Hardware assumptions

- Current-sense output: GPIO5, ADC1 channel 4.
- Reference monitor: GPIO4, ADC1 channel 3. It is displayed and logged, and
  the dashboard also shows a comparison current calculated with this live
  reference.
- Current-sense amplifier: INA241A1, gain 10 V/V.
- Nominal shunt: 0.00050 ohm.
- Current conversion:

  `raw_current_a = (adc_mv - reference_mv) / (gain * shunt_ohm * 1000)`

  `reported_current_a = abs(raw_current_a * scale + offset_a)`

Here, `reference_mv` is the averaged voltage measured on GPIO4. Both corrected
current fields apply the configured gain correction (`scale`) and current
offset (`offset_a`), are converted to positive magnitude with `abs()`, and use
an eight-reading rolling average for the live dashboard and serial display.
The raw current fields remain signed and unfiltered for diagnostics.

The ADC reading is the average of 64 samples. ESP-IDF ADC curve-fitting
calibration is used when eFuse calibration data is available. Normal dashboard
and serial readings use an additional rolling average of eight completed
readings to reduce visible fluctuations. The displayed corrected current is a
positive magnitude; the raw current remains signed for diagnostics and zero
calibration. Long calibration captures are not affected by this live display
average.

## Access point

After flashing, connect to:

- SSID: `TCU-CURRENT`
- Password: `tcu-current`
- Dashboard: `http://192.168.4.1/`

The dashboard refreshes twice per second and provides a JSON endpoint:

`http://192.168.4.1/api/current`

Additional endpoints are:

- `GET /api/capture?n=1000` for a longer averaged calibration reading.
- `POST /api/config?...` to update calibration values.
- `POST /api/zero` to calculate and save the zero-current offset.

## Calibration

The dashboard can update:

- `gain`: INA241 gain.
- `shunt`: shunt resistance in ohms.
- `scale`: empirical scale correction.
- `offset_a`: additive current offset.

Calibration values are validated and stored in NVS. With zero current applied,
use the dashboard's **Zero Current** button. It measures the present raw current and
stores an offset that makes the reported current zero:

`offset_a = -(raw_current_a * scale)`

Use a precision voltmeter and a known shunt as the reference. For a series
control shunt, calculate reference current as:

`reference_current_a = control_shunt_voltage_v / control_shunt_resistance_ohm`

The dashboard stores captured points in the browser, fits `Vadc = a + b * Itrue`,
and suggests a gain correction and offset. Apply the fit only after checking
zero and several known load currents. GPIO4 is the reference used by the main
current calculation; the dashboard's reference-current field exposes the same
calculation for comparison and diagnostics.

The compiled fallback defaults are a 0.00050 ohm shunt and a 5.3 A offset.
Previously saved NVS calibration takes precedence over these defaults; use
**Reset defaults** in the dashboard to replace the saved values.

## Build and upload

Open this directory in PlatformIO and run **Build**, then **Upload**. The
serial monitor is configured for 115200 baud.
