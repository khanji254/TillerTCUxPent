# KSC TCU Firmware — Version Comparison & Architecture Delta

**Base Version:** Upstream Initial Release (`Songa/KSC-TCU-Firmware` / `marymbaire/KSC-TCU-Firmware`)  
**Target Version:** Production Release V1.1.0 (`Rift/KSC_TCU_Firmware-main`)  
**Target Platform:** ESP32-S3 · **Framework:** ESP-IDF v5.4.1 / PlatformIO  
**Date of Analysis:** September 2026

---

## 1. Executive Summary

This document details the complete technical delta between the initial baseline firmware release and the enhanced V1.1.0 production firmware for the **KSC Trike Control Unit (TCU)**. 

The baseline release provided initial drivers and scaffolding for cellular MQTT, GPS, RS-485 Modbus, and ADC sampling. However, it had several critical architectural limitations noted in its initial commit (`4b5ac9e`):
- Cross-cycle data pollution from FreeRTOS queue depth mismatches.
- Temporal misalignment between BMS battery current and analog shunt ADC readings (up to 60-second skew).
- Lack of high-frequency accelerometer sampling, resulting in missed road vibration and collision impacts.
- Dropped MQTT URC messages caused by blocking AT command sequences.
- Uncalibrated ADC current offsets and temperature drift.
- Unimplemented motion-aware shutdown sequencing.

The V1.1.0 release resolves all of these limitations through major architectural redesigns across the BMS monitoring engine, sensor calibration pipelines, cellular modem communication stack, power management, and task watchdog subsystem.

---

## 2. Quantitative Change Summary

| Component / Subsystem | Base Lines | Current Lines | Delta Lines | Key Changes |
|---|---|---|---|---|
| `platformio.ini` | 15 | 10 | -5 / +2 | Added 2MB flash config, `direct, esp32_exception_decoder` filters |
| `main/main.c` | 922 | 1042 | +389 / -269 | URC init, task watchdog feeding, FIFO drain task creation, queue removal |
| `board_config.h` | 248 | 277 | +33 / -4 | 2-pack battery count, Steinhart-Hart NTC coeffs, Board 5/8 current offsets, SAR ADC low-mV scale |
| `bms_monitor_task.c` | 531 | 333 | +194 / -392 | Replaced queue with atomic snapshot mutex; temporal shunt capture; shunt-only testing mode |
| `bms_monitor_task.h` | 117 | 177 | +63 / -3 | Snapshot API (`bms_monitor_get_snapshot()`), timing telemetry, shunt statistics getters |
| `bms_monitor_types.h` | 143 | 139 | +3 / -7 | Removed queue sizing macros; battery count configured to 2 |
| `bms_rs485_mux.c` | 110 | 123 | +15 / -2 | Configurable 1, 2, or 3 battery MUX selection arrays |
| `lis3dhtr.c` | 589 | 1060 | +473 / -2 | 32-level FIFO stream engine, 200 Hz drain task, RMS-of-RMS vibration analytics, click interrupt |
| `lis3dhtr.h` | 325 | 546 | +225 / -4 | FIFO register definitions, streaming modes, TEST vs PRODUCTION HPF modes |
| `trike_sensors.c` | 402 | 552 | +162 / -12 | Zero-current offset calibration, NVS persistence, non-blocking 64x oversampling, Steinhart-Hart NTC |
| `trike_sensors.h` | 189 | 203 | +14 / -0 | Public APIs for calibrated offset retrieval and discrete sensor reads |
| `Quectel_mqtt.c` | 1611 | 1737 | +264 / -138 | In-line AT response URC interception, deterministic provisioning derivation, APN reconfiguration |
| `Quectel_mqtt.h` | 396 | 396 | +2 / -2 | Updated URC queue and provisioning prototypes |
| `Quectel_gps.c` | 556 | 637 | +83 / -2 | Thread-safe speed cache with age invalidation, automatic CME 505 error reinitialization |
| `Quectel_gps.h` | 136 | 167 | +31 / -0 | Speed cache read/write prototypes, motion speed thresholds |
| `rgb_led.c` | 254 | 382 | +148 / -20 | Extended alert patterns, `ALERT_TRIKE_STOPPING`, forced override parameter |
| `rgb_led.h` | 115 | 124 | +14 / -5 | `ALERT_TRIKE_STOPPING` enum, `rgb_led_alert_to_str()`, updated function signatures |
| `power_trike_ctrl.c` | 490 | 507 | +31 / -14 | Motion-aware shutdown polling, GPIO40 output configuration, publish retry |
| `tcu_nvs_creds.h` | 124 | 136 | +12 / -0 | Defined device hub name constants (`TCU_DEVICE_1` through `TCU_DEVICE_10`) |
| `factory_test.c` | 828 | 834 | +18 / -12 | Modem AT command timeout tuning, SIM check re-ordering, increased retry headroom |
| `README.md` | 550 | 488 | +449 / -495 | Comprehensive rewrite focusing on latency reduction, current architecture, and telemetry specs |

---

## 3. Subsystem Architectural Breakdowns

### 3.1 BMS Data Pipeline: Queue Removal & Atomic Snapshot Architecture

```
[Baseline Architecture: Queue-Based (DEPRECATED)]
bms_monitor_task ────> xQueueSend(bms_data_queue) ────> mqtt_publish_task
Problems:
  • B1, B2, B3 could come from different cycles if queue wasn't perfectly drained.
  • Stale data persisted across publish cycles.
  • Required complex queue depth monitoring.

[V1.1.0 Architecture: Atomic Snapshot Mutex]
bms_monitor_task (Core 0)
  ├── Pass 1: Serial + current read (B1, B2, B3) + synchronized shunt ADC snaps
  ├── Pass 2: Registers (voltage, SOC, SOH, alarms, cells diff)
  └── Commit to s_cycle_snapshot[] under s_snapshot_mutex (increments cycle_id)
                                  │
mqtt_publish_task (Core 1) <──────┘
  └── bms_monitor_get_snapshot() atomically copies snapshot iff cycle_id is new.
```

- **Elimination of Cross-Cycle Skew:** Individual battery readings (B1, B2, B3) and shunt current measurements are guaranteed to belong to the exact same execution cycle.
- **Deduplication:** `s_last_consumed_cycle` tracking prevents re-publishing duplicate data even if the publish loop triggers early.
- **RS-485 Latency Mitigation:** Support for splitting Modbus operations into Pass 1 (time-critical current/serial) and Pass 2 (slow-varying registers), plus a bench testing **Shunt-Only Mode** where battery current is acquired directly from the analog shunt when no physical battery packs are attached.
- **Configurable Multi-Pack MUX:** Supports 1, 2, or 3 packs via `MUX_CHANNELS[BMS_BATTERY_COUNT]` controlled via GPIO8/GPIO9 (active configuration set to 2 packs).

---

### 3.2 Current Measurement & ADC Calibration Pipeline

The current sensing subsystem underwent three evolutionary iterations:

```
Iteration 1 (Base):
  Shunt read live in metadata payload at publish tick (up to 60s skew from BMS current).
  Result: Spurious ±20 A divergence between BMS sum and shunt reading.

Iteration 2 (Intermediate):
  BMS cycle split into pass 1 and pass 2; shunt sampled once after all batteries.
  Result: Shunt lagged Battery 1 by ~880 ms (problematic during dynamic PWM motor throttle).

Iteration 3 (V1.1.0 Current):
  Shunt ADC oversampled (64x) immediately after each battery's pass 1 current read (~1 ms lag).
  Yields three synchronized points: Average (Field "15"), Peak Max (Field "15b"), and RMS.
```

- **INA240A1PWR Amplifier & Shunt:** Reverted office test shunt (`0.147 Ω`) to production CG FL-2C 300A/75mV bar (`0.00025 Ω`). INA240 gain characterized at 20.0 V/V (nominally 20.3779 V/V).
- **Zero-Current Board Offset Calibration:**
  - Board 0005 actual: `+5.81 A` (currently configured to `+5.23 A`)
  - Board 0006 actual: `+3.93 A`
  - Board 0008 actual: `+2.36 A`
  - Generic multi-board fallback: `+4.87 A`
  - Value stored in NVS under namespace `trike_sens` with key `cur_offset` (milliamps).
- **Steinhart-Hart NTC Temperature Sensing:**
  Replaced simple Beta equation with Steinhart-Hart third-order polynomial modeling for motor NTC 10 kΩ sensor:
  $$ \frac{1}{T} = A + B \ln(R) + C (\ln(R))^3 $$
  - `BOARD_NTC_SH_A = 6.6264337863e-04f`
  - `BOARD_NTC_SH_B = 2.9900068628e-04f`
  - `BOARD_NTC_SH_C = -7.5152777392e-08f`
- **Low-Voltage SAR ADC Correction:**
  Introduced `BOARD_VOLT_SCALE_FACTOR_LOW_MV = 40.4215f` to linearize the ESP32 SAR ADC response below 1000 mV.

---

### 3.3 Accelerometer (LIS3DHTR) High-Speed Analytics

In the baseline firmware, the accelerometer was polled at low frequency and lacked FIFO buffering, missing road shocks, potholes, and short-duration collisions.

- **32-Level Hardware FIFO Stream Engine:**
  - Configured at 200 Hz Output Data Rate (ODR).
  - 32-sample hardware FIFO fills every 160 ms.
- **Dedicated High-Priority Drain Task (`accel_drain_task`):**
  - Runs on Core 0 with Priority 6 (highest on Core 0) with a 160 ms tick deadline.
  - Flushes samples into a software ring buffer without dropping frames.
- **Windowed RMS Vibration Analytics:**
  - Computes 1 Hz windowed RMS, RMS-of-RMS for X, Y, Z axes, 3D vector magnitude, and peak acceleration.
  - Published to ThingsBoard metadata fields `"17"` through `"21"`.
- **Operating Modes:**
  - **TEST Mode:** High-pass filter disabled; DC gravity vector visible (used for bench angle verification).
  - **PRODUCTION Mode:** High-pass filter enabled (1 Hz cutoff at 50 Hz ODR); static gravity removed to isolate road vibration and vehicle dynamics.
- **Hardware Click Interrupt:**
  - Configured on INT1 (GPIO7) for single-tap collision detection (±16 g full-scale, 5 g threshold).
  - Flags instantaneous impact in metadata field `"22"`.

---

### 3.4 Cellular Modem (Quectel EG915N) & MQTT Stack

- **In-Line URC Notification Interception:**
  - In the baseline version, unsolicited result codes (URCs like `+QMTRECV:` and `+QMTSTAT:`) arriving during synchronous AT command execution were discarded by the UART parser.
  - V1.1.0 inspects every line received in `send_at_command()` and posts matched `+QMTRECV:` notifications to `urc_notification_queue`.
  - A dedicated `process_urcs_task` drains the queue every 100 ms, ensuring RPC commands and cloud state changes are never dropped.
- **Self-Provisioning & Credential Derivation:**
  - Deterministic derivation from MAC address and HUB name:
    - `username` = `HUB_NAME` (e.g., `TCU_0003`)
    - `password` = `HUB_NAME + "_ksc"`
    - `client_id` = ESP32 device MAC address
  - Automated fallback repair: if NVS credentials are corrupted or ThingsBoard returns auth code 5 (client banned/unregistered), credentials are wiped and auto-provisioning re-engages automatically.
- **Boot Time Optimization:**
  - Modem power-key boot delay reduced from 40 seconds to 20 seconds.
- **APN Selection:**
  - Configured for cellular IoT network operation (`safaricomiot` / `safaricom`).

---

### 3.5 GPS Speed Caching & Motion Detection Fusion

- **Decoupled Speed Cache:**
  - Direct GPS NMEA reading requires taking the modem UART mutex. Calling this from multiple tasks caused contention.
  - V1.1.0 introduces `gps_update_speed_cache(speed_kmh, fix_valid)` and `gps_get_speed_cache(&speed_kmh)`.
  - Polled at 1 Hz by `diag_log_task`; consumed instantly without blocking by `bms_monitor_task`.
- **Motion Detection Sensor Fusion:**
  - Primary: GPS speed cache (`speed >= 5.0 km/h` considered in motion).
  - Fallback: If GPS fix is invalid or cache is older than 5000 ms, falls back to analog shunt current magnitude (`|I| > 2.0 A`).
- **Autonomous CME 505 Recovery:**
  - If modem reports `+CME ERROR: 505` (GPS engine not initialized / asleep), `Quectel_gps` automatically executes a reinitialization sequence.

---

### 3.6 Power Management & Motion-Aware Trike Shutdown

- **Motion-Guarded Relay Control:**
  - When a remote power-off command (`trike_ctrl_handle_command(1)`) is received, the TCU queries `check_trike_motion()`.
  - If the trike is moving, the shutdown is deferred, and `trike_wait_for_stop_task` is spawned to poll vehicle speed every 5 seconds.
  - The relay gate on GPIO40 is only de-asserted once the trike comes to a complete stop.
- **GPIO40 Direct Configuration:**
  - Migrated from generic `gpio_set_direction()` to full `gpio_config()` with explicit `pull_up_en = DISABLE`, `pull_down_en = DISABLE`, and `intr_type = DISABLE`.
- **Publish Status Retries:**
  - Added immediate retry logic when publishing trike power-off state to ThingsBoard.

---

### 3.7 RGB LED Status Machine

- **New Alert Mode:** Added `ALERT_TRIKE_STOPPING` (rapid blue flashing with all other colors off) to indicate a shutdown command has been received and the trike is waiting to halt.
- **Forced Override API:** Updated `rgb_led_set_alert(mode, force)` allowing high-priority safety states (e.g. shutdown, voltage anomaly) to immediately override standard heartbeat cycles.
- **String Conversion:** Added `rgb_led_alert_to_str()` for transparent console log debugging.
- **Tuned Blink Timers:** Updated `ALERT_LOW_SOC_WARN` to 2000 ms blink and `ALERT_LOW_SOC_CRIT` to 1000 ms blink.

---

### 3.8 FreeRTOS Task Watchdog (Task WDT) Architecture

- **Dedicated Per-Task Registration:**
  - `bms_monitor` (Core 0), `accel_drain` (Core 0), and `mqtt_publish` (Core 1) individually call `esp_task_wdt_add(NULL)` and feed the watchdog once per valid cycle iteration.
  - Production timeout configured to 300 seconds.
- **Decoupled `app_main` Idle Loop:**
  - `app_main` no longer feeds the watchdog in its 1-second idle loop. This prevents an unresponsive worker task from being masked by a healthy `app_main` loop.

---

## 4. Detailed File-by-File Diff Matrix

| File Path | Functional Changes & Rationale |
|---|---|
| `platformio.ini` | Added `board_upload.flash_size = 2MB`, enabled `monitor_filters = direct, esp32_exception_decoder`. Cleaned up commented monitor settings. |
| `CMakeLists.txt` | Partitioned component directories into logical subsystems (`BMS`, `COMMS`, `STORAGE`, `PERIPHERALS`, `TCU_CONFIG`). |
| `main/main.c` | Implemented `accel_drain_task` (Prio 6, Core 0), integrated URC queue initialization, removed `bms_data_queue`, added cycle-ID health monitoring, connected Task WDT to worker tasks, updated diagnostic prints. |
| `components/board_config/include/board_config.h` | Set `BOARD_BMS_BATTERY_COUNT` to 2. Characterized INA240 gain (20.0 V/V). Defined Board 5 offset (`5.23 A`). Added Steinhart-Hart NTC coefficients and low-voltage SAR ADC correction. |
| `components/bms_monitor_task/bms_monitor_task.c` | Replaced FreeRTOS queue with atomic snapshot mutex. Implemented synchronized shunt snapshots. Added shunt-only testing mode. Integrated GPS speed cache motion check. |
| `components/bms_monitor_task/include/bms_monitor_task.h` | Removed `bms_monitor_get_queue_handle()`. Added `bms_monitor_get_snapshot()`, cycle duration getters, and shunt statistics APIs (`avg`, `max`, `rms`). |
| `components/bms_monitor_types/include/bms_monitor_types.h` | Removed `BMS_QUEUE_SIZE_PER_BATTERY` and `BMS_TOTAL_QUEUE_SIZE`. Fixed battery count definition to 2. |
| `components/bms_rs485_mux/bms_rs485_mux.c` | Restructured `MUX_CHANNELS` array to dynamically size against `BMS_BATTERY_COUNT`, with explicit 1-pack, 2-pack, and 3-pack channel definitions. |
| `components/factory_test/factory_test.c` | Re-ordered cellular modem self-test stages (`AT+CPIN?` checked before `AT+CREG?`), tuned timeout parameters and command retry counts. |
| `components/lis3dhtr/lis3dhtr.c` | Implemented FIFO stream drain engine, added `accel_drain_task`, calculated RMS-of-RMS metrics, integrated INT1 click interrupt. |
| `components/lis3dhtr/include/lis3dhtr.h` | Added FIFO register definitions (`FIFO_CTRL_REG`, `FIFO_SRC_REG`), HPF test vs production modes, and vibration metrics structures. |
| `components/power_trike_ctrl/power_trike_ctrl.c` | Implemented motion-guarded shutdown sequencing, configured GPIO40 via `gpio_config()`, added status publish retry. |
| `components/Quectel_gps/Quectel_gps.c` | Implemented thread-safe GPS speed cache (`s_speed_cache`), added automatic recovery for `+CME ERROR: 505` uninitialized GPS states. |
| `components/Quectel_gps/include/Quectel_gps.h` | Added `gps_update_speed_cache()`, `gps_get_speed_cache()`, and speed threshold definitions. |
| `components/Quectel_mqtt/Quectel_mqtt.c` | Added inline URC sniffing in `send_at_command()`, decoupled URC processing into queue, implemented deterministic auto-provisioning credential derivation, reduced boot delay to 20 s. |
| `components/rgb_led/rgb_led.c` | Added `ALERT_TRIKE_STOPPING` support, implemented `force` override in `rgb_led_set_alert()`, created `rgb_led_alert_to_str()`. |
| `components/rgb_led/include/rgb_led.h` | Updated alert enumeration, updated function signatures with force flag, added string converter prototype. |
| `components/tcu_nvs_creds/include/tcu_nvs_creds.h` | Defined hub constants `TCU_DEVICE_1` through `TCU_DEVICE_10` for easy factory flashing. |
| `components/trike_sensors/trike_sensors.c` | Added NVS current offset loading (`cur_offset`), non-blocking 64x oversampling, Steinhart-Hart temperature calculation. |
| `components/trike_sensors/include/trike_sensors.h` | Exposed calibration retrieval functions and discrete channel read helpers. |

---

## 5. Conclusion & Verification Recommendations

1. **RS-485 Modbus Live Bench Verification:** When transitioning from the current office **shunt-only mode** back to full 2-pack/3-pack JK-BMS Modbus communication, verify that Pass 1 / Pass 2 timing adheres to the ~2030 ms latency reduction budget documented in `README.md`.
2. **Current Offset Validation:** Verify that `BOARD_CURRENT_ZERO_OFFSET_A` matches the physical board under test (Board 5 = 5.23 A, Board 8 = 2.36 A, or Generic = 4.87 A).
3. **Accelerometer High-Pass Filter:** Ensure `lis3dhtr_configure_mode(LIS3DHTR_MODE_PRODUCTION)` is asserted before field vehicle deployment so static 1g gravity does not skew vehicle vibration data.
4. **Diagnostic Logging:** Remove or gate the 1 Hz `diag_log_task` before flashing production units to conserve UART bandwidth and CPU cycles.
