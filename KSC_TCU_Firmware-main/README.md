# KSC TCU V1.1 — Trike Control Unit Firmware

**Platform:** ESP32-S3 · **Framework:** ESP-IDF v5.4.1 · **FW Version:** 1.1.0 · **HW Revision:** V1.0.0  
**ThingsBoard:** `thingsboard.iot.songa.mobi` · **APN:** `safaricomiot`

---

## Table of Contents

1. [URGENT: BMS RS-485 Communication Latency — Current Architecture & Reduction TODO](#1-urgent-bms-rs-485-communication-latency--current-architecture--reduction-todo)
2. [Overview](#2-overview)
3. [Hardware Platform](#3-hardware-platform)
4. [Repository Structure](#4-repository-structure)
5. [Component Architecture](#5-component-architecture)
6. [FreeRTOS Tasks](#6-freertos-tasks)
7. [BMS Data Flow — Queue Removal & Snapshot Architecture](#7-bms-data-flow--queue-removal--snapshot-architecture)
8. [Current Measurement Architecture](#8-current-measurement-architecture)
9. [ThingsBoard Telemetry Fields](#9-thingsboard-telemetry-fields)
10. [NVS Namespaces](#10-nvs-namespaces)
11. [MQTT Provisioning Flow](#11-mqtt-provisioning-flow)
12. [Build & Flash](#12-build--flash)
13. [Review Focus Areas](#13-review-focus-areas)

---

## 1. URGENT: BMS RS-485 Communication Latency — Current Architecture & Reduction TODO

> **Status (as of this update):** Open investigation. Field-measured full read of all 13 parameters from a single JK-BMS battery over RS-485 currently takes **~2030 ms**. This section documents the current pass1/pass2 read architecture and every deliberate delay in the read path, as a starting point for a lag-reduction pass that requires bench/field testing with a live 3-battery rig.

### 1.1 Current read architecture (`bms_monitor_task.c`)

Each BMS cycle reads all `BMS_BATTERY_COUNT` batteries in **two passes** rather than one sequential per-battery read:

```
bms_monitor_task (Core 0, one cycle)
│
├── PASS 1 — serial number + current only, all batteries, time-sensitive
│     for each battery:
│       MUX switch → settle → read serial (retry loop) → read current (retry loop)
│       → snap shunt ADC immediately after this battery's current read
│
├── PASS 2 — all remaining registers, all batteries, non-time-sensitive
│     for each battery (only if pass 1 succeeded):
│       MUX switch → settle → 12 sequential register reads (voltage, SOC, SOH,
│       power, remaining capacity, charge/discharge stat, cycles, alarms,
│       device id/address, cells present, cell count, cells diff)
│
└── Commit staging[] + shunt stats to s_cycle_snapshot[] under mutex
```

This split exists so the **current** reading (the value most sensitive to timing, since it's what gets correlated with the shunt ADC snapshot) is captured for all batteries inside one tight window, before the much slower "everything else" pass runs. See [Section 8](#8-current-measurement-architecture) for why current timing matters.

### 1.2 Where the delay budget goes, per battery

**Pass 1 (`read_battery_pass1()`):**

| Step | Delay | Notes |
|---|---|---|
| MUX switch to battery | — | `bms_rs485_switch_to_battery()`, no fixed delay |
| Bus settle | `100 ms` | Double `uart_flush_input()` around a single 100 ms `vTaskDelay`, before *and* after |
| Serial number read | up to `5 × 50 ms` | `SERIAL_READ_MAX_RETRIES = 5`, `SERIAL_READ_DELAY_MS = 50`; loop exits on first success |
| Current read | up to `2 × 50 ms` | 1 initial attempt + 1 retry, `50 ms` delay before each |
| MUX disable | `10 ms` | Fixed settle after `bms_rs485_disable_battery()` |

Best case (first-try serial + first-try current): **≈210 ms/battery**. Worst case (all serial + current retries exhausted): **≈460 ms/battery**.

**Pass 2 (`read_battery_pass2()`):**

| Step | Delay | Notes |
|---|---|---|
| MUX switch + bus settle | `100 ms` | Same double-flush + 100 ms pattern as pass 1 |
| 12 register reads | `12 × 50 ms` baseline (`READ_REG` macro) | Each register has its own retry loop, up to `3` retries; each retry adds a `30 ms` re-flush delay *on top of* the `50 ms` baseline before every attempt |
| MUX disable | `10 ms` | Fixed settle |

Best case (every register succeeds first try): **≈710 ms/battery**. Worst case with retries on every register is substantially higher — each retried register adds up to `4 × 80 ms` instead of `50 ms`.

**Combined best case per battery (pass 1 + pass 2): ≈920 ms.** The field-measured figure of **~2030 ms/battery** indicates the read path is regularly hitting retries — most likely from Modbus retry loops in pass 2 (12 registers × up to 4 attempts each is the largest lever by far), and/or the two 100 ms MUX-settle windows (one per pass, per battery).

### 1.3 TODO — urgent, needs bench/field testing on the 3-battery rig

1. **Reduce per-battery RS-485 communication time below the current ~2030 ms for all 13 parameters.** This is the top-priority item. It requires real hardware testing (not just static analysis) with the full 3-battery setup, since bus behavior (retry rate, settle requirements) differs from the single-battery field unit currently deployed.

   Candidate levers to test, roughly in order of expected impact:
   - **Pass 2 register read retries** — 12 sequential `READ_REG` calls each allow up to 3 retries with a 30 ms re-flush penalty. Determine whether the observed 2030 ms is dominated by retries (bus noise / JK-BMS unsolicited frame contamination — see the pass1/pass2 UART contamination note in memory) vs. baseline delays, and whether the 50 ms per-attempt floor can be safely lowered.
   - **MUX settle delay** — currently `100 ms` in *both* pass 1 and pass 2 (two 100 ms settles per battery per cycle = 200 ms fixed overhead just from MUX switching). Note: an earlier iteration had reduced the pass 1 settle to `50 ms`, but the code currently in this repo uses `100 ms` in both passes — confirm this was intentional (reverted after intermittent pass 1 failures) before touching it again, and if revisited, validate on a scope per the note in [Section 13](#13-review-focus-areas), item 8.
   - **Register batching** — investigate whether the JK-BMS Modbus interface supports reading multiple contiguous registers in a single transaction instead of 12 separate `READ_REG` calls, which would cut per-register framing/turnaround overhead.
   - **Serial read retry ceiling** — `SERIAL_READ_MAX_RETRIES = 5` may be conservative now that the serial stability window (`bms_serial_validate()`) already guards against truncated frames; worth testing a lower ceiling.
   - **Split pass 2 further** — consider whether all 12 registers need to be read every cycle, or whether slow-changing fields (model_no, device_address, cell_count, cells_present) could be read at a reduced rate (e.g. once every N cycles) rather than every cycle, freeing up per-cycle bus time for the fast-changing fields (voltage, current, SOC, power).

2. Any change here must be validated against the pass1/pass2 UART contamination risk (JK-BMS units broadcast unsolicited ~300-byte status frames every 1–2 s that can contaminate the UART FIFO after a flush) — reducing settle/retry timing too aggressively risks trading latency for reliability.

---

## 2. Overview

The KSC TCU is a cellular IoT gateway mounted on Songa Trikes fleet vehicles. Core responsibilities:

- Monitor **3 × JK-BMS** lithium packs over RS-485 Modbus RTU (MC74HC4052ADG MUX)
- Publish telemetry to **ThingsBoard** over MQTT via **Quectel EG915N** LTE modem
- Stream **GPS position** via the modem's NMEA debug UART
- Control **trike relay gate** (GPIO40) with motion-aware shutdown sequencing
- Measure **pack voltage, pack current, and motor temperature** via ADC1
- Detect **motion, tilt, impact, and free-fall** via LIS3DHTR MEMS accelerometer
- Support **MQTT Basic auto-provisioning** — new units self-register on ThingsBoard

### V1.0 → V1.1 Key Hardware Changes

| Area | V1.0 | V1.1 |
|------|------|------|
| MCU | ESP32 | ESP32-S3 |
| RS-485 routing | 3× GPIO DE/RE pins | MC74HC4052ADG MUX (GPIO8/9 select) |
| Current sense | Discrete shunt | INA240A1PWR amplifier (gain = 20.3779×) |
| Motion detect | None | LIS3DHTR accelerometer (I2C, INT1 = GPIO7) |
| GPS | AT+QGPSLOC polling | Continuous NMEA stream on UART0 |
| LEDs | Single GPIO | RGB: Red=GPIO1, Green=GPIO2, Blue=GPIO42 |
| MQTT credentials | Compile-time macros | NVS-backed + MQTT Basic auto-provisioning |
| Trike power pin | GPIO27 | GPIO40 |
| Mains detect / buzzer / button | Present | Removed |

---

## 3. Hardware Platform

```
ESP32-S3
├── UART0  (GPIO43 TX / GPIO44 RX)  — Quectel debug UART, continuous NMEA @ 115200
├── UART1  (GPIO12 TX / GPIO13 RX)  — RS-485 half-duplex via MUX @ 115200
├── UART2  (GPIO17 TX / GPIO18 RX)  — Quectel main UART (AT + MQTT) @ 115200
├── I2C0   (GPIO47 SCL / GPIO48 SDA) — LIS3DHTR (addr 0x18, 100 kHz)
├── ADC1
│   ├── CH3 (GPIO4)  — Pack voltage (÷41 resistive divider, 56–84 V)
│   ├── CH4 (GPIO5)  — Pack current (INA240A1PWR, Vref = 1650 mV)
│   └── CH5 (GPIO6)  — Motor temperature (NTC 10 kΩ, B = 3380 K)
├── GPIO7   — LIS3DHTR INT1 (click + motion, polled)
├── GPIO8/9 — MUX SEL_A / SEL_B
├── GPIO10  — Quectel RST_N
├── GPIO11  — Quectel PWR_KEY
├── GPIO15  — Quectel RI (input)
├── GPIO16  — Quectel DTR (output)
├── GPIO40  — Trike relay gate
├── GPIO1/2/42 — LED Red / Green / Blue
└── GPIO14/21/38 — BMS1/2/3 DE/RE (reserved)
```

**Current measurement circuit:**
- Shunt: CG FL-2C 300 A / 75 mV bar → R = 0.00025 Ω
- INA240A1PWR: characterised gain = 20.3779 V/V, REF midpoint = 1650 mV
- Formula: `I = (adc_mv − 1650) / (20.3779 × 0.00025 × 1000) + BOARD_CURRENT_ZERO_OFFSET_A`
- Fixed zero offset: `BOARD_CURRENT_ZERO_OFFSET_A = 2.36 A` (board-characterised; Brd0005 = +5.81 A, Brd0006 = +3.93 A)
- ADC: 64-sample oversampling, ~0.64 ms per read at 80 MHz APB, no `vTaskDelay`

---

## 4. Repository Structure

```
KSC_TCU/
├── main/
│   └── main.c              — app_main, mqtt_publish_task, all JSON payload builders
├── components/
│   ├── board_config/       — Single source of truth for all GPIO, UART, ADC constants
│   ├── bms_monitor_task/   — BMS cycling task, atomic snapshot API, shunt statistics
│   ├── bms_monitor_types/  — Shared structs (bms_queued_data_t, bms_system_stats_t)
│   ├── bms_rs485_mux/      — MC74HC4052ADG MUX control
│   ├── jkbms_serial_storage/ — NVS-backed BMS serial number cache
│   ├── trike_sensors/      — ADC voltage / current / temperature
│   ├── lis3dhtr/           — Accelerometer driver (FIFO, click, motion, RMS stats)
│   ├── power_trike_ctrl/   — Trike relay gate + modem power management
│   ├── rgb_led/            — Alert-state LED driver
│   ├── Quectel_mqtt/       — AT command engine, MQTT pub/sub, provisioning workflow
│   ├── Quectel_gps/        — NMEA parser, GPS position + speed cache
│   ├── factory_test/       — Boot self-test suite
│   ├── tcu_nvs_creds/      — MQTT credential NVS store + repair
│   └── version/            — FW_VERSION_STR, HW_VERSION_STR, build date
│
│   (Git submodules — team-owned, shared across projects)
│   ├── jk_bms/             — JK-BMS Modbus RTU register definitions and read functions
│   ├── modbus_rtu/         — Generic Modbus RTU framing layer
│   └── bms_common_data/    — Shared BMS type definitions
├── CMakeLists.txt
└── sdkconfig
```

---

## 5. Component Architecture

### `board_config` (header-only)
Single `.h` file. All GPIO numbers, UART numbers, ADC channels, and circuit constants live here. **No application file should contain raw `GPIO_NUM_xx` literals.** Hardware porting requires editing only this file.

Key constants relevant to code review:
- `BOARD_ADC_OVERSAMPLE_COUNT 64` — 64-sample average, ~0.64 ms at 80 MHz APB 
- `BOARD_INA240_GAIN 20.3779f` — characterised from 70 measurement points across 2 boards
- `BOARD_RS485_BAUD_RATE 115200` — each Modbus RTT is ~73 ms

### `bms_monitor_task`
Manages the BMS read cycle on Core 0, split into a pass1/pass2 architecture — see [Section 1](#1-urgent-bms-rs-485-communication-latency--current-architecture--reduction-todo) for the current per-battery timing breakdown and open latency-reduction work, and [Section 7](#7-bms-data-flow--queue-removal--snapshot-architecture) for the full data-flow description. Key public API:

| Function | Purpose |
|---|---|
| `bms_monitor_get_snapshot(out, cycle_id, shunt_a, shunt_max)` | Consume latest cycle; returns false if no new cycle |
| `bms_monitor_get_ok_count()` | How many batteries read successfully last cycle |
| `bms_monitor_get_shunt_avg/max/rms()` | Per-cycle shunt statistics (see Section 8) |
| `bms_monitor_get_last_cycle_ms()` | Total cycle duration; high values indicate Modbus retries |
| `bms_monitor_get_last_pass1_ms()` | Pass 1 (serial + current) duration |
| `bms_monitor_get_last_pass2_ms()` | Pass 2 (all other registers) duration |

### `trike_sensors`
ADC reads for voltage, current, and temperature. `trike_sensors_read_current()` has **no internal `vTaskDelay`** — safe to call inside the BMS pass 1 loop without adding measurable latency. Calibration constants are NVS-backed and survive reboot; on first boot the `board_config.h` defaults are used.

### `lis3dhtr`
Operates in two modes controlled by `lis3dhtr_configure_mode()`:
- **TEST mode** (boot default): HPF off, gravity visible, used for bench verification
- **PRODUCTION mode**: HPF on (1 Hz cutoff at 50 Hz ODR), DC gravity removed, vibration only

FIFO stream mode is drained by a dedicated `accel_drain_task` (200 Hz ODR, 160 ms drain period — see [Section 6](#6-freertos-tasks)) into a ring buffer, from which 1 Hz windowed RMS accumulation is read non-destructively by `diag_log_task`. The click interrupt (single-tap, ±16 g, 5 g threshold) drives the impact detection flag consumed by `lis3dhtr_consume_impact()` in `mqtt_publish_task`.

### `Quectel_mqtt`
All AT commands serialised through `uart_modem_mutex`. `send_at_command()` uses a static 832-byte response buffer (safe because the mutex prevents re-entrancy). URC notifications (`+QMTRECV:`, `+QMTSTAT:`) are intercepted both inside `send_at_command()` and in the dedicated `check_mqtt_urc()` poll loop running in `process_urcs_task`.

### `power_trike_ctrl`
Combines modem power management and trike relay control. `trike_ctrl_handle_command(1)` (power off) checks `check_trike_motion()` before acting: if the trike is moving it spawns `trike_wait_for_stop_task` which polls every 5 s until motion stops, then executes poweroff. Power state is NVS-backed and restored on boot.

---

## 6. FreeRTOS Tasks

| Task | Core | Priority | Stack | Period / Trigger |
|------|------|----------|-------|-----------------|
| `bms_monitor` | 0 | 5 | 4096 W | Continuous ~6 s cycle |
| `bms_led` | 0 | 4 | 2048 W | 50 ms tick |
| `accel_drain` | 0 | 6 | 3072 W | 160 ms tick (LIS3DHTR FIFO drain, highest priority on Core 0) |
| `mqtt_publish` | 1 | 5 | 6144 W | 60 s publish + impact trigger |
| `process_urcs` | 1 | 5 | 6144 W | 100 ms URC queue poll |
| `stats_monitor` | 1 | 3 | 3072 W | 30 s |
| `diag_log` | 0 | 2 | 5120 W | 1 Hz (**remove before production**) |
| `power_monitor` | 1 | 6 | 4096 W | 500 ms notification wait |
| `trike_wait` | 1 | 4 | 3072 W | Spawned on-demand during poweroff |

**Inter-task communication:**

```
bms_monitor (Core 0)
    └─ writes → s_cycle_snapshot[] + shunt stats  ← protected by s_snapshot_mutex
                                                   ← read by mqtt_publish via bms_monitor_get_snapshot()

accel_drain (Core 0)
    └─ writes → LIS3DHTR ring buffer  ← read by diag_log (1 Hz snapshot) and mqtt_publish (60 s publish stats)

mqtt_publish (Core 1)
    └─ writes → urc_notification_queue  ← read by process_urcs
    └─ reads  → power_event_queue       ← written by power_monitor

diag_log (Core 0)
    └─ writes → gps speed cache (gps_update_speed_cache())
    └─ reads  → bms_monitor_get_shunt_snapshot() + get_shunt_max()
```

**Watchdog:** `bms_monitor`, `accel_drain`, and `mqtt_publish` each self-register with `esp_task_wdt` via `esp_task_wdt_add(NULL)` and reset it once per loop iteration; the configured production timeout is 300 s. **`app_main`'s main loop does *not* register with or feed the watchdog** (the `esp_task_wdt_add(NULL)` and `esp_task_wdt_reset()` calls in `app_main()`'s idle loop are currently commented out) — this is intentional per the watchdog-architecture guidance that `app_main` should not independently feed the WDT, since doing so can mask a single frozen subscriber task. Confirm this remains the case before any refactor of `app_main()`.

---

## 7. BMS Data Flow — Queue Removal & Snapshot Architecture

### What changed (Architecture 1 → current)

**Architecture 1 (original):** A FreeRTOS queue (`bms_data_queue`) held per-battery readings. `xQueueSend()` was called after each battery read; `xQueueReceive()` was called in the publish task. This caused cross-cycle mixing — the publish task could consume B1 from cycle N and B2 from cycle N+1 if the queue was not fully drained.

**Current architecture:** The queue is gone entirely. Battery data flows through a two-stage pipeline:

```
bms_monitor_task (Core 0)
│
├─ PASS 1 (all 3 batteries):
│   read_battery_pass1(B1) → staging[0].batt_i
│   trike_sensors_read_current() → pass1_snaps[0]   ← shunt snap #1
│   read_battery_pass1(B2) → staging[1].batt_i
│   trike_sensors_read_current() → pass1_snaps[1]   ← shunt snap #2
│   read_battery_pass1(B3) → staging[2].batt_i
│   trike_sensors_read_current() → pass1_snaps[2]   ← shunt snap #3
│   compute avg / max / rms from pass1_snaps[]
│
├─ PASS 2 (all 3 batteries):
│   read_battery_pass2(B1/B2/B3) → staging[].batt_v, soc, soh, etc.
│   (batt_i is NOT overwritten in pass 2 — pass 1 value retained)
│
└─ xSemaphoreTake(s_snapshot_mutex)
    memcpy(staging → s_cycle_snapshot[])
    s_shunt_current_snapshot_a = avg
    s_shunt_snap_avg/max/rms   = computed values
    s_snapshot_cycle_id++
   xSemaphoreGive(s_snapshot_mutex)

mqtt_publish / Quectel_mqtt (Core 1)
└─ bms_monitor_get_snapshot(out, &cycle_id, &shunt_a, &shunt_max)
    ← returns false if cycle_id unchanged (no new data)
    ← all three batteries + both shunt values from the same mutex take
    → publish B1/B2/B3 payloads, then metadata payload
```

**Key guarantees:**
- B1, B2, B3 readings and shunt values are always from the same read cycle — no cross-cycle mixing
- The publish task never re-publishes a cycle (cycle ID guard in `s_last_consumed_cycle`)
- Partial publish still works: batteries with `status == BMS_STATUS_NO_DATA` are skipped, `BMS_STATUS_OK` ones are published individually

**Removed from codebase:**
- `bms_data_queue`, `xQueueCreate/Send/Receive` for BMS data
- `BMS_QUEUE_SIZE_PER_BATTERY`, `BMS_TOTAL_QUEUE_SIZE` constants
- `bms_monitor_get_queue_handle()`
- Queue-depth health check in `mqtt_publish_task` → replaced by cycle-ID staleness check

---

## 8. Current Measurement Architecture

Three architectural iterations have been made. The reference document `TCU_Current_Measurement_Architecture.docx` covers all three in full detail. Summary for code review:

### Architecture 1 (removed)
Shunt ADC read live inside `create_metadata_payload()` at publish time — up to 60 s after BMS data was collected. Temporal mismatch caused apparent diffs of ±20 A even on a stationary trike.

### Architecture 2 (intermediate)
BMS cycle split into pass 1 (serial + current for all 3 batteries) and pass 2 (all other registers). One shunt snap taken after the pass 1 loop. Shunt lagged B1 by ~880 ms — still significant at PWM motor current timescales.

### Architecture 3 (current)
One shunt snap taken **immediately after each battery's pass 1 current read** — lag reduced to ~1 ms per battery. Three snaps produce:

| Statistic | Published as | Description |
|---|---|---|
| `avg` | Field `"15"` (P29) | Best single value for energy accounting |
| `max` | Field `"15b"` | Peak load indicator |
| `rms` | `bms_monitor_get_shunt_rms()` | Available, not yet published |

Pass 1 duration: ~1112 ms (dominated by Modbus RTT at 115200 baud — ~8 ms per read). The three shunt snaps add ~2 ms total overhead (<0.2%).

**Dynamic BMS-derived calibration is disabled.** Fixed offset `BOARD_CURRENT_ZERO_OFFSET_A = 2.36 A` is used. The dynamic calibration code (`trike_sensors_update_board_offset`, `first_boot` guard) is retained but commented out pending field validation.

---

## 9. ThingsBoard Telemetry Fields

### Per-battery payload (published 3× per cycle, one per battery)

| Field | Unit | Source |
|---|---|---|
| `Bx_P1` | V | `batt_v / 1000` (pack voltage) |
| `Bx_P2` | A | `batt_i / 1000` (pack current, pass 1 value) |
| `Bx_P3` | % | `soc` |
| `Bx_P4` | % | `soh` |
| `Bx_P5` | cycles | `charge_cycles` |
| `Bx_P6` | — | `status` (0=OK, 1=no data) |
| `Bx_P7` | — | `charge_stat` |
| `Bx_P8` | — | `discharge_stat` |
| `Bx_P9` | — | `serial_no` (string) |
| `Bx_P10` | mV | `cells_diff` (max cell voltage deviation) |
| `Bx_P11` | — | `alarms` bitmask |
| `Bx_P12` | — | `cell_count` |
| `Bx_P13` | Ah | `remaining_capacity / 10000` |
| `Bx_P14` | 0/1 | Voltage anomaly vs other packs (>5000 mV diff) |

### Metadata payload (published once per cycle, after battery payloads)

| Field | Unit | Source |
|---|---|---|
| `"1"` | — | Device serial (MAC string) |
| `"2"` | — | Valid battery count |
| `"3"` | dBm | GSM signal strength |
| `"4"/"5"/"6"` | deg/deg/m | GPS latitude, longitude, altitude |
| `"7"` | — | GPS satellite count |
| `"8"` | km/h | GPS speed |
| `"9"` | — | GPS fix type |
| `"10"` | bool | `resp_cmd` flag (power-off pipeline) |
| `"11"` | bool | `power_confirmation` (trike ON/OFF, NVS-backed) |
| `"12"` | % | Average SOC (all 3 batteries) |
| `"13"` | — | SOC unavailable sentinel (0) |
| `"14"` | V | Pack voltage (live ADC read) |
| `"15"` | A | Shunt current average (pass 1 snaps) |
| `"15b"` | A | Shunt current peak (pass 1 snaps) |
| `"16"` | °C | Motor temperature (live ADC read) |
| `"17"–"21"` | mg | Accelerometer RMS-of-RMS X/Y/Z, vector magnitude, X peak |
| `"22"` | 0/1 | Impact detected (click interrupt) |
| `"23"` | 0/1 | Trike in motion |
| `P_offset_precfg` | A | `BOARD_CURRENT_ZERO_OFFSET_A` constant |
| `P_offset_board` | A | Active current offset (should equal precfg when dynamic cal disabled) |
| `P_bms_cycle_ms` | ms | Total BMS read cycle duration |
| `P_bms_pass1_ms` | ms | Pass 1 duration (current capture window) |
| `P_bms_pass2_ms` | ms | Pass 2 duration |
| `fw` | — | Firmware version string |
| `hw` | — | Hardware revision string |

---

## 10. NVS Namespaces

| Namespace | Component | Keys | Purpose |
|---|---|---|---|
| `tcu_creds` | `tcu_nvs_creds` | `prov_done`, `username`, `password`, `client_id`, `device_name` | MQTT credentials from ThingsBoard provisioning |
| `trike_sens` | `trike_sensors` | `shunt_ohms`, `cur_offset`, `temp_offset` | ADC calibration offsets |
| `trike_ctrl` | `power_trike_ctrl` | `pwr_conf` | Trike ON/OFF state, restored on boot |
| `jkbms_ser` | `jkbms_serial_storage` | `bms0_ser`, `bms1_ser`, `bms2_ser` | BMS serial number cache for fallback on read failure |

---

## 11. MQTT Provisioning Flow

New units ship without credentials. On first boot `tcu_load_or_provision_creds()` is called from `mqtt_subclient()`:

```
Boot
 │
 ├── NVS has prov_done=1 AND credentials load OK?
 │       └── YES → populate mqtt_username/password/client_id → normal MQTT connect
 │
 ├── NVS has prov_done=1 BUT load fails (corruption)?
 │       └── tcu_nvs_repair() → erase namespace → fall through to provisioning
 │
 └── prov_done=0 (never provisioned) → provisioning flow:
         1. Connect to TB with fixed identity: clientId=deviceSerial, user="provision"
         2. Subscribe to /provision/response
         3. Publish to /provision/request (JSON with HUB_NAME, provisionDeviceKey/Secret)
         4. Wait up to TCU_PROV_RESPONSE_TIMEOUT_MS for +QMTRECV: URC
         5. read_buffered_messages() parses response, saves credentials to NVS, esp_restart()
         6. On next boot: Step 1 succeeds → normal operation
```

**Credential derivation** (deterministic, survives NVS corruption + re-provision):
- `username`  = `HUB_NAME` (e.g. `jmbc_0008`)
- `password`  = `HUB_NAME + "_ksc"`
- `client_id` = `deviceSerial` (MAC hex string)

**Device deleted from ThingsBoard:** `open_mqtts_with_creds()` detects CONNACK return code 5 (`+QMTCONN: 0,x,5`), sets `mqtt_auth_failed`, and `mqtts_init()` calls `tcu_force_reprovision()` + `esp_restart()`.

**Trike swap RPC:** Send `{"method":"TRSW","params":0}` from TB dashboard. `read_buffered_messages()` handles `cmd_type == "SW"` by calling `tcu_force_reprovision()` + `esp_restart()`, triggering re-registration under the new trike identity.

---

## 12. Build & Flash

**Prerequisites:** ESP-IDF v5.4.1, Python 3.11, Git

```bash
# Clone (include submodules)
git clone --recurse-submodules https://github.com/<org>/KSC_TCU.git
cd KSC_TCU

# Set IDF environment
. $IDF_PATH/export.sh          # Linux/macOS
# or: C:\Espressif\esp-idf\export.bat   (Windows)

# Build
idf.py build

# Flash via USB OTG (DFU mode — hold BOOT, press RESET, release BOOT)
idf.py -p /dev/ttyACM0 flash   # Linux
idf.py -p COM3 flash           # Windows

# Monitor
idf.py -p /dev/ttyACM0 monitor
```

**To force re-provisioning** (erase credentials only, keep all other NVS):
Uncomment `tcu_nvs_erase_creds()` in `app_main()`, flash once, then recomment and reflash. Alternatively send the `TRSW` RPC from ThingsBoard.

**To set the device hub name** (must be done before first provisioning):
In `app_main()`, edit `snprintf(HUB_NAME, sizeof(HUB_NAME), "%s", TCU_DEVICE_8)` to the correct `TCU_DEVICE_x` constant from `tcu_nvs_creds.h`.

---

## 13. Review Focus Areas

1. **`diag_log_task` must be removed before production deployment.** It runs at 1 Hz on Core 0, calls `trike_sensors_read_voltage/temperature()` live (separate from the BMS-aligned snapshot), and fills the serial log. Gate behind `#ifdef CONFIG_TCU_DIAG_LOG` or delete. Stack is 5120 words.

2. **`BOARD_CURRENT_ZERO_OFFSET_A = 2.36 A` is specific to one board under test.** The generalised value from two-board characterisation is 4.87 A. Confirm the active value matches the deployed hardware before production.

3. **Dynamic BMS calibration is disabled** (`trike_sensors_update_board_offset` and the `first_boot` guard in `bms_monitor_task` are commented out). The code is retained for future use but must not be accidentally re-enabled without simultaneous BMS + shunt timing validation.

4. **`mqtt_pubclient_status()` in `power_trike_ctrl.c`** is called with a `bool` argument in some places (`mqtt_pubclient_status(false)`). The current implementation takes no arguments. Verify all call sites match the current signature.

5. **`bms_monitor_task` stack is 4096 words.** With the `READ_REG` macro expanding 12 register reads in `read_battery_pass2()` and `jk_data_t` on the stack, confirm the high-water mark log (`[MQTT task stack HWM]`) has not shown values below 256 words in field conditions.

6. **GPS UART0 / ESP-IDF console conflict.** `board_config.h` assigns UART0 (GPIO43/44) to GPS NMEA streaming. UART0 is the default IDF console. Verify `sdkconfig` has `CONFIG_ESP_CONSOLE_USB_CDC=y` or `CONFIG_ESP_CONSOLE_UART_NONE=y` — if not, IDF boot logs corrupt GPS data before `gps_init()` runs.

7. **`read_battery_pass2()` re-reads `batt_i` but discards the result.** The commented-out line `/* queued_data->batt_i = jk_data_temp.batt_i; */` is intentional — pass 1 current is retained for temporal alignment with the shunt snap. The Modbus transaction still executes. This is correct by design but unusual; the comment explains the reason.

8. **MUX settle is currently `100 ms` in *both* pass 1 and pass 2** (`vTaskDelay(pdMS_TO_TICKS(100))`, double-flushed). An earlier iteration had reduced the pass 1 settle to 50 ms, but the code currently in this repo has reverted to 100 ms in both passes. This is now one of the candidate levers being tested as part of the [urgent RS-485 latency reduction TODO](#1-urgent-bms-rs-485-communication-latency--current-architecture--reduction-todo) — do not shorten it again without scope validation on the RS-485 lines under worst-case MUX switching conditions with the full 3-battery rig.

9. **`s_impact_pending` flag in `lis3dhtr.c` is set inside `lis3dhtr_read_motion()`** and consumed by `lis3dhtr_consume_impact()`. `read_motion()` is only called from the FIFO fallback path — in normal FIFO operation (drained by `accel_drain_task`) the flag is never set via this path. Impact detection currently relies entirely on `lis3dhtr_check_clear_impact()` being polled indirectly. Review whether the click interrupt path is fully exercised when FIFO is enabled.

10. **`power_mgmt_recover_modem()` in `power_trike_ctrl.c`** calls `open_mqtts_with_creds(mqtt_client_id, mqtt_username, mqtt_password)` directly. These are `Quectel_mqtt.c` module-level globals accessed as externals. This works in the monolithic binary but creates a tight coupling. If `Quectel_mqtt` is ever separated into a library, the linker will fail.