# KSC TCU V1.1 — Trike Control Unit Firmware

**Platform:** ESP32-S3 · **Framework:** ESP-IDF v5.4.1 / PlatformIO · **FW Version:** 1.1.0 · **HW Revision:** V1.0.0  
**ThingsBoard:** `thingsboard.iot.songa.mobi` · **APN:** `safaricom` / `safaricomiot`

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware Platform & Pinout](#2-hardware-platform--pinout)
3. [Repository Structure](#3-repository-structure)
4. [Operating Modes: RS-485 Modbus vs Shunt-Only Mode](#4-operating-modes-rs-485-modbus-vs-shunt-only-mode)
5. [URGENT: BMS RS-485 Communication Latency — Investigation & Optimization](#5-urgent-bms-rs-485-communication-latency--investigation--optimization)
6. [Component Architecture & Data Flow](#6-component-architecture--data-flow)
   - [6.1 BMS Data Pipeline & Atomic Snapshot API](#61-bms-data-pipeline--atomic-snapshot-api)
   - [6.2 Current Measurement & Multi-Point Calibration](#62-current-measurement--multi-point-calibration)
   - [6.3 IMU Accelerometer (LSM6DSOX / LIS3DHTR) & High-Speed FIFO Engine](#63-imu-accelerometer-lsm6dsox--lis3dhtr--high-speed-fifo-engine)
   - [6.4 Quectel Cellular Modem, URC Queue & GPS Speed Cache](#64-quectel-cellular-modem-urc-queue--gps-speed-cache)
   - [6.5 Power Management & Motion-Aware Trike Shutdown](#65-power-management--motion-aware-trike-shutdown)
   - [6.6 RGB LED Alert Management](#66-rgb-led-alert-management)
   - [6.7 Wi-Fi SoftAP & Live Telemetry Web Portal](#67-wi-fi-softap--live-telemetry-web-portal)
7. [Wi-Fi SoftAP & Field Web Portal Specification](#7-wi-fi-softap--field-web-portal-specification)
8. [FreeRTOS Tasks & Core Allocation](#8-freertos-tasks--core-allocation)
9. [ThingsBoard Telemetry & Attributes Specification](#9-thingsboard-telemetry--attributes-specification)
10. [NVS Namespaces & Storage](#10-nvs-namespaces--storage)
11. [MQTT Auto-Provisioning & Credential Lifecycle](#11-mqtt-auto-provisioning--credential-lifecycle)
12. [Factory Boot Self-Test Suite](#12-factory-boot-self-test-suite)
13. [Build, Flash & Monitor](#13-build-flash--monitor)
14. [Version Comparison & Technical Delta](#14-version-comparison--technical-delta)
15. [Review Focus Areas & Production Checklist](#15-review-focus-areas--production-checklist)

---

## 1. Overview

The KSC TCU (Trike Control Unit) is an industrial cellular IoT telematics gateway engineered for electric three-wheel fleet vehicles (Songa Trikes). The TCU is powered by an ESP32-S3 dual-core microcontroller and interfaces with the vehicle's traction battery system, motor controller, cellular network, and cloud platform.

### Core Capabilities:
- **Multi-Pack Battery Management:** Interfaces with up to 3 × JK-BMS lithium battery packs over RS-485 Modbus RTU using a 2-channel analog multiplexer (MC74HC4052ADG).
- **Cellular Cloud Telemetry:** Publishes telemetry, alarms, and diagnostics over MQTT to ThingsBoard via a Quectel EG915N LTE modem.
- **Precision Current Sensing:** High-side current measurement utilizing an INA240A1PWR bidirectional amplifier and a CG FL-2C 300A / 75mV precision shunt.
- **Dynamic Motion & Vibration Analytics:** High-frequency 208 Hz LSM6DSOX 6-axis IMU stream processing (with LIS3DHTR compatibility) using a 32-sample hardware FIFO to detect road roughness, RMS vibration, and collision impacts.
- **Continuous NMEA GPS Tracking:** Real-time speed and position caching with motion detection sensor fusion.
- **Motion-Guarded Relay Actuation:** Trike main power contactor control (GPIO40) that guarantees the vehicle has come to a complete stop before power de-assertion.
- **Local Wi-Fi Access Point & Live Diagnostic Portal:** Embedded Wi-Fi SoftAP (`TCU_<serial>`) with captive DNS and a standalone web dashboard for live field monitoring and 1-click CSV/JSON telemetry download.
- **Zero-Touch Provisioning:** Deterministic MQTT Basic auto-provisioning against ThingsBoard with autonomous NVS repair.

### V1.0 → V1.1 Key Hardware Changes

| Area | V1.0 (ESP32) | V1.1 (ESP32-S3) |
|---|---|---|
| MCU | ESP32-D0WDQ6 | ESP32-S3 (Dual-core Xtensa LX7 @ 240 MHz) |
| RS-485 Routing | 3× Discrete GPIO DE/RE lines | MC74HC4052ADG MUX (GPIO8/9 select) |
| Current Sense | Discrete op-amp shunt circuit | INA240A1PWR amplifier (Gain = 20.0× / 20.3779×) |
| Motion Detection | None | LSM6DSOX 6-axis IMU / LIS3DHTR (I2C, INT1 on GPIO7) |
| Field Diagnostics| Serial monitor only | Wi-Fi SoftAP (`192.168.4.1`) & Live Web Dashboard |
| GPS Pipeline | Periodic `AT+QGPSLOC` polling | Continuous NMEA stream on UART0 with speed cache |
| Status LEDs | Single monochrome LED | RGB LED: Red=GPIO1, Green=GPIO2, Blue=GPIO42 |
| Device Provisioning| Hardcoded compile-time tokens | NVS-backed dynamic auto-provisioning via ThingsBoard |
| Trike Power Pin | GPIO27 | GPIO40 |

---

## 2. Hardware Platform & Pinout

```
ESP32-S3 (WROOM-1 / DevKitC-1)
├── UART0  (GPIO43 TX / GPIO44 RX)  — Quectel debug UART: Continuous NMEA @ 115200 baud
├── UART1  (GPIO12 TX / GPIO13 RX)  — RS-485 half-duplex via MUX @ 115200 baud
├── UART2  (GPIO17 TX / GPIO18 RX)  — Quectel main AT + MQTT command port @ 115200 baud
├── I2C0   (GPIO47 SCL / GPIO48 SDA) — LSM6DSOX 6-axis IMU / LIS3DHTR (Addr 0x6A/0x6B, 100 kHz)
├── ADC1   (Dedicated ADC1 prevents Wi-Fi RF / ADC2 channel conflict)
│   ├── CH3 (GPIO4)  — Traction pack voltage (÷41 resistive divider, 56–84 V range)
│   ├── CH4 (GPIO5)  — Shunt current sense (INA240A1PWR, Vref = 1650 mV)
│   └── CH5 (GPIO6)  — Motor temperature (NTC 10 kΩ, Steinhart-Hart B=3380 K)
├── Wi-Fi  (Internal 2.4 GHz RF)     — SoftAP (192.168.4.1), Captive DNS & Live HTTP Web Portal
├── GPIO7   — IMU INT1 (hardware tap/click/motion collision interrupt)
├── GPIO8   — RS-485 MUX Select A (`SEL_A`)
├── GPIO9   — RS-485 MUX Select B (`SEL_B`)
├── GPIO10  — Quectel Modem Reset (`RST_N`, active low)
├── GPIO11  — Quectel Power Key (`PWR_KEY`, active low pulse)
├── GPIO15  — Quectel Ring Indicator (`RI`, input)
├── GPIO16  — Quectel Data Terminal Ready (`DTR`, output)
├── GPIO40  — Trike relay gate control (active high)
├── GPIO1   — Status LED Red
├── GPIO2   — Status LED Green
└── GPIO42  — Status LED Blue
```

### Current Measurement Circuit Parameters:
- **Shunt Resistor:** CG FL-2C 300 A / 75 mV manganese-copper bar $\rightarrow R = 0.00025\ \Omega$.
- **Amplifier:** INA240A1PWR characterized at $20.0\ \text{V/V}$ (nominal characterization $20.3779\ \text{V/V}$, $V_{\text{REF}} = 1650\ \text{mV}$).
- **Transfer Function:**
  $$I_{\text{amps}} = \frac{V_{\text{ADC\_mV}} - 1650.0}{20.0 \times 0.00025 \times 1000} + I_{\text{zero\_offset}}$$
- **Active Board Zero Offset:**
  - Board 0005: `+5.23 A` (active in `board_config.h`)
  - Board 0006: `+3.93 A`
  - Board 0008: `+2.36 A`
  - Multi-Board Generic: `+4.87 A`
- **Oversampling:** 64-sample hardware average per conversion (~0.64 ms at 80 MHz APB, no `vTaskDelay`).

---

## 3. Repository Structure

```
KSC_TCU/
├── main/
│   ├── CMakeLists.txt
│   └── main.c                        — System init, task spawning, WDT, publish loop
├── components/
│   ├── board_config/                 — Master hardware pinouts, ADC constants, and version.h
│   ├── bms_monitor_task/             — BMS cycling engine, atomic snapshot mutex, shunt telemetry
│   ├── bms_monitor_types/            — Shared structs (bms_queued_data_t, bms_system_stats_t)
│   ├── bms_rs485_mux/                — MC74HC4052ADG 2-channel RS-485 multiplexer driver
│   ├── jkbms_serial_storage/         — NVS-backed serial number cache for degraded mode
│   ├── trike_sensors/                — ADC voltage, current, and Steinhart-Hart temperature engine
│   ├── lsm6dsox/                     — LSM6DSOX 6-axis IMU driver, FIFO stream, RMS vibration analytics
│   ├── lis3dhtr/                     — LIS3DHTR legacy compatibility wrapper
│   ├── power_trike_ctrl/             — Motion-aware contactor sequencing and modem power
│   ├── rgb_led/                      — Non-blocking state-machine RGB status indicator
│   ├── Quectel_mqtt/                 — AT command engine, in-line URC sniffer, auto-provisioning
│   ├── Quectel_gps/                  — NMEA parser, asynchronous speed and position cache
│   ├── tcu_wifi_ap/                  — Wi-Fi SoftAP, captive DNS server, live web dashboard & CSV export
│   ├── factory_test/                 — Production line boot self-test suite
│   ├── tcu_nvs_creds/                — ThingsBoard credentials storage, validation, and repair
│   └── version/                      — Version string headers (FW_VERSION_STR, HW_VERSION_STR)
│
│   (Git Submodules)
│   ├── jk_bms/                       — JK-BMS Modbus register definitions & decoders
│   └── modbus_rtu/                   — Modbus RTU framing, checksums, and parser
├── FIRMWARE_VERSION_COMPARISON.md    — Detailed technical delta between base and V1.1.0
├── platformio.ini                    — PlatformIO project configuration
├── CMakeLists.txt                    — ESP-IDF root project configuration
├── sdkconfig.defaults                — Default configuration overrides
└── README.md                         — Master documentation (this file)
```

---

## 4. Operating Modes: RS-485 Modbus vs Shunt-Only Mode

The firmware supports two operational profiles in `components/bms_monitor_task/bms_monitor_task.c`:

### Mode A: Full RS-485 Modbus Multi-Battery Mode (Production)
In this mode, `bms_monitor_task` continuously switches the MC74HC4052ADG multiplexer across configured batteries (`BOARD_BMS_BATTERY_COUNT = 2` or `3`), querying serial numbers, cell voltages, SOC, SOH, alarms, and cycle counts.

### Mode B: Shunt-Only Mode (Bench Testing & Office Verification)
When working on the bench without physical JK-BMS lithium packs connected, the firmware can run in **Shunt-Only Mode**:
- RS-485 Modbus polling loops are bypassed to prevent continuous bus timeout delays.
- Pack current is sampled directly from the analog shunt sensor at the configured cycle rate.
- Staging records are populated with `BMS_STATUS_NO_DATA` sentinels, allowing MQTT telemetry, GPS tracking, accelerometer analysis, and ThingsBoard publishing to function unimpeded.

> **Toggling Modes:**  
> To switch between Shunt-Only Mode and Modbus Mode, inspect `components/bms_monitor_task/bms_monitor_task.c` and enable/disable the RS-485 read loops (`read_battery_pass1` / `read_battery_pass2`).

---

## 5. URGENT: BMS RS-485 Communication Latency — Investigation & Optimization

> **Status:** Active Field/Bench Investigation.  
> A full 13-parameter read of a single JK-BMS pack over RS-485 currently measures **~2030 ms**. On a 3-pack vehicle, a full cycle consumes >6 seconds.

### 5.1 Two-Pass Read Architecture

To prevent temporal skew between battery current and the analog shunt, reads are split into two phases:

```
bms_monitor_task (Core 0, Single Cycle)
│
├── PASS 1 — Serial Number & Battery Current (Time-Critical)
│     For each configured battery (B1, B2, ...):
│       1. MUX switch to channel → Bus settle (100 ms)
│       2. Read serial number (up to 5 retries @ 50 ms)
│       3. Read battery current (up to 2 retries @ 50 ms)
│       4. Sample physical shunt ADC immediately (~1 ms latency)
│
├── PASS 2 — Remaining 12 Telemetry Registers (Non-Time-Critical)
│     For each configured battery:
│       1. MUX switch to channel → Bus settle (100 ms)
│       2. Read 12 sequential Modbus registers:
│          [Voltage, SOC, SOH, Power, Capacity, Charge Stat,
│           Discharge Stat, Cycles, Alarms, Device ID, Cell Count, Cell Diff]
│
└── Atomically commit staging[] to s_cycle_snapshot[] under mutex
```

### 5.2 Latency Budget Breakdown per Battery

| Phase | Operation | Nominal Time | Worst Case (Retried) |
|---|---|---|---|
| **Pass 1** | MUX switch & settle | 100 ms | 100 ms |
| | Serial read | 50 ms | 250 ms (5 retries) |
| | Current read | 50 ms | 100 ms (2 retries) |
| | Shunt ADC snap | 0.64 ms | 0.64 ms |
| | **Pass 1 Total** | **~210 ms** | **~460 ms** |
| **Pass 2** | MUX switch & settle | 100 ms | 100 ms |
| | 12 Register reads (`READ_REG`) | 600 ms (12 × 50 ms) | 1440 ms (retries + flush penalties) |
| | **Pass 2 Total** | **~710 ms** | **~1550 ms** |
| **Combined Total** | **Pass 1 + Pass 2** | **~920 ms** | **~2010–2030 ms** |

### 5.3 Active Latency Optimization Levers

1. **Pass 2 Register Batching:** JK-BMS protocol investigation to determine if contiguous registers can be read in a single multi-register Modbus transaction (Function Code `0x03`), cutting framing overhead from 12 round-trips to 1–2.
2. **MUX Settle Delay Reduction:** Currently set to `100 ms` in both passes. Must be verified with an oscilloscope on the RS-485 transceiver output before reducing to 50 ms to avoid frame truncation from line capacitance.
3. **Decouple Slow-Changing Registers:** Read static fields (`cell_count`, `device_address`, `model_no`) once every $N$ cycles instead of every cycle.

---

## 6. Component Architecture & Data Flow

### 6.1 BMS Data Pipeline & Atomic Snapshot API

The legacy FreeRTOS queue (`bms_data_queue`) was completely removed to eliminate cross-cycle data mixing. Data now moves through an atomic snapshot buffer protected by a mutex:

```c
// Thread-safe atomic snapshot retrieval in Quectel_mqtt / mqtt_publish_task
bool bms_monitor_get_snapshot(bms_queued_data_t out[BMS_BATTERY_COUNT],
                              uint32_t         *cycle_id_inout,
                              float            *shunt_a_out,
                              float            *shunt_max_out);
```

- **Guarantees:** The publish task never receives mismatched battery packs from different cycles.
- **Staleness Tracking:** Monotonic `cycle_id` prevents duplicate re-publishing if a publish cycle triggers before the BMS task completes a new sweep.

### 6.2 Current Measurement & Multi-Point Calibration

- **Pass 1 Synchronized Snaps:** An analog shunt reading is taken within ~1 ms of each battery's digital current read. The three snaps produce:
  - `avg` (Field `"15"`): Best representation for vehicle state of charge accounting.
  - `max` (Field `"15b"`): Dynamic acceleration peak indicator.
  - `rms`: Windowed root-mean-square current.
- **Steinhart-Hart Temperature Modeling:** Converts NTC resistance to motor temperature using 3rd-order polynomial fitting, eliminating non-linear thermistor error across the -10 °C to +85 °C operating range.

### 6.3 IMU Accelerometer (LSM6DSOX / LIS3DHTR) & High-Speed FIFO Engine

```
LSM6DSOX 6-Axis IMU (208 Hz ODR, Stream Mode)
       │ (fills 32-sample hardware FIFO every 154 ms)
       ▼
accel_drain_task (Core 0, Priority 6) ──> Drains FIFO into software ring buffer
       │
       ├── 1 Hz Windowed RMS Calculation ──> diag_log_task & tcu_wifi_ap
       └── 60 s Window Vibration Stats ───> mqtt_publish_task (Fields "17"-"21")

Hardware INT1 (GPIO7) ──> Tap / Shock / Wake-up Detection (±16g, 5g thres) ──> Field "22"
```

The system features native drivers for the high-performance **LSM6DSOX** 6-axis IMU (`components/lsm6dsox`), with automatic WHO_AM_I detection (`0x6C`) and full backward-compatibility drop-in aliases for the legacy **LIS3DHTR** driver.

### 6.4 Quectel Cellular Modem, URC Queue & GPS Speed Cache

- **In-Line URC Sniffing:** `send_at_command()` inspects every incoming character line. If a `+QMTRECV:` or `+QMTSTAT:` URC appears during AT command execution, it is posted to `urc_notification_queue` rather than being dropped.
- **Dedicated URC Dispatcher:** `process_urcs_task` runs every 100 ms on Core 1 to handle incoming RPCs and cloud acknowledgments.
- **Asynchronous GPS Speed Cache:** `Quectel_gps` maintains `s_speed_cache` protected by a mutex. Polled safely by any task without locking the modem UART.

### 6.5 Power Management & Motion-Aware Trike Shutdown

- **Safety Interlock:** If ThingsBoard issues an RPC power-off command (`method: "P_OFF"` or `TRSW`), the TCU evaluates `check_trike_motion()`.
- **Motion Wait Loop:** If vehicle speed exceeds 5.0 km/h (or shunt current exceeds 2.0 A), power-off is held in pending status, triggering the `ALERT_TRIKE_STOPPING` LED pattern while `trike_wait_for_stop_task` polls for 0 km/h before opening the contactor relay.

### 6.6 RGB LED Alert Management

| State | Pattern | Condition |
|---|---|---|
| `ALERT_BOOT` | White (All ON) | System initializing and running factory self-test |
| `ALERT_NORMAL` | Green Steady | Trike ON, modem connected, normal operation |
| `ALERT_TRIKE_OFF` | Red Steady | Contactor relay open (trike powered down) |
| `ALERT_RPC_RECEIVED` | Blue Fast Blink (500 ms) | Cloud RPC command executed |
| `ALERT_LOW_SOC_WARN` | Red+Blue Slow Blink (2000 ms) | Average battery SOC < 20% |
| `ALERT_LOW_SOC_CRIT` | Red+Blue Medium Blink (1000 ms)| Average battery SOC < 10% |
| `ALERT_NO_BATTERIES` | Red Fast Blink (500 ms) | No JK-BMS packs responding |
| `ALERT_VOLTAGE_ANOMALY` | Red+Blue Fast Blink (500 ms) | Cell delta > 5000 mV between packs |
| `ALERT_TRIKE_STOPPING`| Blue Rapid Flash (Others OFF) | Power-off received; waiting for vehicle to halt |

### 6.7 Wi-Fi SoftAP & Live Telemetry Web Portal

```
Live Diagnostics / Field Inspection
       │
       ├── ESP32-S3 SoftAP: SSID "TCU_<MAC_HEX>", IP 192.168.4.1
       ├── Captive DNS (UDP port 53): Resolves all domains to 192.168.4.1
       ├── Embedded HTTP Server: Serves standalone HTML5/CSS/JS dashboard
       │
       ├── Live Data Polling (1 Hz):
       │     GET /api/live ──> Real-time JSON (GPS, Battery, Shunt, Temp, IMU, GSM)
       │
       └── Historical Telemetry Export:
             ├── GET /api/log/download      ──> Streams 1000-sample rolling CSV file
             ├── GET /api/snapshot/download ──> Downloads current JSON snapshot
             └── POST /api/log/clear        ──> Resets RAM circular buffer
```

---

## 7. Wi-Fi SoftAP & Field Web Portal Specification

The TCU incorporates an autonomous Wi-Fi Access Point and diagnostic web server (`components/tcu_wifi_ap`), enabling field technicians, service personnel, and developers to monitor real-time vehicle operations without physical debug cables or internet access.

### 7.1 Access Point Configuration
- **SSID:** `TCU_<DeviceSerial>` (e.g. `TCU_A4CF12B3E501`, unique 12-character hex MAC serial).
- **Default Password:** `12345678` (WPA2-PSK; set in `tcu_wifi_ap.h`).
- **Default IP:** `192.168.4.1` (Subnet `255.255.255.0`).
- **Max Client Connections:** 4 simultaneous devices.
- **Captive Portal Auto-Detect:** Built-in UDP 53 DNS server redirects standard OS captive check domains (`connectivitycheck.gstatic.com`, `apple.com`, `msftconnecttest.com`) to `http://192.168.4.1/`.

### 7.2 Embedded Web Dashboard Features
The portal is fully self-contained in MCU flash with no external CDN or internet dependencies:
- **Live Battery & Power Panel:** Pack Voltage ($V$), INA240 Shunt Current ($A$), Peak Shunt Current ($A$), Instantaneous Power ($W$), NTC Temperature ($^\circ\text{C}$), Average SOC ($0\text{--}100\%$) with animated gradient bar, and Trike Relay state (`ON` / `OFF`).
- **GPS & Navigation Panel:** Fix state (`3D FIX`, `2D FIX`, `NO FIX`), Speed over ground ($\text{km/h}$), Satellites count, Altitude ($m$), UTC time/date, Lat/Lon coordinates with 1-click clipboard copy and Google Maps link.
- **Motion & Dynamics Panel:** LSM6DSOX 1-second RMS vibration magnitude ($mg$), Accel $X, Y, Z$ axes, Motion status (`MOVING` / `STATIONARY`), and Impact Alarm indicator (`IMPACT DETECTED`).
- **Cellular & System Health:** GSM Signal quality (RSSI & $\text{dBm}$), ThingsBoard MQTT status, system uptime, and internal free heap memory.
- **Interactive Controls:** Stream Pause/Resume toggle, Manual Refresh, and live rolling telemetry table of the last 10 samples.

### 7.3 REST Endpoints & Data Download

| Method | Endpoint | Format | Description |
|---|---|---|---|
| `GET` | `/` or `/index.html` | HTML | Interactive telemetry dashboard web application |
| `GET` | `/api/live` | JSON | Current live telemetry snapshot (1 Hz updates) |
| `GET` | `/api/log/download` | CSV | Streams rolling circular buffer as `tcu_telemetry_log.csv` |
| `GET` | `/api/snapshot/download` | JSON | Downloads current state as `tcu_telemetry_snapshot.json` |
| `POST` | `/api/log/clear` | JSON | Clears in-memory rolling history buffer |

### 7.4 Rolling In-Memory CSV Log Buffer
- **Capacity:** 1,000 continuous records stored in internal RAM (~64 KB buffer footprint).
- **Duration:** Provides ~16.6 minutes of continuous $1\,\text{Hz}$ telemetry recording.
- **CSV Headers:** `timestamp_ms, uptime_s, voltage_v, current_a, current_max_a, power_w, temp_c, soc_pct, trike_on, motion, impact, lat, lon, altitude_m, speed_kmh, sats, gps_valid, accel_x_mg, accel_y_mg, accel_z_mg, accel_mag_mg, gsm_rssi, gsm_dbm`.

---

## 8. FreeRTOS Tasks & Core Allocation

| Task Name | Core | Priority | Stack (Words) | Interval / Trigger | Purpose |
|---|---|---|---|---|---|
| `accel_drain` | 0 | 6 | 3072 | 160 ms periodic | High-priority IMU FIFO drainage (Must meet 160 ms deadline) |
| `bms_monitor` | 0 | 5 | 4096 | Continuous (~2–6 s)| Modbus BMS sweep or shunt-only sampling; feeds WDT |
| `bms_led` | 0 | 4 | 2048 | 50 ms tick | Non-blocking RGB LED state pattern engine |
| `tcu_dns` | 0 | 3 | 4096 | Event-driven (UDP 53)| Captive DNS query handler for SoftAP clients |
| `diag_log` | 0 | 2 | 5120 | 1 Hz periodic | Lab diagnostic sensor logger and live Wi-Fi telemetry feeder |
| `power_monitor`| 1 | 6 | 4096 | 500 ms / Event | Contactor state monitoring and power rail management |
| `mqtt_publish` | 1 | 5 | 6144 | 60 s / Event | Consumes BMS snapshot, builds JSON, publishes to MQTT; feeds WDT |
| `process_urcs` | 1 | 5 | 6144 | 100 ms queue poll | Drains intercepted modem URC notifications |
| `stats_monitor`| 1 | 3 | 3072 | 30 s periodic | Heap high-water mark, stack usage, and system health logging |
| `trike_wait` | 1 | 4 | 3072 | On-demand (5 s poll)| Spawned during power-off while trike is in motion |
| `httpd` | Any | 5 | 6144 | TCP Port 80 | ESP-IDF HTTP server serving dashboard & streaming CSV/JSON |

---

## 9. ThingsBoard Telemetry & Attributes Specification

### 9.1 Per-Battery Telemetry (Published $N$ times per cycle for each pack)

| Key | Unit | Description | Source |
|---|---|---|---|
| `Bx_P1` | V | Pack Voltage | `batt_v / 1000.0` |
| `Bx_P2` | A | Pack Current (Pass 1 aligned) | `batt_i / 1000.0` |
| `Bx_P3` | % | State of Charge (SOC) | `soc` |
| `Bx_P4` | % | State of Health (SOH) | `soh` |
| `Bx_P5` | cycles | Lifetime Charge Cycles | `charge_cycles` |
| `Bx_P6` | 0/1 | Communication Status | `0 = OK, 1 = NO_DATA` |
| `Bx_P7` | — | Charge MOS Status | `charge_stat` |
| `Bx_P8` | — | Discharge MOS Status | `discharge_stat` |
| `Bx_P9` | — | Battery Serial Number | `serial_no` |
| `Bx_P10` | mV | Max Cell Voltage Delta | `cells_diff` |
| `Bx_P11` | bitmask | BMS Hardware Alarm Mask | `alarms` |
| `Bx_P12` | count | Total Connected Cells | `cell_count` |
| `Bx_P13` | Ah | Remaining Capacity | `remaining_capacity / 10000.0` |
| `Bx_P14` | 0/1 | Cross-Pack Voltage Anomaly | `1` if pack differs > 5.0 V from siblings |

### 9.2 System Metadata & Telemetry (Published once per cycle)

| Key | Unit | Description | Key | Unit | Description |
|---|---|---|---|---|---|
| `"1"` | — | Device MAC / Serial | `"14"` | V | Live Analog Pack Voltage |
| `"2"` | count | Valid Battery Count | `"15"` | A | Pass-1 Synchronized Shunt Average |
| `"3"` | dBm | GSM Signal Strength | `"15b"` | A | Pass-1 Synchronized Shunt Peak Max |
| `"4"` | deg | GPS Latitude | `"16"` | °C | Motor Temperature (Steinhart-Hart) |
| `"5"` | deg | GPS Longitude | `"17"`–`"19"`| mg | Accelerometer RMS X / Y / Z |
| `"6"` | m | GPS Altitude | `"20"` | mg | Accelerometer 3D Vector Magnitude |
| `"7"` | count | GPS Visible Satellites | `"21"` | mg | Accelerometer Peak Dynamic Acceleration |
| `"8"` | km/h | GPS Speed Over Ground | `"22"` | 0/1 | Hardware Impact / Tap Detected |
| `"9"` | fix | GPS Fix Quality (0/1/2/3) | `"23"` | 0/1 | Trike in Motion Flag |
| `"10"` | bool | Response Command Acknowledge | `P_offset_precfg`| A | Pre-configured Offset Constant |
| `"11"` | bool | Trike Contactor State (1=ON) | `P_offset_board` | A | Active Board Zero-Current Offset |
| `"12"` | % | Mean Traction Battery SOC | `P_bms_cycle_ms` | ms | Total BMS Cycle Duration |
| `"13"` | — | SOC Sentinel | `fw` / `hw` | str | Firmware & Hardware Versions |

---

## 10. NVS Namespaces & Storage

| Namespace | Partition | Key | Type | Description |
|---|---|---|---|---|
| `tcu_creds` | `nvs` | `prov_done` | `u8` | `1` if device has completed ThingsBoard provisioning |
| | | `username` | `str` | MQTT username (derived from `HUB_NAME`) |
| | | `password` | `str` | MQTT password (`HUB_NAME + "_ksc"`) |
| | | `client_id`| `str` | MQTT client ID (device MAC hex) |
| `trike_sens`| `nvs` | `cur_offset`| `i32` | Calibrated zero-current offset in integer mA |
| | | `temp_offset`| `i32` | Calibrated temperature offset in m°C |
| `trike_ctrl`| `nvs` | `pwr_conf` | `u8` | Contactor state (restored across reboots) |
| `jkbms_ser` | `nvs` | `bms0_ser` | `str` | Cached serial number for Battery 1 |
| | | `bms1_ser` | `str` | Cached serial number for Battery 2 |
| | | `bms2_ser` | `str` | Cached serial number for Battery 3 |

---

## 11. MQTT Auto-Provisioning & Credential Lifecycle

```
Power On / Boot
       │
       ▼
tcu_load_or_provision_creds()
       ├── [prov_done == 1] ──> Load credentials from NVS ──> Connect to ThingsBoard
       │                              │
       │                              └── (Auth Code 5: Auth Rejected) ──> Force Reprovision
       ▼
Provisioning Workflow:
  1. Connect to ThingsBoard with clientId = MAC, username = "provision"
  2. Subscribe to /provision/response
  3. Publish request payload to /provision/request containing HUB_NAME and device secrets
  4. Intercept +QMTRECV URC notification containing access credentials
  5. Commit credentials to NVS under `tcu_creds` namespace, set prov_done = 1
  6. Reboot MCU via esp_restart()
```

---

## 12. Factory Boot Self-Test Suite

Every cold boot executes `run_factory_tests()` in `components/factory_test/factory_test.c`:
1. **LSM6DSOX / LIS3DHTR Test:** Verifies WHO_AM_I register (`0x6C` / `0x33`) over I2C0.
2. **Modem AT Test:** Validates communication (`AT`), echoes off (`ATE0`), checks IMEI (`AT+GSN`), CSQ signal quality (`AT+CSQ`), SIM presence (`AT+CPIN?`), and network registration (`AT+CREG?`).
3. **RS-485 MUX Test:** Toggles MUX pins GPIO8/9 and verifies bus line bias.
4. **RGB LED Test:** Sequences Red, Green, and Blue channels for visual inspection.
5. **ADC Rail Test:** Reads pack voltage, shunt midpoint, and motor temperature.

---

## 13. Build, Flash & Monitor

### 13.1 Building with PlatformIO (Recommended)

The project includes an optimized `platformio.ini` configured with 2MB flash and ESP32 exception decoding:

```bash
# Build firmware
platformio run

# Flash to device (Linux)
platformio run --target upload --upload-port /dev/ttyACM0

# Launch serial monitor with exception decoding
platformio run --target monitor
```

### 13.2 Building with Native ESP-IDF v5.4.1

```bash
# Setup environment
. $IDF_PATH/export.sh

# Target configuration (ESP32-S3)
idf.py set-target esp32s3

# Build
idf.py build

# Flash and open serial monitor
idf.py -p /dev/ttyACM0 flash monitor
```

---

## 14. Version Comparison & Technical Delta

For a complete line-by-line comparative audit of all changes between upstream base firmware (`marymbaire/KSC-TCU-Firmware`) and this V1.1.0 production release, refer to the dedicated companion document:

👉 **[FIRMWARE_VERSION_COMPARISON.md](file:///home/goaty/Desktop/Rift/KSC_TCU_Firmware-main/FIRMWARE_VERSION_COMPARISON.md)**

---

## 15. Review Focus Areas & Production Checklist

1. [ ] **Verify Diagnostic Logging / Wi-Fi Feeder:** In `main/main.c`, `diag_log_task` prints verbose logs at 1 Hz and feeds telemetry to the `tcu_wifi_ap` portal. For quiet serial production, the serial print statements may be muted while keeping the portal update active.
2. [ ] **Verify Active Current Offset:** Confirm that `BOARD_CURRENT_ZERO_OFFSET_A` in `board_config.h` matches the specific PCB revision being flashed (Board 5 = 5.23 A, Board 8 = 2.36 A, Generic = 4.87 A).
3. [ ] **Confirm Accelerometer Mode:** Verify that `lsm6dsox_configure_mode(LSM6DSOX_MODE_PRODUCTION)` is executed after factory test completion to engage the high-pass filter.
4. [ ] **Set Production HUB Name:** Ensure `HUB_NAME` in `main.c` is configured with the correct production device identifier (`TCU_DEVICE_x` from `tcu_nvs_creds.h`) prior to first cloud provisioning.
5. [ ] **Operating Mode Verification:** When installing on a trike equipped with physical JK-BMS packs, ensure `bms_monitor_task.c` is operating in **Full Modbus Mode** rather than **Shunt-Only Mode**.
6. [ ] **Wi-Fi SoftAP Access Security:** Review default Wi-Fi password in `tcu_wifi_ap.h` (`TCU_WIFI_AP_DEFAULT_PASSWORD`) if field security policies require a per-fleet passphrase.