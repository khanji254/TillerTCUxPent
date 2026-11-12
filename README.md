# KSC TCU V2.1 — Tiller Control Unit Firmware

**Platform:** ESP32-S3 · **Framework:** ESP-IDF v5.4.1 / PlatformIO · **FW Version:** 1.0.0 (Wan) · **HW Revision:** 2.1 (2.1.0)  
**ThingsBoard:** `thingsboard.iot.songa.mobi` (MQTTS Port 8883 / TLS) · **APN:** `safaricom` / `safaricomiot`  
**Target Vehicle:** Kilikawi Tiller (Electric Multi-Purpose Agricultural Tool)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware Platform & Pinout](#2-hardware-platform--pinout)
3. [Repository Structure](#3-repository-structure)
4. [Operating Modes: Shunt-Only Mode vs RS-485 Modbus](#4-operating-modes-shunt-only-mode-vs-rs-485-modbus)
5. [BMS RS-485 Communication Latency & Architecture](#5-bms-rs-485-communication-latency--architecture)
6. [Component Architecture & Data Flow](#6-component-architecture--data-flow)
   - [6.1 BMS Data Pipeline & Atomic Snapshot API](#61-bms-data-pipeline--atomic-snapshot-api)
   - [6.2 Current Measurement, Polarity Correction & Calibration](#62-current-measurement-polarity-correction--calibration)
   - [6.3 Load Cell Agricultural Sensing Interface](#63-load-cell-agricultural-sensing-interface)
   - [6.4 IMU Accelerometer (LSM6DSOX / LIS3DHTR) & High-Speed FIFO Engine](#64-imu-accelerometer-lsm6dsox--lis3dhtr--high-speed-fifo-engine)
   - [6.5 Quectel Cellular Modem, In-Line URC Queue & GPS Speed Cache](#65-quectel-cellular-modem-in-line-urc-queue--gps-speed-cache)
   - [6.6 Power Management & Motion-Aware Tiller Shutdown](#66-power-management--motion-aware-tiller-shutdown)
   - [6.7 Status LED Alert Management](#67-status-led-alert-management)
   - [6.8 Wi-Fi SoftAP & Live Telemetry Web Portal](#68-wi-fi-softap--live-telemetry-web-portal)
7. [Wi-Fi SoftAP & Field Web Portal Specification](#7-wi-fi-softap--field-web-portal-specification)
8. [FreeRTOS Tasks & Core Allocation](#8-freertos-tasks--core-allocation)
9. [ThingsBoard Telemetry & Attributes Specification](#9-thingsboard-telemetry--attributes-specification)
10. [NVS Namespaces & Storage](#10-nvs-namespaces--storage)
11. [MQTT Auto-Provisioning & Credential Lifecycle](#11-mqtt-auto-provisioning--credential-lifecycle)
12. [Factory Boot Self-Test Suite](#12-factory-boot-self-test-suite)
13. [Build, Flash & Monitor](#13-build-flash--monitor)
14. [Production Verification Checklist](#14-production-verification-checklist)

---

## 1. Overview

The **KSC TCU (Tiller Control Unit) V2.1** is a cellular IoT telematics gateway and machine supervisor engineered for the **Kilikawi Tiller** — an electric multi-purpose agricultural tractor/tiller tool. Powered by a dual-core ESP32-S3 microcontroller, the TCU interfaces with the tiller's traction battery system, analog shunt current sensors, agricultural draft load cells, motor temperature, 6-axis inertial motion unit, GNSS positioning receiver, LTE cellular modem, and cloud infrastructure.

### Core Capabilities:
- **Dual-Pack Battery Management:** Interfaces with up to 2 × JK-BMS lithium battery packs (`BOARD_BMS_BATTERY_COUNT = 2`, extensible to 3) over RS-485 Modbus RTU using a 2-channel analog multiplexer (MC74HC4052ADG).
- **Bench & Field Shunt-Only Mode:** Dedicated operational profile that bypasses RS-485 polling and derives vehicle current directly from the analog shunt sensor, populating sentinels to keep GPS tracking, IMU analytics, and MQTT telemetry active.
- **Precision Bidirectional Current Sensing:** Shunt current measurement utilizing a Texas Instruments **INA241A1IDDFR** current sense amplifier ($10.0\ \text{V/V}$ gain) across a $500\ \mu\Omega$ shunt bar, with empirical baseline characterization ($1452.0\ \text{mV}$ zero-current reference) and corrected discharge polarity.
- **Agricultural Load Cell Sensing:** Integrated differential ADC interfaces for dual agricultural load cells (Load Cell A & B) with dedicated $1.65\ \text{V}$ mid-rail bias references for draft load and implement force measurement.
- **Dynamic Terrain & Vibration Analytics:** High-frequency $208\ \text{Hz}$ LSM6DSOX 6-axis IMU stream processing (with LIS3DHTR backward-compatibility layer) using a 32-sample hardware FIFO to detect soil roughness, RMS vibration, and collision impacts.
- **Continuous NMEA GNSS Tracking:** Asynchronous NMEA sentence streaming over UART0 with thread-safe GPS speed and position caching.
- **Motion-Guarded Power & Kill Switch Actuation:** Tiller high-voltage kill switch and contactor control (GPIO21) that enforces a vehicle halt interlock before de-asserting machine power.
- **Autonomous Wi-Fi SoftAP & Diagnostic Portal:** Embedded Wi-Fi Access Point (`KTCU_<id>` or `TCU_<serial>`) with captive DNS and a standalone web dashboard for live field monitoring and 1-click CSV/JSON telemetry download.
- **Zero-Touch Provisioning & Dynamic Fleet Naming:** Secure MQTT Basic auto-provisioning against ThingsBoard with 32-character cryptographically random tokens, dynamic NVS fleet naming (e.g. `KTCU_0000`, `KTCU_0001`...), zero hardcoded secrets, and autonomous NVS credential caching and repair.

### TCU Hardware Evolution (V1.0 → Gen 2.1)

| Subsystem / Feature | Legacy TCU V1.0 (ESP32) | TCU Gen 2.1 (ESP32-S3 Tiller) |
|---|---|---|
| **Microcontroller** | ESP32-D0WDQ6 (Xtensa LX6) | **ESP32-S3** (Dual-core Xtensa LX7 @ 240 MHz) |
| **BMS Battery Count** | 3 packs | **2 packs** (`BOARD_BMS_BATTERY_COUNT = 2`) |
| **RS-485 Transceiver Routing** | 3× Discrete GPIO DE/RE lines | **MC74HC4052ADG MUX** (`SEL_A`=GPIO39, `SEL_B`=GPIO40) |
| **Current Sense Amplifier** | Discrete op-amp shunt circuit | **INA241A1IDDFR** (Gain = $10.0\ \text{V/V}$, GPIO5 ADC1_CH4) |
| **Current VREF Bias** | Theoretical 1650 mV | **Characterized baseline 1452 mV** (`BOARD_CURRENT_VREF_MV`, GPIO4 ADC1_CH3) |
| **Shunt Resistor** | Uncalibrated discrete shunt | **$500\ \mu\Omega$ ($0.000500\ \Omega$)** precision manganese-copper bar |
| **Pack Voltage Sense** | GPIO4 (ADC1_CH3) | **GPIO14 (ADC2_CH3)** via ÷41 resistive divider ($200\ \text{k}\Omega / 5\ \text{k}\Omega$) |
| **Load Cell Interfaces** | None | **Dual Load Cell inputs** (A: GPIO8/9, B: GPIO16/15) |
| **Inertial Measurement (IMU)** | LIS3DHTR | **LSM6DSOX 6-axis IMU** (I2C0, INT1 on GPIO7, 208 Hz FIFO) |
| **Status LEDs** | Single monochrome LED | **4 Status LEDs**: Red=GPIO1, Green=GPIO37, Blue=GPIO36, Aux=GPIO35 |
| **Tiller Power / Kill Switch** | GPIO27 | **GPIO21** (Net `Kill_switch`, active-high gate drive) |
| **Modem Control Pins** | RI=GPIO15, DTR=GPIO16 | **RI=GPIO41, DTR=GPIO42**; PWR_KEY=GPIO11, RST_N=GPIO10 |
| **GPS Interface** | Periodic `AT+QGPSLOC` polling | **Continuous NMEA stream on UART0** (GPIO43/44) with speed cache |
| **Field Diagnostics** | Serial monitor only | **Wi-Fi SoftAP (`192.168.4.1`) & Live Web Dashboard** |
| **Cloud Provisioning** | Hardcoded compile-time tokens | **Dynamic NVS-backed auto-provisioning** (Kconfig/NVS secrets, RNG per-device passwords, sequential `KTCU_xxxx` naming) |

---

## 2. Hardware Platform & Pinout

```
ESP32-S3 (WROOM-1 / DevKitC-1, Gen 2.1 Board)
├── UART0  (GPIO43 TX / GPIO44 RX)  — Quectel debug UART: Continuous NMEA GPS stream @ 115200 baud
├── UART1  (GPIO12 TX / GPIO13 RX)  — RS-485 half-duplex via MC74HC4052ADG MUX @ 115200 baud
├── UART2  (GPIO17 TX / GPIO18 RX)  — Quectel main AT + MQTT command port @ 115200 baud
├── I2C0   (GPIO47 SCL / GPIO48 SDA) — LSM6DSOX 6-axis IMU (Addr 0x6A/0x6B, 100 kHz, 4.7 kΩ pull-ups)
│
├── ADC1   (Analog-to-Digital Converter Unit 1)
│   ├── CH3 (GPIO4)  — INA241 mid-rail bias reference (CURRENT_1.65V)
│   ├── CH4 (GPIO5)  — Shunt current sense output (INA241A1IDDFR, Vref = 1452 mV)
│   ├── CH5 (GPIO6)  — Motor / tiller NTC temperature (10 kΩ, Steinhart-Hart B=3380 K)
│   ├── CH7 (GPIO8)  — LOAD_CELL_A (via JP18)
│   └── CH8 (GPIO9)  — LOAD_CELL_A_1.65V reference
│
├── ADC2   (Analog-to-Digital Converter Unit 2)
│   ├── CH3 (GPIO14) — Traction pack voltage (÷41 resistive divider, 56–84 V range)
│   ├── CH4 (GPIO15) — LOAD_CELL_B_1.65V reference
│   └── CH5 (GPIO16) — LOAD_CELL_B
│
├── Digital & Control I/O
│   ├── GPIO1   — Status LED Red (Power indicator; fault blink)
│   ├── GPIO37  — Status LED Green (Heartbeat; active in normal operation)
│   ├── GPIO36  — Status LED Blue (MQTT / RPC activity indicator)
│   ├── GPIO35  — Status LED Aux (General purpose LED via R70 2.2 kΩ)
│   ├── GPIO7   — IMU INT1 (hardware tap / shock / motion collision interrupt)
│   ├── GPIO21  — Tiller power / kill switch control (active-high P-MOSFET gate)
│   ├── GPIO39  — RS-485 MUX Select A (`SEL_A`)
│   ├── GPIO40  — RS-485 MUX Select B (`SEL_B`)
│   ├── GPIO10  — Quectel Modem Reset (`RST_N`, active low)
│   ├── GPIO11  — Quectel Power Key (`PWR_KEY`, active high pulse)
│   ├── GPIO41  — Quectel Ring Indicator (`RI`, input)
│   ├── GPIO42  — Quectel Data Terminal Ready (`DTR`, output)
│   ├── GPIO38  — User Terminal J10 Pin 3 (TVS protected)
│   └── GPIO2   — User Terminal J10 Pin 4 (TVS protected)
│
└── Wi-Fi  (Internal 2.4 GHz RF)     — SoftAP (192.168.4.1), Captive DNS (UDP 53) & Web Portal
```

### Current Measurement Circuit Parameters:
- **Shunt Resistor:** Precision manganese-copper shunt bar $\rightarrow R = 0.000500\ \Omega$ ($500\ \mu\Omega$, 150 A / 75 mV rating).
- **Current Amplifier:** Texas Instruments **INA241A1IDDFR** (SOT-23-8) with fixed gain of $10.0\ \text{V/V}$.
- **Output Low-Pass Filter:** $R83 = 1\ \text{k}\Omega$, $C84 = 1.5\ \mu\text{F}$, cutoff frequency $f_c \approx 106\ \text{Hz}$.
- **Zero-Current Reference Voltage ($V_{\text{ref}}$):** Nominal $1650.0\ \text{mV}$, characterized baseline $1452.0\ \text{mV}$ (`BOARD_CURRENT_VREF_MV`).
- **Transfer Function (Discharge Positive):**
  $$\Delta V_{\text{mV}} = V_{\text{ref, mV}} - V_{\text{ADC, mV}}$$
  $$I_{\text{amps}} = \frac{\Delta V_{\text{mV}}}{\text{Gain} \times R_{\text{shunt}} \times 1000} + I_{\text{zero\_offset}}$$
  *Note: Discharge current causes $V_{\text{ADC}} < V_{\text{ref}}$, producing positive amperage.*
- **Zero Offset Default:** `BOARD_CURRENT_ZERO_OFFSET_A = 0.0f` (dynamically updated and stored in NVS under `trike_sens`).
- **Oversampling:** 64-sample hardware average per conversion (~0.64 ms conversion time, non-blocking).

---

## 3. Repository Structure

```
KSC_TCU_Firmware-tiller/
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild             — Project build configuration options
│   └── main.c                        — System init, task spawning, WDT, publish loop
├── components/
│   ├── board_config/                 — Master hardware pinouts, ADC constants, and version.h
│   ├── bms_monitor_task/             — BMS cycling engine, atomic snapshot mutex, shunt-only telemetry
│   ├── bms_monitor_types/            — Shared structs (bms_queued_data_t, bms_system_stats_t)
│   ├── bms_rs485_mux/                — MC74HC4052ADG 2-channel RS-485 multiplexer driver
│   ├── jkbms_serial_storage/         — NVS-backed serial number cache for degraded mode
│   ├── trike_sensors/                — ADC voltage, current (INA241), and Steinhart-Hart temperature engine
│   ├── lsm6dsox/                     — LSM6DSOX 6-axis IMU driver, 208 Hz FIFO stream, RMS vibration
│   ├── lis3dhtr/                     — LIS3DHTR legacy compatibility wrapper
│   ├── power_trike_ctrl/             — Motion-aware contactor/kill switch sequencing and modem supervisor
│   ├── rgb_led/                      — Non-blocking state-machine 4-LED status indicator
│   ├── Quectel_mqtt/                 — AT command engine, in-line URC sniffer, auto-provisioning
│   ├── Quectel_gps/                  — NMEA parser, asynchronous speed and position cache
│   ├── tcu_wifi_ap/                  — Wi-Fi SoftAP, captive DNS server, live web dashboard & CSV export
│   ├── factory_test/                 — Production line boot self-test suite
│   ├── tcu_nvs_creds/                — ThingsBoard credentials storage, validation, and repair
│   │
│   │   (Git Submodules)
│   ├── jk_bms/                       — JK-BMS Modbus register definitions & decoders
│   └── modbus_rtu/                   — Modbus RTU framing, checksums, and parser
├── CURRENT_SENSOR_CALIBRATION_REPORT.md — Empirical Fluke 374FC calibration and polarity analysis
├── FIRMWARE_VERSION_COMPARISON.md    — Detailed technical delta between base and V1.0.0 (Wan)
├── platformio.ini                    — PlatformIO project configuration
├── CMakeLists.txt                    — ESP-IDF root project configuration
├── sdkconfig.defaults                — Default configuration overrides
└── README.md                         — Master documentation (this file)
```

---

## 4. Operating Modes: Shunt-Only Mode vs RS-485 Modbus

The firmware supports two primary operating profiles in `components/bms_monitor_task/bms_monitor_task.c`:

### Mode A: Shunt-Only Mode (Active Tiller Profile)
In this mode, RS-485 Modbus polling loops are completely bypassed to eliminate bus timeout delays when physical JK-BMS packs are disconnected or during standalone machine operation:
- Pack current is sampled directly from the analog shunt sensor at the BMS read interval (`BMS_READ_INTERVAL_MS = 2000 ms`).
- Staging records are populated with `BMS_STATUS_NO_DATA` sentinels for the configured battery slots (`BOARD_BMS_BATTERY_COUNT = 2`).
- Battery current field `batt_i` is directly populated with `(int32_t)(snap * 1000.0f)`.
- Motion detection fuses GPS speed cache with IMU motion and analog shunt current magnitude ($|I| > 2.0\ \text{A}$).
- Allows MQTT telemetry, GPS tracking, accelerometer terrain analysis, Wi-Fi SoftAP web portal, and ThingsBoard publishing to function unimpeded.

### Mode B: Full RS-485 Modbus Multi-Battery Mode
When physical JK-BMS lithium packs are connected to the RS-485 bus:
- `bms_monitor_task` switches the MC74HC4052ADG multiplexer across configured batteries (`BOARD_BMS_BATTERY_COUNT = 2`), querying serial numbers, cell voltages, SOC, SOH, alarms, and cycle counts.
- Uses the two-pass read architecture to prevent temporal skew between battery current and the analog shunt sensor.

---

## 5. BMS RS-485 Communication Latency & Architecture

### 5.1 Two-Pass Read Architecture
When RS-485 Modbus communication is active, reads are split into two phases to synchronize physical shunt current with digital BMS readings:

```
bms_monitor_task (Core 0, Single Cycle)
│
├── PASS 1 — Serial Number & Battery Current (Time-Critical)
│     For each configured battery (B1, B2):
│       1. MUX switch to channel (GPIO39/40) → Bus settle (100 ms)
│       2. Read serial number (up to 5 retries @ 50 ms)
│       3. Read battery current (up to 2 retries @ 50 ms)
│       4. Sample physical shunt ADC immediately (~0.64 ms latency)
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
| | Shunt ADC oversampled snap | 0.64 ms | 0.64 ms |
| | **Pass 1 Total** | **~210 ms** | **~460 ms** |
| **Pass 2** | MUX switch & settle | 100 ms | 100 ms |
| | 12 Register reads (`READ_REG`) | 600 ms (12 × 50 ms) | 1440 ms (retries + flush penalties) |
| | **Pass 2 Total** | **~710 ms** | **~1550 ms** |
| **Combined Total** | **Pass 1 + Pass 2** | **~920 ms** | **~2010–2030 ms** |

---

## 6. Component Architecture & Data Flow

### 6.1 BMS Data Pipeline & Atomic Snapshot API

The legacy FreeRTOS queue (`bms_data_queue`) was replaced with an atomic snapshot buffer protected by a mutex:

```c
bool bms_monitor_get_snapshot(bms_queued_data_t out[BMS_BATTERY_COUNT],
                              uint32_t         *cycle_id_inout,
                              float            *shunt_a_out,
                              float            *shunt_max_out);
```

- **Guarantees:** Eliminates cross-cycle data mixing. The publish task never receives mismatched battery pack records.
- **Deduplication:** Monotonic `cycle_id` prevents duplicate re-publishing if a publish cycle triggers before a new BMS cycle completes.

### 6.2 Current Measurement, Polarity Correction & Calibration

Empirical characterization with a calibrated **Fluke 374FC** True-RMS clamp multimeter revealed two key hardware characteristics:
1. **Polarity Correction:** In the physical circuit, battery discharge current drops the INA241 output below the mid-rail reference ($V_{\text{ADC}} < V_{\text{ref}}$). The firmware calculates:
   $$\Delta V = V_{\text{ref}} - V_{\text{ADC}}$$
   Ensuring positive current during discharge and negative current during regenerative events.
2. **Characterized Reference Baseline:** On this hardware revision, the resting zero-current ADC reference voltage sits at $1452.0\ \text{mV}$ (`BOARD_CURRENT_VREF_MV`), eliminating artificial offset hacks.
3. **Synchronized Snaps:** Three values are derived each cycle:
   - `avg` (Field `"15"`): Real-time vehicle load average.
   - `max` (Field `"15b"`): Dynamic acceleration peak indicator.
   - `rms`: Windowed root-mean-square current.

> For full calibration regression curves, timing alignment, and empirical test logs, see [CURRENT_SENSOR_CALIBRATION_REPORT.md](file:///home/goaty/Desktop/Rift/KSC_TCU_Firmware-tiller/CURRENT_SENSOR_CALIBRATION_REPORT.md).

### 6.3 Load Cell Agricultural Sensing Interface

The TCU Gen 2.1 hardware incorporates dedicated differential inputs for dual load cells on agricultural implements:
- **Load Cell A:** Measurement on GPIO8 (`ADC1_CH7`), $1.65\ \text{V}$ reference on GPIO9 (`ADC1_CH8`).
- **Load Cell B:** Measurement on GPIO16 (`ADC2_CH5`), $1.65\ \text{V}$ reference on GPIO15 (`ADC2_CH4`).
- Enables real-time measurement of draft force, implement strain, and soil resistance during tilling operations.

### 6.4 IMU Accelerometer (LSM6DSOX / LIS3DHTR) & High-Speed FIFO Engine

```
LSM6DSOX 6-Axis IMU (208 Hz ODR, Stream Mode)
       │ (fills 32-sample hardware FIFO every 154 ms)
       ▼
accel_drain_task (Core 0, Priority 6) ──> Drains FIFO into software ring buffer
       │
       ├── 1 Hz Windowed RMS Calculation ──> diag_log_task & tcu_wifi_ap
       └── 60 s Window Vibration Stats ───> mqtt_publish_task (Fields "17"-"21")

Hardware INT1 (GPIO7) ──> Tap / Shock / Collision Interrupt ──> Field "22"
```

The system features native drivers for the **LSM6DSOX** 6-axis IMU (`components/lsm6dsox`), with automatic WHO_AM_I detection (`0x6C`) and backward-compatibility drop-in aliases for legacy LIS3DHTR configurations.

### 6.5 Quectel Cellular Modem, In-Line URC Queue & GPS Speed Cache

- **In-Line URC Sniffing:** `send_at_command()` inspects every incoming character stream. If a `+QMTRECV:` or `+QMTSTAT:` URC appears during AT command execution, it is posted to `urc_notification_queue` rather than being dropped.
- **Dedicated URC Dispatcher:** `process_urcs_task` runs every 100 ms on Core 1 to handle incoming RPCs and cloud acknowledgments.
- **Asynchronous GPS Speed Cache:** `Quectel_gps` maintains `s_speed_cache` protected by a mutex, polled safely by any task without locking the modem UART.

### 6.6 Power Management & Motion-Aware Tiller Shutdown

- **Kill Switch & Contactor Gate:** Driven via GPIO21 (`BOARD_TRIKE_PWR_PIN`).
- **Safety Interlock:** If ThingsBoard issues an RPC power-off command (`method: "P_OFF"` or `TRSW`), the TCU evaluates `check_trike_motion()`.
- **Motion Wait Loop:** If vehicle speed exceeds 1.5 km/h, traction shunt current exceeds 2.0 A, or IMU motion/vibration is active, power-off is held in pending status, triggering the `ALERT_TRIKE_STOPPING` LED pattern while `trike_wait_for_stop_task` polls for complete stillness (debounced over 3 seconds) before de-asserting the kill switch.

### 6.7 Status LED Alert Management

The Gen 2.1 hardware provides four distinct indicator LEDs:
- **Red (GPIO1):** Power / fault indicator.
- **Green (GPIO37):** Normal operation heartbeat.
- **Blue (GPIO36):** MQTT and RPC activity.
- **Aux (GPIO35):** General purpose.

| Alert State | Visual Pattern | Operating Condition |
|---|---|---|
| `ALERT_BOOT` | Red ON, Green OFF, Blue OFF | System initializing and running boot self-test |
| `ALERT_NORMAL` | Green Steady Heartbeat | Tiller active, modem connected, normal operation |
| `ALERT_TRIKE_OFF` | Red Steady | Contactor open / kill switch de-asserted |
| `ALERT_RPC_RECEIVED` | Blue Fast Blink (500 ms) | Cloud RPC command executed |
| `ALERT_LOW_SOC_WARN` | Red + Blue Slow Blink (2000 ms) | Battery SOC < 20% |
| `ALERT_LOW_SOC_CRIT` | Red + Blue Medium Blink (1000 ms)| Battery SOC < 10% |
| `ALERT_NO_BATTERIES` | Red Fast Blink (500 ms) | No JK-BMS packs responding |
| `ALERT_VOLTAGE_ANOMALY` | Red + Blue Fast Blink (500 ms) | Pack voltage delta > 5000 mV |
| `ALERT_TRIKE_STOPPING`| Blue Rapid Flash | Power-off received; waiting for vehicle to halt |

### 6.8 Wi-Fi SoftAP & Live Telemetry Web Portal

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

The TCU incorporates an autonomous Wi-Fi Access Point and diagnostic web server (`components/tcu_wifi_ap`), enabling field technicians and operators to monitor real-time machine operations without physical debug cables or cellular reception.

### 7.1 Access Point Configuration
- **SSID:** `TCU_<DeviceSerial>` (e.g. `TCU_9C139E1AF418`, unique 12-character hex MAC serial).
- **Default Password:** `12345678` (WPA2-PSK; configurable in `tcu_wifi_ap.h`).
- **Default IP:** `192.168.4.1` (Subnet `255.255.255.0`).
- **Max Client Connections:** 4 simultaneous devices.
- **Captive Portal Auto-Detect:** Built-in UDP 53 DNS server redirects standard OS captive check domains (`connectivitycheck.gstatic.com`, `apple.com`, `msftconnecttest.com`) to `http://192.168.4.1/`.

### 7.2 Embedded Web Dashboard Features
The portal is self-contained in MCU flash with no external CDN or internet dependencies:
- **Live Battery & Power Panel:** Pack Voltage ($V$), INA241 Shunt Current ($A$), Peak Current ($A$), Instantaneous Power ($W$), NTC Temperature ($^\circ\text{C}$), Average SOC ($0\text{--}100\%$), and Tiller Kill Switch state (`ON` / `OFF`).
- **GPS & Navigation Panel:** Fix state (`3D FIX`, `2D FIX`, `NO FIX`), Speed over ground ($\text{km/h}$), Satellites count, Altitude ($m$), Lat/Lon coordinates with 1-click clipboard copy and Google Maps link.
- **Motion & Dynamics Panel:** LSM6DSOX 1-second RMS vibration magnitude ($mg$), Accel $X, Y, Z$ axes, Motion status (`MOVING` / `STATIONARY`), and Impact Alarm indicator (`IMPACT DETECTED`).
- **Cellular & System Health:** GSM Signal quality (RSSI & $\text{dBm}$), ThingsBoard MQTT status, system uptime, and internal free heap memory.
- **Data Export:** 1-click rolling CSV telemetry download (1,000 continuous records, ~16.6 minutes @ $1\ \text{Hz}$).

### 7.3 REST Endpoints

| Method | Endpoint | Format | Description |
|---|---|---|---|
| `GET` | `/` or `/index.html` | HTML | Interactive telemetry dashboard web application |
| `GET` | `/api/live` | JSON | Current live telemetry snapshot (1 Hz updates) |
| `GET` | `/api/log/download` | CSV | Streams rolling circular buffer as `tcu_telemetry_log.csv` |
| `GET` | `/api/snapshot/download` | JSON | Downloads current state as `tcu_telemetry_snapshot.json` |
| `POST` | `/api/log/clear` | JSON | Clears in-memory rolling history buffer |

---

## 8. FreeRTOS Tasks & Core Allocation

| Task Name | Core | Priority | Stack (Words) | Interval / Trigger | Purpose |
|---|---|---|---|---|---|
| `accel_drain` | 0 | 6 | 3072 | 160 ms periodic | High-priority IMU FIFO drainage (Must meet 160 ms deadline) |
| `bms_monitor` | 0 | 5 | 4096 | Continuous (~2 s)| Shunt-only current sampling or Modbus sweep; feeds Task WDT |
| `bms_led` | 0 | 4 | 2048 | 50 ms tick | Non-blocking 4-LED status indicator state machine |
| `tcu_dns` | 0 | 3 | 4096 | Event-driven (UDP 53)| Captive DNS query handler for SoftAP clients |
| `diag_log` | 0 | 2 | 5120 | 1 Hz periodic | Diagnostic sensor logger and live Wi-Fi telemetry feeder |
| `power_monitor`| 1 | 6 | 4096 | 500 ms / Event | Contactor/kill-switch state monitoring and modem brownout supervisor |
| `mqtt_publish` | 1 | 5 | 6144 | 60 s / Event | Consumes BMS snapshot, builds JSON, publishes to ThingsBoard; feeds WDT |
| `process_urcs` | 1 | 5 | 6144 | 100 ms queue poll | Drains intercepted modem URC notifications (RPCs) |
| `stats_monitor`| 1 | 3 | 3072 | 30 s periodic | Heap high-water mark, stack usage, and system health logging |
| `trike_wait` | 1 | 4 | 3072 | On-demand (5 s poll)| Spawned during power-off while vehicle is in motion |
| `httpd` | Any | 5 | 6144 | TCP Port 80 | ESP-IDF HTTP server serving dashboard & streaming CSV/JSON |

---

## 9. ThingsBoard Telemetry & Attributes Specification

### 9.1 Per-Battery Telemetry (Published for each pack, `BOARD_BMS_BATTERY_COUNT = 2`)

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
| `"1"` | — | Device MAC / Serial | `"14"` | V | Live Analog Pack Voltage (ADC2_CH3) |
| `"2"` | count | Valid Battery Count | `"15"` | A | Synchronized Shunt Current Average |
| `"3"` | dBm | GSM Signal Strength | `"15b"` | A | Synchronized Shunt Current Peak Max |
| `"4"` | deg | GPS Latitude | `"16"` | °C | Motor/Tiller Temperature (Steinhart-Hart) |
| `"5"` | deg | GPS Longitude | `"17"`–`"19"`| mg | Accelerometer RMS X / Y / Z |
| `"6"` | m | GPS Altitude | `"20"` | mg | Accelerometer 3D Vector Magnitude |
| `"7"` | count | GPS Visible Satellites | `"21"` | mg | Accelerometer Peak Dynamic Acceleration |
| `"8"` | km/h | GPS Speed Over Ground | `"22"` | 0/1 | Hardware Impact / Tap Detected |
| `"9"` | fix | GPS Fix Quality (0/1/2/3) | `"23"` | 0/1 | Vehicle in Motion Flag |
| `"10"` | bool | Response Command Acknowledge | `P_offset_precfg`| A | Pre-configured Offset Constant |
| `"11"` | bool | Tiller Contactor/Kill State (1=ON)| `P_offset_board` | A | Active Board Zero-Current Offset |
| `"12"` | % | Mean Traction Battery SOC | `P_bms_cycle_ms` | ms | Total BMS Cycle Duration |
| `"13"` | — | SOC Sentinel | `fw` / `hw` | str | Firmware & Hardware Versions |

---

## 10. NVS Namespaces & Storage

| Namespace | Partition | Key | Type | Description |
|---|---|---|---|---|
| `tcu_creds` | `nvs` | `prov_done`   | `u8`  | `1` if device has completed ThingsBoard provisioning |
|             |       | `device_name` | `str` | Assigned fleet name (`KTCU_0000`, `KTCU_0001`...) or fallback `TCU_<MAC>` |
|             |       | `username`    | `str` | MQTT username (assigned device name) |
|             |       | `password`    | `str` | 32-character cryptographically random token (hardware true RNG) |
|             |       | `client_id`   | `str` | MQTT client ID (device MAC hex string) |
|             |       | `prov_key`    | `str` | (Optional) Factory provisioning key; overrides Kconfig if present |
|             |       | `prov_secret` | `str` | (Optional) Factory provisioning secret; overrides Kconfig if present |
| `trike_sens`| `nvs` | `cur_offset`  | `i32` | Calibrated zero-current offset in integer mA |
|             |       | `temp_offset` | `i32` | Calibrated temperature offset in m°C |
| `trike_ctrl`| `nvs` | `pwr_conf`    | `u8`  | Tiller power / kill-switch state (restored across reboots) |
|             |       | `pwr_pend`    | `u8`  | Pending power-off state across reboots (1=killswitch armed/waiting for halt) |
| `jkbms_ser` | `nvs` | `bms0_ser`    | `str` | Cached serial number for Battery 1 |
|             |       | `bms1_ser`    | `str` | Cached serial number for Battery 2 |
|             |       | `bms2_ser`    | `str` | Cached serial number for Battery 3 |

---

## 11. MQTT Auto-Provisioning & Credential Lifecycle

```
Power On / Boot
       │
       ▼
Read Fleet Identity:
  ├── Check NVS "device_name" (e.g. KTCU_0000) ──> Set HUB_NAME & Wi-Fi AP SSID
  └── If missing in NVS ──> Fall back to hardware MAC (TCU_<MAC>)
       │
       ▼
tcu_load_or_provision_creds()
       ├── [prov_done == 1] ──> Load credentials from NVS ──> Connect to ThingsBoard
       │                              │
       │                              └── (Auth Code 5: Auth Rejected) ──> Reprovision (keeps device_name)
       ▼
Provisioning Workflow:
  1. Retrieve provisioning key/secret:
       ├── Try NVS ("prov_key" / "prov_secret")
       └── Fall back to Kconfig (CONFIG_TB_PROVISION_KEY / CONFIG_TB_PROVISION_SECRET)
  2. Generate 32-character random password using ESP32 hardware RNG (esp_fill_random).
  3. Connect to ThingsBoard with clientId = MAC, username = "provision".
  4. Subscribe to /provision/response.
  5. Publish request to /provision/request with deviceName = HUB_NAME and random password.
  6. Intercept +QMTRECV URC notification containing confirmed access credentials.
  7. Commit credentials to NVS under `tcu_creds` namespace, set prov_done = 1.
  8. Reboot MCU via esp_restart() to begin production telemetry streaming.
```

> [!NOTE]
> **Zero Hardcoded Secrets & Transport Encryption:**
> - **Hardware TLS (MQTTS Port 8883):** All cellular traffic (provisioning, telemetry, and RPC) is encrypted via the Quectel EG915N internal SSL engine (`AT+QMTCFG="ssl"` & `AT+QSSLCFG`). Zero plaintext MQTT.
> - **RPC Anti-Replay Guard:** Incoming ThingsBoard RPC commands are de-duplicated against a 32-entry circular buffer. Duplicate `requestId`s (such as replayed power-off commands) are automatically detected and dropped.
> - **NVS Credential Survival:** Device credentials survive power cycles, and credential erasure (re-provisioning) preserves the assigned `device_name`.

---

## 12. Factory Boot Self-Test Suite

Every cold boot executes `factory_test_run()` in `components/factory_test/factory_test.c`:
1. **LSM6DSOX IMU Test:** Verifies WHO_AM_I register (`0x6C`) over I2C0.
2. **Modem AT Test:** Validates communication (`AT`), echoes off (`ATE0`), checks model (`ATI`), IMEI (`AT+GSN`), CSQ signal quality (`AT+CSQ`), and SIM presence (`AT+CPIN?`).
3. **Status LEDs Test:** Sequences Red, Green, and Blue channels for visual inspection.
4. **ADC Rail Test:** Reads pack voltage, shunt midpoint, and NTC temperature.

---

## 13. Build, Flash & Monitor

### 13.1 Building with PlatformIO (Recommended)

The project includes an optimized `platformio.ini` configured with 16MB flash and ESP32 exception decoding:

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

# Configure provisioning keys (optional, or inject via environment/NVS)
idf.py menuconfig   # -> ThingsBoard Configuration

# Build
idf.py build

# Flash and open serial monitor
idf.py -p /dev/ttyACM0 flash monitor
```

### 13.3 Factory Sequential Fleet Provisioning Workflow (`tools/`)

The firmware supports a **universal binary workflow**: you compile the firmware binary once, and individual tillers receive their unique sequential identities (`KTCU_0000`, `KTCU_0001`, `KTCU_0002`...) via an automated NVS provisioning script.

#### Step 1: Compile Firmware Once
Compile the universal application binary (reused across all vehicles without code changes):
```bash
platformio run     # Or: idf.py build
```

#### Step 2: Check Fleet Status
Preview the next sequential device name to be assigned:
```bash
./tools/flash_next.sh --status
# Or preview without flashing:
./tools/flash_next.sh --dry-run
```

#### Step 3: Connect the Board via USB
Plug the ESP32-S3 board into your computer. The script automatically detects the active serial port (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`).

#### Step 4: Execute Flashing
Choose between full flashing or fast identity flashing:

* **Option A: Brand-New / Unprogrammed Board (Full Flash)**  
  Flashes bootloader (`0x0`), partition table (`0x8000`), NVS identity (`0x9000`), and application (`0x10000`):
  ```bash
  ./tools/flash_next.sh --full
  ```

* **Option B: Pre-Flashed Board (Fast NVS Flash ~1s)**  
  If the board already has firmware and only needs its sequential identity assigned:
  ```bash
  ./tools/flash_next.sh
  ```

#### What Happens Automatically
1. **MAC Detection:** Reads the hardware eFuse MAC address via `esptool`.
2. **NVS Partition Generation:** Creates a compliant 24 KB ESP-IDF NVS v2 binary containing `device_name = "KTCU_xxxx"`.
3. **Flashing:** Writes to offset `0x9000` (and application partitions if `--full`).
4. **Audit Logging:** Appends a timestamped record to `tools/provisioned_fleet.csv`:
   ```csv
   2026-11-10 11:45:10, KTCU_0000, A4CF12B3E501, /dev/ttyACM0, SUCCESS
   ```
5. **Auto-Increment:** Advances `tools/fleet_registry.json` so the next board flashed receives `KTCU_0001`.

#### Step 5: Post-Flash Verification
Upon reboot, verify the device:
1. **Serial Console:** Confirm log output:
   ```text
   I (1022) TCU_MAIN: Fleet device name loaded from NVS: KTCU_0000
   I (1032) TCU_WIFI_AP: Wi-Fi SoftAP started successfully!
   I (1033) TCU_WIFI_AP:   SSID: KTCU_0000
   ```
2. **Wi-Fi SoftAP:** Scan for Wi-Fi networks on a phone or laptop. The SSID will be `KTCU_0000` (Default Password: `12345678`).
3. **ThingsBoard Provisioning:** When the cellular modem attaches, the TCU auto-provisions under `KTCU_0000` with a 32-character hardware-RNG password.

#### Flashing the Next Board in Assembly
Unplug the finished board, plug in the next board, and run:
```bash
./tools/flash_next.sh
```
The board will automatically receive `KTCU_0001`, then `KTCU_0002`, and so forth.

#### Manual Overrides
```bash
./tools/flash_next.sh --set-id 10         # Jump counter to KTCU_0010
./tools/flash_next.sh --set-prefix TILLER_ # Change prefix to TILLER_0000
./tools/flash_next.sh --name KTCU_SPECIAL # Flash a specific one-off name
./tools/flash_next.sh --port /dev/ttyUSB1 # Specify custom serial port
```

---

## 14. Production Verification Checklist

1. [ ] **Shunt Current Polarity & Offset:** Confirm that `BOARD_CURRENT_VREF_MV` is set to `1452.0f` and `delta_mv = vref_mv - adc_mv` in `trike_sensors.c` so discharge current is positive.
2. [ ] **Operating Mode Verification:** Confirm whether the tiller is operating in **Shunt-Only Mode** (bypassed RS-485 Modbus) or **Full Modbus Mode** in `bms_monitor_task.c`.
3. [ ] **Active Battery Count:** Ensure `BOARD_BMS_BATTERY_COUNT` in `board_config.h` matches the installed pack count (default: `2`).
4. [ ] **Kill Switch Contactor Pin:** Confirm that `BOARD_TRIKE_PWR_PIN` is configured to `GPIO_NUM_21` (Net `Kill_switch`).
5. [ ] **Wi-Fi SoftAP Security:** Verify default Wi-Fi password in `tcu_wifi_ap.h` (`TCU_WIFI_AP_DEFAULT_PASSWORD`) if field deployment requires custom credentials.
6. [ ] **Sequential NVS Naming & Provisioning:** Verify sequential identity has been flashed via `./tools/flash_next.sh` (or confirm fallback to `TCU_<MAC>`). Ensure ThingsBoard provisioning credentials are set via Kconfig or NVS.
