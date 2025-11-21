# Smart Plant Monitoring System (SPMS)

The **Smart Plant Monitoring System (SPMS)** is embedded firmware for the **Nordic nRF52DK (nRF52832)** using the **nRF Connect SDK (Zephyr RTOS)**.

It periodically measures environmental values for plant health, logs them to on-board flash using NVS, and exposes live data + historical logs over Bluetooth Low Energy (BLE).

---

## Project Overview

SPMS monitors:
- **Temperature & Humidity** (CHT832X / SEN0546 via I²C)
- **Ambient Light** (SEN0390 via I²C)
- **Soil Moisture** (SEN0114 via SAADC)
- **Timestamps** (monotonic seconds)

The firmware:
- Logs data into an NVS-backed ring buffer
- Streams logs over BLE through a custom service
- Stores and updates a configurable **Plant Profile**
- Sends live sensor notifications over BLE
- Uses a simple timing scheme (1 Hz BLE tick, 5 s sensor updates, 60 s logging)

---

## Directory Structure

Project root: `rev_0/`

```text
rev_0/
├── build/                   # Build artifacts (generated)
├── debug/                   # Debugger / tooling configs
├── include/                 # Public project headers
│   ├── log_store.h
│   ├── spms_ble.h
│   ├── spms_sensors.h
│   ├── sen0114.h
│   ├── sen0390.h
│   └── sen0546.h
├── src/                     # Firmware source files
│   ├── main.c               # Main loop, scheduling, LED + app glue
│   ├── log_store.c          # NVS-backed logging + plant profile persistence
│   ├── spms_ble.c           # BLE services, GATT characteristics, log dump
│   ├── spms_sensors.c       # Sensor reads, filtering, unit conversions
│   ├── sen0114.c            # Soil moisture ADC driver
│   ├── sen0390.c            # Ambient light sensor driver + lux estimator
│   └── sen0546.c            # Temp/Humidity sensor driver (CHT832X)
├── CMakeLists.txt           # Zephyr build configuration
├── prj.conf                 # Kconfig options for this application
├── nrf52dk_nrf52832.overlay # Devicetree overlay for sensors, LEDs, ADC
└── pm_static.yml            # Flash partition map (NVS storage for logs/profile)
```

This layout follows a simple pattern:
- **`include/`**: all public headers for modules
- **`src/`**: all C source files (implementations)
- Top-level config files (`CMakeLists.txt`, `prj.conf`, overlay, partition map) live directly under `rev_0/`.

---

## Hardware Platform

**Board:** Nordic nRF52DK / nRF52832  

**Sensors:**
- Temp/Humidity – CHT832X (module `sen0546`)
- Ambient Light – SEN0390
- Soil Moisture – SEN0114 (analog, via SAADC)

**LEDs:**
- **LED1** – BLE connection indicator (on when connected)
- **LED2** – Log write indicator (brief flash when a record is stored)

---

## Build & Flash

Requirements:
- nRF Connect SDK 3.x
- Zephyr toolchain + west

Typical flow:

```bash
cd rev_0
west build -b nrf52dk_nrf52832 .
west flash
```

Then attach a serial terminal to the board’s VCOM port (115200 8N1) to view log output.

---

## BLE Services (High-Level)

The firmware exposes three custom BLE services:

1. **SPMS Data Service**
   - Temperature (read / notify) — 0.01 °C units
   - Humidity (read / notify) — 0.01 %RH units
   - Debug value (read / write / notify)

2. **Plant Profile Service**
   - Read/write the full plant threshold structure (temperature, humidity, moisture, lux)
   - Automatically persisted to NVS

3. **Log Dump Service**
   - Control characteristic (start/stop dump)
   - Data characteristic (notifications sending raw `struct log_record` entries)

---

## Logging System

A `struct log_record` is stored every **60 seconds**:

```c
struct log_record {
    uint32_t ts_s;        // Timestamp (seconds since boot)
    int32_t  t_raw;       // Temperature x100 (0.01 °C)
    int32_t  rh_raw;      // Relative humidity x100 (0.01 %RH)
    uint32_t lux_raw;     // Raw 32-bit light sensor value
    int32_t  moisture_mv; // Soil moisture (millivolts)
};
```

Records are kept in a ring buffer using NVS in the `storage` partition defined by `pm_static.yml`.  
The full history can be streamed over BLE via the Log Dump service.

---

## Intended Audience

This README is aimed at **engineering students and embedded developers** who want to:

- Understand the project layout at a glance
- Build and flash the firmware
- Explore or extend the sensors, BLE services, or logging system
