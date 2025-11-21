# Smart Plant Monitoring System (SPMS)

The **Smart Plant Monitoring System (SPMS)** is embedded firmware for the **Nordic nRF52DK (nRF52832)** using the **nRF Connect SDK (Zephyr RTOS)**.

It periodically measures environmental values for plant health, logs them to on‑board flash using NVS, and exposes live data + historical logs over Bluetooth Low Energy (BLE).

---

## Project Overview

SPMS monitors:
- **Temperature & Humidity** (CHT832X / SEN0546 via I²C)
- **Ambient Light** (SEN0390 via I²C)
- **Soil Moisture** (SEN0114 via SAADC)
- **Timestamps** (monotonic seconds)

The firmware:
- Logs data into an NVS‑backed ring buffer
- Streams logs over BLE through a custom service
- Stores and updates a configurable **Plant Profile**
- Sends live sensor notifications over BLE
- Provides a simple 1 Hz BLE tick + 5 s sensor update cycle

---

## Directory Structure

```
rev_0/
├── build/                 # Build artifacts (generated)
├── debug/                 # Debugger configs
├── include/               # Public headers
├── src/                   # Firmware source
│   ├── main.c             # Main loop / scheduling
│   ├── spms_ble.c/h       # BLE services & notifications
│   ├── spms_sensors.c/h   # Sensor reads, filtering, conversions
│   ├── log_store.c/h      # NVS ring buffer logging + plant profile
│   ├── sen0114.c/h        # Soil moisture ADC driver
│   ├── sen0390.c/h        # Ambient light driver
│   ├── sen0546.c/h        # Temp/Humidity sensor driver
├── CMakeLists.txt         # Build configuration
├── nrf52dk_nrf52832.overlay
│                          # Device tree overlay for sensors/LEDs/ADC
└── pm_static.yml          # Flash/NVS partition map
```

---

## Hardware Platform

**Board:** Nordic nRF52DK / nRF52832  
**Sensors:**  
- Temp/Humidity (CHT832X / SEN0546)  
- Ambient Light (SEN0390)  
- Soil Moisture (SEN0114 ADC)  
**LEDs:**  
- LED1 – BLE connection indicator  
- LED2 – Log write indicator  

---

## Build & Flash

Requires:
- nRF Connect SDK 3.x  
- West + Zephyr toolchain  

```
cd rev_0
west build -b nrf52dk_nrf52832 .
west flash
```

---

## BLE Services Summary

### **SPMS Data Service**
- Temperature (read / notify)
- Humidity (read / notify)
- Debug value (read / write / notify)

### **Plant Profile Service**
- Read/write full plant threshold structure  
- Automatically persisted to NVS  

### **Log Dump Service**
- Control characteristic (start/stop dump)
- Data characteristic (notifications of log records)

---

## Logging System

A `struct log_record` is stored every **60 seconds**:

```
uint32_t ts_s;        // Timestamp (seconds)
int32_t  t_raw;       // Temperature x100
int32_t  rh_raw;      // Relative humidity x100
uint32_t lux_raw;     // Raw 32-bit light sample
int32_t  moisture_mv; // Soil moisture (millivolts)
```

Records are written to a ring buffer implemented on NVS.  
A full historical dump can be streamed via BLE.

---

## Intended Audience

This README is designed for **engineering students or embedded developers** who want to understand the project layout and build/run the firmware.

