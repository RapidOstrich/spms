# Smart Plant Monitoring System (SPMS)

This repository contains the system software for the **Smart Plant Monitoring System (SPMS)**, developed for the **Nordic nRF52DK/nRF52832** platform using the **nRF Connect SDK**.

## Overview
The SPMS project is designed to monitor environmental conditions for plant health. It utilizes the nRF52 series microcontroller for low-power sensing and wireless communication.

## Directory Structure
- **`blinky_pwn/`** – A copied sample project used to verify board functionality prior to system development.  
- **`sen0546/`** – Contains setup code and driver software for interfacing with the **CHT832X** humidity and temperature sensor.

## Platform
- **Hardware:** Nordic nRF52DK / nRF52832  
- **SDK:** nRF Connect SDK (Zephyr RTOS)  
- **Language:** C  

## Getting Started
1. Open the desired project (e.g., `sen0546`) in **nRF Connect for VS Code**.  
2. Select the appropriate **board target** (e.g., `nrf52dk_nrf52832`).  
3. Build and flash the project to the board.  
4. Open a serial terminal to view debug output.  

## Future Development
- Sensor data logging  
- BLE data transmission  
- Power optimization for long-term operation  
