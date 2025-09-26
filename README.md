# Instrument Firmware Repository

This repository contains the complete firmware stack for the **Instrument MCU** used on the CubeSat platform.  
It is organized into two primary components:

- **BootManager** – a minimal startup firmware that validates and chain-loads the application, or falls back to the STM32 ROM bootloader.
- **H7 Instrument Firmware** – the main FreeRTOS-based application responsible for science operations, housekeeping, high-voltage rail control, and packet transmission.

---

## Overview

The Instrument MCU runs a **dual-image architecture**:

1. **BootManager** (small, at flash base):

   - Validates the application image (stack pointer / reset vector / CRC).
   - Supports manual UART override and RAM flag to force entry into the STM32 system bootloader.
   - Always ensures safe recovery if the application is corrupt or update fails.

2. **H7 Instrument Application** (main firmware):
   - Runs FreeRTOS tasks for science and housekeeping packet generation.
   - Controls multiple power rails (+5V, −3.3V, −5V, +15V, −200V, +800V).
   - Provides a UART command interface for mode control, rail toggling, and data requests.
   - Integrates ECC error callbacks, reset-cause tracking, watchdog refresh, and voltage monitoring.
   - Responds to firmware update commands by setting the BootManager flag and resetting into the ST ROM bootloader.

---

## Development Workflow

- **BootManager** and **Application** are built separately, each with its own linker script:
  - BootManager @ `0x08000000`
  - Application @ `0x08020000`
- The **Application** reserves space for a CRC word (patched after build) which BootManager checks before jumping.
- Firmware updates are coordinated by the **groundstation** via UART:
  - Upload new app binary to the OBC.
  - OBC triggers update command (`0x2A`) → MCU resets → BootManager jumps to ROM bootloader for flashing.

---

## Documentation

- [BootManager README](BootManager_H753/README.md) – covers startup logic, memory layout, validation, and update flow.
- [H7 Instrument Firmware README](Firmware_H753/README.md) – covers main app features, tasks, commands, and packet handling.
