# H7-Instrument-Software

## Overview

This project is the **Instrument firmware** for an STM32H753-based system that handles and transmits data for **Housekeeping (HK)**, **PMT (Photomultiplier Tube)**, and **ERPA**. It performs real-time acquisition, processing, and transmission over UART (interrupt/DMA driven), and controls multiple power rails and generators. The firmware runs under **FreeRTOS** and supports a BootManager-based update flow.

## Features

- **Data Sampling**: Collects, processes, and packets HK/PMT/ERPA data.
- **Real-Time Communication**: UART TX/RX via interrupts/DMA for robust streaming.
- **Power Management**: Enter/exit STOP mode and orderly rail control.
- **Error Handling**: ECC callbacks, reset-cause capture, counters, and reporting.
- **Synchronization & Modes**: Sync/Science/Idle mode flags with deterministic timers.
- **Boot/Update Support**: Ground command to enter the ST ROM bootloader via BootManager.

## Components

- **Microcontroller**: STM32H753 (Cortex-M7)
- **Peripherals**: ADC1/ADC3, DAC1 (DMA), SPI1/SPI2, I2C1, USART1, TIM1/TIM2/TIM3, RTC, IWDG1, RAMECC, DMA, GPIO
- **RTOS**: FreeRTOS for task scheduling and inter-task signaling (event flags)

## Functionality

### Sampling Functions

- **`sample_pmt()`**: Samples PMT (SPI/uptime) and sends a PMT packet.
- **`sample_erpa()`**: Samples ERPA (SPI/ADC/uptime) and sends an ERPA packet.
- **`sample_hk()`**: Samples housekeeping (sensors/I2C rails/etc.) and sends an HK packet.

### Communication

- **`UART_TX_init(void *argument)`**: TX task pulls from the queue and transmits via UART (DMA/interrupts).

### Power Management

- **`enter_stop()`**: Suspends RTOS tasks, enters STOP mode, and resumes via clean reset path.

### Synchronization & Modes

- **`sync()`** (if used): Coordinates with external system timing and RTC.
- **Mode flags** (SYNC, SCIENCE, IDLE) drive which packet producers/timers are active.

## Boot & Update (How the App Works with BootManager)

- **BootManager** at `0x08000000` validates the application at `0x08020000` and chain-loads it when valid; otherwise it jumps to the **STM32 ROM Bootloader** at `0x1FF09800`.
- In the **Instrument App**, sending **`CMD_UPDATE_FIRMWARE (0x2A)`** sets a RAM flag and resets the MCU. On the next boot, **BootManager** detects the flag and immediately jumps to the ROM bootloader for flashing.
- **`CMD_ST_BL_SYNC (0x7F)`** returns an immediate `0x79` ACK to help ground tools verify the link before switching into the ST bootloader workflow.

## Error Handling

Use the project’s error framework to capture and report faults:

- **ECC Callbacks**: `HAL_FLASHEx_EccCorrectionCallback()` / `HAL_FLASHEx_EccDetectionCallback()`
- **Reset Cause**: `get_reset_cause()` records IWDG/BOR events
- **Central Handler**: `Error_Handler()` increments counters and logs/reporting

## Dependencies

- STM32 HAL (H7)
- FreeRTOS

## Usage

| Command | Action               | Meaning                                               |
| ------- | -------------------- | ----------------------------------------------------- |
| `0x10`  | SDN1 ON              | Turn on SDN1 (High)                                   |
| `0x00`  | SDN1 OFF             | Turn off SDN1 (Low)                                   |
| `0x11`  | SYS ON PB5           | Turn on system on PB5                                 |
| `0x01`  | SYS OFF PB5          | Turn off system on PB5                                |
| `0x12`  | 5V ON PC7            | Turn on 5V on PC7                                     |
| `0x02`  | 5V OFF PC7           | Turn off 5V on PC7                                    |
| `0x13`  | N3V3 ON PC6          | Turn on negative 3.3V on PC6                          |
| `0x03`  | N3V3 OFF PC6         | Turn off negative 3.3V on PC6                         |
| `0x14`  | N5V ON PC8           | Turn on negative 5V on PC8                            |
| `0x04`  | N5V OFF PC8          | Turn off negative 5V on PC8                           |
| `0x15`  | 15V ON PC9           | Turn on 15V on PC9                                    |
| `0x05`  | 15V OFF PC9          | Turn off 15V on PC9                                   |
| `0x16`  | N200V ON PC13        | Turn on negative 200V on PC13                         |
| `0x06`  | N200V OFF PC13       | Turn off negative 200V on PC13                        |
| `0x17`  | 800V ON PB6          | Turn on 800V on PB6                                   |
| `0x07`  | 800V OFF PB6         | Turn off 800V on PB6                                  |
| `0x18`  | AUTO SWEEP ON        | Enable DAC auto-sweep (DMA)                           |
| `0x08`  | AUTO SWEEP OFF       | Disable DAC auto-sweep                                |
| `0x19`  | ERPA ON              | Enable ERPA (sets PWM via TIM2->CCR4)                 |
| `0x09`  | ERPA OFF             | Disable ERPA (CCR4 = 0)                               |
| `0x1A`  | PMT ON               | Start PMT (TIM1 OC IT) and queue PMT packet           |
| `0x0A`  | PMT OFF              | Stop PMT (TIM1 OC IT)                                 |
| `0x1B`  | HK ON                | Enable/queue HK packets                               |
| `0x0B`  | HK OFF               | Disable HK packets                                    |
| `0x1C`  | STEP UP              | Increase DAC step (bounded)                           |
| `0x0C`  | STEP DOWN            | Decrease DAC step (bounded)                           |
| `0x1D`  | FACTOR UP            | Increase cadence (double, up to limit)                |
| `0x0D`  | FACTOR DOWN          | Decrease cadence (halve, down to limit)               |
| `Any`   | WAKE UP              | Wake the system (if applicable)                       |
| `0x1E`  | AUTO INIT            | Automatic initialization sequence                     |
| `0x0E`  | AUTO DEINIT          | Automatic de-initialization sequence                  |
| `0xA0`  | SYNC                 | Enter Sync mode                                       |
| `0xA1`  | SCIENCE MODE         | Enter Science mode                                    |
| `0xA2`  | IDLE MODE            | Enter Idle mode                                       |
| `0xA3`  | SLEEP                | Enter STOP mode (low power)                           |
| `0xA4`  | RESET ERROR COUNTERS | Reset error counters                                  |
| `0xA5`  | SEND PREVIOUS ERROR  | Send last error (if enabled)                          |
| `0xA6`  | SEND VERSION PACKET  | Send 5-byte version packet                            |
| `0xA7`  | SEND VERSION INFO    | Print `GIT_INFO` string (if available)                |
| `0x2A`  | UPDATE FIRMWARE      | Set BootManager flag and reset into ST ROM bootloader |
| `0x7F`  | ST BL SYNC           | Respond `0x79` ACK (pre-bootloader link check)        |
