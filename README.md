# H7-Instrument-Software

## Overview

This project involves a microcontroller-based system designed to handle and transmit various types of data related to housekeeping, PMT (Photomultiplier Tube), and ERPA. It features real-time data acquisition, processing, and transmission via UART with DMA. The system integrates multiple peripherals including ADCs, DACs, timers, and GPIOs to accomplish its tasks.

## Features

- **Data Sampling**: Collects and processes data from sensors and peripherals.
- **Real-Time Communication**: Transmits data using UART with DMA.
- **Power Management**: Supports entering and exiting low-power modes.
- **Error Handling**: Implements error protocols for system reliability.
- **Synchronization**: Calibrates and synchronizes RTC with external systems.

## Components

- **Microcontroller**: STM32 series (or equivalent)
- **Peripherals**: ADCs, DACs, UART, Timers, GPIO
- **RTOS**: FreeRTOS for task management and inter-task communication

## Functionality

### Sampling Functions

- **`sample_pmt()`**: Collects data from the PMT sensor, including SPI data and uptime, then sends it as a packet.
- **`sample_erpa()`**: Collects data from the ERPA sensor, including SPI data, ADC readings, and uptime, then sends it as a packet.
- **`sample_hk()`**: Collects housekeeping data from sensors and I2C devices, and sends it as a packet.

### Communication

- **`UART_TX_init(void *argument)`**: Task responsible for retrieving messages from the queue and transmitting them via UART using DMA.

### Power Management

- **`enter_stop()`**: Puts the system into stop mode and resumes operations upon waking.

### Synchronization

- **`sync()`**: Handles synchronization with an external system, calibrates the RTC, and sends acknowledgements.

## Error Handling

- Implement error handling with **`error_protocol()`** to manage and report system errors.

## Dependencies

- STM32 HAL Library
- FreeRTOS

## Usage
| Command | Action | Meaning |
|---------|--------|---------|
| `0x10` | SDN1 ON | Turn on SDN1 (High) |
| `0x00` | SDN1 OFF | Turn off SDN1 (Low) |
| `0x11` | SYS ON PB5 | Turn on system on PB5 |
| `0x01` | SYS OFF PB5 | Turn off system on PB5 |
| `0x12` | 3V3 ON PC10 | Turn on 3.3V on PC10 |
| `0x02` | 3V3 OFF PC10 | Turn off 3.3V on PC10 |
| `0x13` | 5V ON PC7 | Turn on 5V on PC7 |
| `0x03` | 5V OFF PC7 | Turn off 5V on PC7 |
| `0x14` | N3V3 ON PC6 | Turn on negative 3.3V on PC6 |
| `0x04` | N3V3 OFF PC6 | Turn off negative 3.3V on PC6 |
| `0x15` | N5V ON PC8 | Turn on negative 5V on PC8 |
| `0x05` | N5V OFF PC8 | Turn off negative 5V on PC8 |
| `0x16` | 15V ON PC9 | Turn on 15V on PC9 |
| `0x06` | 15V OFF PC9 | Turn off 15V on PC9 |
| `0x17` | N200V ON PC13 | Turn on negative 200V on PC13 |
| `0x07` | N200V OFF PC13 | Turn off negative 200V on PC13 |
| `0x18` | 800V ON PB6 | Turn on 800V on PB6 |
| `0x08` | 800V OFF PB6 | Turn off 800V on PB6 |
| `0x19` | AUTO SWEEP ON | Turn on automatic sweep |
| `0x09` | AUTO SWEEP OFF | Turn off automatic sweep |
| `0x1A` | ERPA ON | Turn on ERPA |
| `0x0A` | ERPA OFF | Turn off ERPA |
| `0x1B` | PMT ON | Turn on Photomultiplier Tube |
| `0x0B` | PMT OFF | Turn off Photomultiplier Tube |
| `0x1C` | HK ON | Turn on Housekeeping |
| `0x0C` | HK OFF | Turn off Housekeeping |
| `0x1D` | STEP UP | Increase step |
| `0x0D` | STEP DOWN | Decrease step |
| `0x1E` | FACTOR UP | Increase factor |
| `0x0E` | FACTOR DOWN | Decrease factor |
| `Any` | WAKE UP | Wake up the system |
| `0x0F` | SLEEP | Put the system to sleep |
| `0xE0` | AUTO INIT | Automatic initialization |
| `0xD0` | AUTO DEINIT | Automatic deinitialization |
| `0xAF` | SYNC | Synchronize |
| `0xBF` | FLIGHT MODE | Enter flight mode |
| `0xCF` | IDLE MODE | Enter Idle mode |
| `0xDF` | RESET ERROR COUNTERS | Reset error counters |
| `0xEF` | SEND PREVIOUS ERROR | Send the last error packet |
| `0x1F` | SEND VERSION PACKET | Send the version packet |
| `0x2F` | SEND VERSION INFO | Send the version information |
| `0x2A` | ENTER BOOTLOADER | Enter the integrated bootloader |
## License

This project is licensed under the [MIT License](LICENSE). See the [LICENSE](LICENSE) file for details.

