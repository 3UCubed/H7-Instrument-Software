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
- See accepted commands

## License

This project is licensed under the [MIT License](LICENSE). See the [LICENSE](LICENSE) file for details.

