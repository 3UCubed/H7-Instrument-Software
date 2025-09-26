# BootManager (Instrument Firmware)

The **BootManager** is a minimal firmware that resides at the **start of flash** (`0x08000000`) on the STM32H753-based Instrument MCU.  
Its purpose is to validate and chain-load the primary application firmware, or fall back to the STM32 ROM bootloader when needed.

---

## Memory Layout

- **BootManager**: `0x08000000` → starts at reset
- **Application**: `0x08020000` → normal Instrument firmware
- **Stored CRC**: `0x08040000` → application CRC value written after build
- **System Bootloader (ROM)**: `0x1FF09800`

---

## Boot Flow

1. **Reset** into BootManager at `0x08000000`.
2. **Manual Boot Window** (1 second, UART @ 115200 9E1):
   - Send byte `0x2B` → BootManager jumps immediately to the STM32 system bootloader.
3. **RAM Flag Check**:
   - If `*(0x2001FFF0) == 0xDEADBEEF`, BootManager clears the flag and jumps to the system bootloader.
4. **Application Validation**:
   - **SP/PC check** – validates the app vector table (stack pointer in RAM, reset handler in flash, Thumb bit set).
   - **CRC check** – computes CRC32 over `[0x08020000, 0x0803FFFF]` and compares against the stored CRC at `0x08040000`.
5. **Branching**:
   - If **valid** → jump to application @ `0x08020000`.
   - If **invalid** → jump to system bootloader @ `0x1FF09800`.

---

## Features

- **Fail-safe**: Corrupt or missing app code always falls back to ROM bootloader.
- **Manual override**: Force bootloader entry with UART `0x2B` within the 1s window.
- **RAM override**: Setting the flag at `0x2001FFF0` to `0xDEADBEEF` forces next boot into ROM bootloader.
- **Verbose UART logs**: Provides visibility into validity checks, CRC results, and boot decisions.

---

## Usage

### Normal Operation

- After reset, BootManager validates and jumps to the Instrument firmware.
- UART logs confirm the validation sequence.

### Forcing Bootloader Mode

- **UART method**: Send `0x2B` within 1s after reset.
- **RAM flag method**: Write `0xDEADBEEF` to address `0x2001FFF0` before reset.

### Application CRC Workflow

- Application is built with space reserved at `0x08040000` for the CRC.
- After build of H7 firmware, a Python script computes CRC32 over `[0x08020000, 0x0803FFFF]` and patches the result at `0x08040000`.

---

## Integration Notes

- **BootManager linker script**: places firmware at `0x08000000`.
- **Application linker script**: places firmware at `0x08020000`.
- **Firmware updater tool**:
  - Erases/writes only `>= 0x08020000`.
  - Appends CRC word at `0x08040000` after flashing.
- **Option bytes**: must boot from Flash (`0x08000000`), not ROM.
