# STM32G4 CAN-FD Sensor Node

> **Status:** Work in Progress (WIP) — Hardware peripherals (`LED_STATUS` & `TMP101` over `I2C1`) and firmware build (`0 errors, 0 warnings`) are progressing toward final KiCad `ERC`/`DRC` and `PCB` layout.

## Overview
An embedded sensor node hardware and firmware design built around the **STM32G431CBUx** microcontroller (`UFQFPN48`), targeting CAN-FD communication alongside analog and digital sensor interfaces.

## What Has Been Done & The Story So Far

1. **Core MCU & System Setup**
   - Configured the **STM32G431CBUx** project across **STM32CubeMX** and **STM32CubeIDE** (`stm32g4-canfd-sensor-node.ioc`), including `ADC1` (`ADC1_IN1` on `PA0`) and `FDCAN1` (`PA11` / `PA12`).
   - Organized KiCad 3D package models locally under `/home/ensiyeh/kicad_3dmodels/kicad-packages3D` with `KICAD10_3DMODEL_DIR` configured.

2. **Status LED (`PB0`) & Firmware Bring-Up**
   - Added status LED `D4` (yellow, `0603`) with series resistor `R9` (`820R`, `0603`) driven by `PB0` (`LED_STATUS`).
   - Added blink logic (`HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin)` with `HAL_Delay(500)`) inside `while (1)` in `Core/Src/main.c` and verified clean builds (`0 errors, 0 warnings`) in STM32CubeIDE.

3. **Digital Temperature Sensor (`TMP101` on `I2C1`)**
   - Selected **`TMP101`** (`SOT-23-6`) in KiCad (`U4`) with decoupling capacitor `C22` (`100nF`, `0603`) between `+3V3` and `GND`, pull-up resistors `R10` and `R11` (`4.7k` / `4k7`, `0603`) to `+3V3` on `SDA` and `SCL`, `ADD0` tied to `GND` (7-bit address `0x48`), and `ALERT` left unconnected (`No Connect`).
   - Enabled **`I2C1`** (`100 kHz` Standard Mode, 7-bit addressing) in STM32CubeMX on **`PB7` (`I2C1_SDA`)** and **`PA15` (`I2C1_SCL`)**, generated `i2c.c` / `i2c.h` and `MX_I2C1_Init()`, and added 12-bit temperature reading (`TMP101_ReadCelsius` via `HAL_I2C_Mem_Read` from register `0x00`) in `Core/Src/main.c`.
   - Verified the updated firmware builds with **`0 errors, 0 warnings`**.

## Next Steps
- [ ] Run final KiCad `ERC` on the schematic and update the `PCB` (`Update PCB from Schematic` / `F8`) with the new components (`D4`, `R9`, `U4`, `C22`, `R10`, `R11`).
- [ ] Complete `PCB` placement, routing, and `DRC` checks.
- [ ] Test and debug firmware on target hardware and publish `temperature_c` over `FDCAN1`.
