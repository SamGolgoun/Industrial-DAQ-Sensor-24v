# STM32G4 CAN-FD Sensor Node

> **Status:** Work in Progress (WIP) — KiCad schematic design is actively under development and ERC verification.

## Overview
**STM32G4 CAN-FD Sensor Node** is an embedded sensor node hardware design built around the **STM32G431CBUx** microcontroller (**UFQFPN48** package) with a **CAN-FD** bus interface, designed in **KiCad**.

## Key Features & Architecture
- **Microcontroller:** STM32G431CBUx (UFQFPN48 package) with external high-speed crystal (`HSE`, `8 MHz`), reset (`NRST`), `BOOT0` configuration, and `SWD` programming/debug header (`J2`).
- **Power Supply & Analog Reference:** Regulated power rails (`+3V3`) and filtered analog supply (`+3V3_ANA`) via ferrite bead (`FB1`), with decoupling capacitors (`C20` `100nF` on `TCAN332` `VCC`, plus `VDD`/`VDDA`/`VREF+` decoupling).
- **CAN-FD Transceiver:** Non-isolated `3.3V` **TCAN332** (`U3`) transceiver interfaced to `FDCAN1_TX` (`PA12` / pin `34`) and `FDCAN1_RX` (`PA11` / pin `33`) using directional global labels.
- **Bus Connector & Switchable Termination:** 4-pin bus connector `J1` (`24V_IN`, `GND`, `CAN_H`, `CAN_L`) with a `120 Ω` termination resistor (`R6`) in series with a 2-pin jumper (`J3`, `Conn_01x02`) so termination can be enabled or disabled depending on bus position.

## Current Progress
- [x] Core schematic setup in KiCad (power, `NRST`, `BOOT0`, `SWD`, `8 MHz` `HSE`)
- [x] `CAN-FD` stage with `TCAN332` (`U3`) and jumper-selectable `120 Ω` termination (`R6` + `J3`)
- [x] Directional global labels and `ERC` fixes for `VREF+` and `VDDA` analog supply
- [ ] Sensor interfaces and remaining MCU pin assignments
- [ ] Clean final `ERC` pass and `PCB` layout
