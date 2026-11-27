# Industrial DAQ Sensor Node (24V) — STM32G4 CAN-FD PCB Design

## Overview & Purpose
This project designs a **24V industrial sensor and data acquisition (DAQ) node** in **KiCad** built around the **STM32G431CBUx (UFQFPN48)** microcontroller and a **TCAN332 CAN-FD transceiver** for industrial sensing and CAN-FD communication.

## Current Status
- **Schematic:** Finalized and included in the repository.
- **PCB Layout (In Progress):** The `.kicad_pcb` layout is being routed and validated using an **AI-assisted, code-driven workflow** in **VS Code** with **Python** scripts rather than manual GUI-only edits.

## Key Engineering Features
- **Code-Driven & AI-Assisted Workflow:** Routing steps, geometry checks, and rule gates are scripted in Python inside VS Code with AI assistant tools.
- **Safety & Pre-Write Rule Gates:** Timestamped backups are created before edits; pre-write checks enforce component and courtyard clearance, forbid routing over component bodies or courtyards, prevent unintended `GND` shorts, and verify parenthesis balance.
- **Stepped Trace Widths & Angles:** Narrow exits step up to wider widths in open corridors where clearance is satisfied, avoiding sharp acute angles (`< 90°`).
- **Verification:** Scripts are syntax-checked (`python3 -m py_compile`) and paired with DRC checks and copper zone refills (`B`) in KiCad.
