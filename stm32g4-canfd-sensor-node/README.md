# Industrial DAQ Sensor Node (24V) — STM32G4 CAN-FD PCB Design

## Overview & Purpose
This project designs a **24V industrial sensor and data acquisition (DAQ) node** in **KiCad** built around the **STM32G431CBUx (UFQFPN48)** microcontroller and a **TCAN332 CAN-FD transceiver**[cite: 7]. It is designed for industrial sensing and reliable CAN-FD communication in noisy environments[cite: 7].

## Current Status (Schematic & AI-Assisted PCB Routing)
- **Schematic:** The circuit schematic is finalized and included in the repository[cite: 7].
- **PCB Layout (In Progress):** The `.kicad_pcb` layout is actively being routed and refined using an **AI-assisted, code-driven workflow** in **VS Code** with **Python** scripts rather than manual GUI-only editing.

## Key Engineering Features & Design Rules
- **Code-Driven & AI-Assisted Workflow:** Routing operations, geometry checks, net/pin inspections, and rule gates are scripted in Python inside VS Code with AI assistant tools for deterministic, reproducible layout changes.
- **Safety & Pre-Write Rule Gates:**
  - Timestamped backups are created prior to modifying the `.kicad_pcb` file.
  - Pre-write checks enforce component and courtyard clearance, forbid routing over component bodies or courtyards, prevent unintended `GND` connections or shorts, and verify parenthesis balance.
- **Stepped Trace Widths & Angle Discipline:**
  - Tight fan-out/exit segments start at `0.15 mm` and step up to `0.30 mm` in open corridors where clearance is satisfied.
  - Acute/sharp trace angles (`< 90°`) are avoided in favor of clean, standard routing angles.
- **Verification:** Scripts are syntax-checked (`python3 -m py_compile`) and paired with DRC checks and copper zone refills (`B`) in KiCad.
