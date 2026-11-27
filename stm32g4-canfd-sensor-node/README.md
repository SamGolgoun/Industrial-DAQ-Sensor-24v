# STM32G4 CAN-FD Sensor Node — Code-Driven KiCad PCB Design

This repository contains the KiCad PCB layout (`stm32g4-canfd-sensor-node.kicad_pcb`) and a **code-driven Python automation workflow** developed and executed in **VS Code** with **AI assistant tools**.

## Workflow & Engineering Approach
- **Code-Driven Layout & Automation:** Routing updates, net inspections, backup creation, and rule checks are driven deterministically via **Python** scripts in **VS Code** alongside **AI assistant tools** rather than manual GUI-only edits.
- **Safety & Pre-Write Rule Gates:**
  - Timestamped backups are created before modifying `.kicad_pcb`.
  - Pre-write checks enforce strict component and courtyard clearance, prohibit routing over component bodies or courtyards, prevent unintended `GND` connections or shorts, and verify parenthesis balance (`count("(") == count(")")`).
  - Stepped trace widths (`0.15 mm` pin exits stepping to `0.30 mm` in open space where clearance allows) and IPC-aligned angles are applied per stage.
- **Validation & Documentation:** Scripts are syntax-checked (`python3 -m py_compile`) and paired with DRC/delta checks and KiCad copper zone refill (`B`) and inspection.
