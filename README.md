# Industrial DAQ Sensor 24V

4-layer industrial sensor and data acquisition (DAQ) node designed in KiCad around **STM32G431CBUx**, powered from a **24V** supply via **TPS54302** buck regulator, featuring **CAN FD** (**TCAN332**), **4–20mA** analog input, and **TMP101** + **LIS3DH** sensors.

## Completed Work
- **Schematic & Protection:** Finalized STM32G431CBUx schematic with TPS54302 (24V step-down), SMBJ28A TVS protection, Zener clamps, TCAN332 CAN FD transceiver, 4–20mA analog input conditioning, TMP101 temperature sensor, and LIS3DH accelerometer.
- **Footprints & Mechanical Design:** Assigned SMD footprints, added isolated M3 mounting holes (`MountingHole_3.2mm_M3_Pad` unconnected to nets to prevent ground loops), and placed fiducial markers.
- **4-Layer PCB Setup & Initial Layout:** Configured 4-layer stackup (`F.Cu`, `In1.Cu` for `GND`, `In2.Cu` for power rails, `B.Cu`), set Design Rules (`Constraints` and `Pre-defined Sizes`), and started `Edge.Cuts` and component placement in KiCad PCB Editor.
