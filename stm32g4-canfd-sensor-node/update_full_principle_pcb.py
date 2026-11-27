import datetime, pathlib, re
pcb = pathlib.Path("stm32g4-canfd-sensor-node.kicad_pcb")
md = pathlib.Path("full_principle_pcb.md")
now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
pads = segs = vias = zones = nets = 0
balanced = True
if pcb.exists():
    t = pcb.read_text(encoding="utf-8", errors="ignore")
    balanced = (t.count("(") == t.count(")"))
    pads = len(re.findall(r"\(pad\b", t))
    segs = len(re.findall(r"\(segment\b", t))
    vias = len(re.findall(r"\(via\b", t))
    zones = len(re.findall(r"\(zone\b", t))
    nets = len([m for m in re.finditer(r"\(\s*net\s+(\d+)\s+\"([^\"]+)\"\s*\)", t) if int(m.group(1)) > 0])
scripts = sorted([p.name for p in pathlib.Path(".").glob("*.py")])
backups_cnt = len(list(pathlib.Path("backups").glob("*"))) if pathlib.Path("backups").exists() else 0
content = f"""# Full Principle PCB Design & Automated Workflow (`stm32g4-canfd-sensor-node`)

Last Updated: `{now_str}`  
Board File: `stm32g4-canfd-sensor-node.kicad_pcb` (`balanced={balanced}`)

---

## 1. Core Hardware & Stackup Architecture
- **Target Board:** STM32G4 CAN-FD Sensor Node (`80.0 mm x 60.0 mm` outline).
- **4-Layer Stackup:**
  - `F.Cu`: Primary component placement and signal/power routing.
  - `In1.Cu`: Dedicated `GND` reference plane.
  - `In2.Cu`: Dedicated power plane (`+3V3` / `24V_PROT`).
  - `B.Cu`: Secondary routing and ground fill.
- **Design & Routing Constraints:**
  - Fine-pitch SMD routing width: `0.150 mm` (for dense areas around `D3`, `R7`, `C17`, `C22`) and `0.200 mm` default.
  - Minimum clearance: `0.150 mm`.
  - Grid step: `0.200 mm`.

---

## 2. Automated Terminal-First & VS Code Engineering Rules
1. **Zero Manual GUI Boilerplate:** All structural PCB edits, footprint swaps, net assignments, silkscreen/Fab cleanup, and DRC audits are executed via deterministic Python scripts in the terminal or VS Code.
2. **Strict KiCad S-Expression Integrity:**
   - Every edit verifies balanced parentheses (`count('(') == count(')')`).
   - Every net reference uses valid KiCad 10 syntax: `(net <id> "<net_name>")` where `<id> > 0` and `<net_name>` is non-empty.
   - `pcbnew` must be closed (`pgrep -af '[p]cbnew'`) before running file-modifying scripts, with timestamped snapshots saved in `backups/`.
3. **VS Code & Copilot / Agent Execution Standard:**
   - Keep project scripts modular, idempotent, and self-verifying.
   - Always run `python3 run_drc_check.py` after any modification and refill copper zones (`B` in KiCad) when `zones > 0`.

---

## 3. Completed Milestones & Applied Fixes (Up to Pre-Routing Baseline)
1. **Clean Layout Baseline Restoration:**
   - Restored the clean unrouted placement baseline (`Q1` preserved as `Package_TO_SOT_SMD:SOT-23` at `X=117.5 mm, Y=78.5 mm`, `F.Fab` hidden, and duplicate reference labels cleaned).
2. **Component Package & Footprint Standardization (`D3`):**
   - Replaced `D3` (`12V` Zener) with 2-pin SMD footprint `Diode_SMD:D_SOD-123` positioned at `X=126.0 mm, Y=99.8 mm`.
3. **DRC Project Rules & Exclusions (`.kicad_pro`):**
   - Configured `.kicad_pro` DRC `ignore_tests` (`footprint_link_issues`, `lib_footprint_mismatch`, `footprint_mismatch`) to suppress non-blocking library mismatch warnings on customized footprints.
4. **Net Assignment & Short-Circuit Resolution (`D3`, `R7`, `GND`, `NRST`):**
   - Diagnosed `Items shorting two nets` and `Front solder mask aperture bridges items with different nets` caused by `<no net>` pads meeting routed tracks.
   - Assigned `Net-(D3-K)` to `D3` Pad 1 (connecting toward `R7`) and `GND` to `D3` Pad 2.
   - Identified and unified `NRST` across `SW1` (Pad 2), `U2` (Pad 7), `J2` (Pad 5), and `C17` (Pad 2).
5. **Interactive Router Calibration for Manual Routing:**
   - Set Interactive Router mode to `Highlight collisions` with `0.150 mm` track width and waypoint placement to prevent router pushback around tight courtyards.
6. **Pre-Routing Snapshot & Verification:**
   - Saved verified pre-routing backup inside `backups/` (`bk_before_routing_*.kicad_pcb`).

---

## 4. Current Board Telemetry & Workspace Scripts
- **Board Stats:** `pads={pads}` | `segments={segs}` | `vias={vias}` | `zones={zones}` | `named_nets={nets}` | `balanced={balanced}`
- **Backup Archive Count:** `{backups_cnt}` files in `backups/`
- **Active Project Scripts:** `{chr(44).join(scripts)}`

---

## 5. Next Execution Phase: Systematic Trace Routing
1. Open `stm32g4-canfd-sensor-node.kicad_pcb` in KiCad PCB Editor and press `B` to refill copper zones.
2. Route critical control and reset nets first (`NRST` across `SW1`, `C17`, `U2`, `J2`), followed by CAN-FD differential pairs, power rails, and remaining signal nets.
3. Save (`Ctrl+S`) frequently and validate each routing milestone with `python3 run_drc_check.py`.
"""
md.write_text(content, encoding="utf-8")
print(f"UPDATED_FULL_PRINCIPLE_PCB_OK | path={md} | bytes={md.stat().st_size} | balanced={balanced}")
