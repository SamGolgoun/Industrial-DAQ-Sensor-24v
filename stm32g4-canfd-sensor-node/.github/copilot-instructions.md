# KiCad PCB debugging and routing

- Auto-discover `kicad-cli` and bundled `python3.11` beneath `/tmp/.mount_kicad*/usr/bin/`.
- Run DRC with `--refill-zones --save-board`.

- Keep edits and reports targeted. Print only concise PASS/FAIL results; never dump the full PCB or DRC report.
- Before writing, check for `~*.kicad_pcb.lck`. If present, do not edit, save, move, or route the board; ask the user to close KiCad before continuing.
- After any placement or routing edit, run `scripts/verify_kicad_board.py` and fix failures before calling the board verified.
- Use bounded routing attempts. Do not run PNS/shove without a short timeout; prefer deterministic routes on a selected copper layer and continue to other candidates after a failed route.
- Preserve connector access, mounting holes, courtyard clearance, and the board edge. Rotate/move a footprint only after checking its pads, courtyard, mechanical access, and affected nets.
- Route on F.Cu, In1.Cu, In2.Cu, and B.Cu where clear paths exist. Use 0.15 mm escapes, about 0.30 mm routine tracks, and 0.40–0.60 mm power tracks where measured clearance permits.
- Use standard through-vias only in open copper; never place a via in a pad. Refill GND zones and verify no isolated GND islands.
- Do not report completion unless the verifier shows zero unconnected items and zero copper clearance, short, and track-width DRC errors.
