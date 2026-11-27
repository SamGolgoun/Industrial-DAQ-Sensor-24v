#!/usr/bin/env bash
# Usage (from the board folder):  bash run_all.sh
# Optional: KICAD_CLI=/path/to/kicad-cli bash run_all.sh
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCB="${PCB:-stm32g4-canfd-sensor-node.kicad_pcb}"
KICAD_CLI="${KICAD_CLI:-$HOME/Downloads/kicad-10.0.7-x86_64.AppImage (1)/squashfs-root/usr/bin/kicad-cli}"

drc() {   # $1 = output report
  local extra=()
  if "$KICAD_CLI" pcb drc --help 2>&1 | grep -q -- '--refill-zones'; then extra+=(--refill-zones); fi
  "$KICAD_CLI" pcb drc --format report --severity-all --all-track-errors "${extra[@]}" \
      --output "$1" "$PCB" 2>&1 | grep -v 'LD_PRELOAD' || true
}

echo "== Close KiCad (PCB editor) before continuing, or it will overwrite the changes =="
cp "$PCB" "$PCB.bak-before-run_all"
echo "== DRC before =="
drc drc_before.rpt
python3 "$HERE/fix_board.py" --pcb "$PCB" --report drc_before.rpt "$@"
# note: /OSC_IN could not be auto-routed on the real board (see fix_board.py --help, --ripup)
echo "== DRC after =="
drc drc_after.rpt
echo "-- summary after --"
grep -E "^\[" drc_after.rpt | sed 's/:.*//' | sort | uniq -c | sort -rn
echo "-- real errors (should be empty) --"
grep -B1 -A3 "; error" drc_after.rpt || echo "none"
